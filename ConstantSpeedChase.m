function ConstantSpeedChase()
    % Initial conditions
    fox_init = [300, -550];    % Initial position of fox (F)
    rabbit_init = [0, 0];      % Initial position of rabbit (R)
    burrow = [600, 600];       % Position of burrow (B)
    sw_corner = [200, -400];   % SW corner of warehouse
    nw_corner = [200, 0];      % NW corner of warehouse
    
    sf0 = 15;     % Fox speed (m/s)
    sr0 = 11;     % Rabbit speed (m/s)
    
    % Initial state vector [fox_x, fox_y, rabbit_x, rabbit_y]
    y0 = [fox_init(1), fox_init(2), rabbit_init(1), rabbit_init(2)];
    
    % Global variable to track if fox has reached SW corner
    global has_reached_sw;
    has_reached_sw = false;
    
    % Set options for ODE solver
    options = odeset('Events', @(t,y) captureOrEscape(t,y,burrow), ...
                    'RelTol', 1e-8, 'AbsTol', 1e-8);
    
    % Solve ODE
    [t, y, te, ye, ie] = ode45(@(t,y) chase(t,y,sf0,sr0,sw_corner,nw_corner,burrow), ...
                              [0, inf], y0, options);
    
    % Plot results
    figure;
    % Create plot handles for legend control
    h_fox = plot(y(:,1), y(:,2), 'r-', 'LineWidth', 2);  % Fox path
    hold on;
    h_rabbit = plot(y(:,3), y(:,4), 'b-', 'LineWidth', 2);  % Rabbit path
    h_corners = plot(sw_corner(1), sw_corner(2), 'ks', 'MarkerSize', 10);  % SW corner
    plot(nw_corner(1), nw_corner(2), 'ks', 'MarkerSize', 10);  % NW corner

    % Draw warehouse boundaries
    xlim_val = 800;  % Set x-axis range for extended walls
    % Vertical wall
    h_walls = plot([sw_corner(1), nw_corner(1)], [sw_corner(2), nw_corner(2)], 'k-', 'LineWidth', 2);
    % Extended walls to east
    plot([sw_corner(1), xlim_val], [sw_corner(2), sw_corner(2)], 'k-', 'LineWidth', 2);  % SW extension
    plot([nw_corner(1), xlim_val], [nw_corner(2), nw_corner(2)], 'k-', 'LineWidth', 2);  % NW extension

    % Plot burrow with circle marker
    h_burrow = plot(burrow(1), burrow(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % Burrow with circle

    % Create legend with specific line colors and markers
    legend([h_fox, h_rabbit, h_corners, h_walls, h_burrow], ...
           {'Fox Path', 'Rabbit Path', 'Warehouse Corners', 'Warehouse Walls', 'Burrow'}, ...
           'Location', 'best');

    grid on;
    xlim([min(fox_init(1)-50, 0), xlim_val]);
    ylim([min(sw_corner(2)-50), max(burrow(2)+50, 600)]);
    
% Calculate and display results
    T = t(end);
    final_fox_pos = [y(end,1), y(end,2)];
    final_rabbit_pos = [y(end,3), y(end,4)];
    distance_traveled = calculateDistance(y(:,1:2));
    
    % Determine if rabbit escaped or was captured
    capture_dist = norm(final_fox_pos - final_rabbit_pos);
    escape_dist = norm(final_rabbit_pos - burrow);
    
    fprintf('\nSimulation Results:\n');

    if escape_dist <= 0.1
        fprintf('Rabbit successfully escaped to the burrow!\n');
    elseif capture_dist <= 0.1
        fprintf('Fox caught the rabbit.\n');
    end

    fprintf('Time T: %.2f seconds\n', T);
    fprintf('Final fox position: (%.2f, %.2f)\n', final_fox_pos(1), final_fox_pos(2));
    fprintf('Distance traveled by fox: %.2f meters\n', distance_traveled);
    
end

function dydt = chase(t, y, sf0, sr0, sw_corner, nw_corner, burrow)
    % Extract positions
    fox = [y(1), y(2)];
    rabbit = [y(3), y(4)];
    
    % Calculate rabbit direction (always straight to burrow)
    rabbit_dir = burrow - rabbit;
    rabbit_dir = rabbit_dir / norm(rabbit_dir);
    
    % Determine fox direction based on visibility and position
    fox_dir = getFoxDirection(fox, rabbit, sw_corner, nw_corner);
    
    % Set velocities
    dydt = zeros(4,1);
    dydt(1:2) = sf0 * fox_dir';  % Convert to column vector
    dydt(3:4) = sr0 * rabbit_dir';  % Convert to column vector
end

function fox_dir = getFoxDirection(fox, rabbit, sw_corner, nw_corner)
    global has_reached_sw;
    
    if isRabbitVisible(fox, rabbit, sw_corner, nw_corner)
        % Direct chase if rabbit is visible
        fox_dir = rabbit - fox;
    else
        % If rabbit is not visible
        if ~has_reached_sw && ~isAtPoint(fox, sw_corner)
            % Move to SW corner if not reached before
            fox_dir = sw_corner - fox;
        else
            % Mark as reached if at SW corner and move northward
            if isAtPoint(fox, sw_corner)
                has_reached_sw = true;
            end
            fox_dir = [0, 1];  % North direction unit vector
        end
    end
    fox_dir = fox_dir / norm(fox_dir);
end

function visible = isRabbitVisible(fox, rabbit, sw_corner, nw_corner)
    % Check if rabbit is blocked by warehouse boundaries
    
    % Check vertical wall
    if rabbit(1) >= sw_corner(1) && fox(1) < sw_corner(1)
        if lineIntersectsWarehouse(fox, rabbit, sw_corner, nw_corner)
            visible = false;
            return;
        end
    end
    
    % Check SW extension
    if fox(2) < sw_corner(2) && rabbit(2) > sw_corner(2) && ...
       min(fox(1), rabbit(1)) <= sw_corner(1)
        visible = false;
        return;
    end
    
    % Check NW extension
    if fox(2) > nw_corner(2) && rabbit(2) < nw_corner(2) && ...
       min(fox(1), rabbit(1)) <= nw_corner(1)
        visible = false;
        return;
    end
    
    % Special check: at SW corner, check if rabbit is west of warehouse
    if isAtPoint(fox, sw_corner) && rabbit(1) <= sw_corner(1)
        visible = false;
        return;
    end
    
    visible = true;
end

function intersects = lineIntersectsWarehouse(p1, p2, sw_corner, nw_corner)
    % Check if line segment intersects with warehouse wall
    x1 = p1(1); y1 = p1(2);
    x2 = p2(1); y2 = p2(2);
    x3 = sw_corner(1); y3 = sw_corner(2);
    x4 = nw_corner(1); y4 = nw_corner(2);
    
    denominator = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if denominator == 0
        intersects = false;
        return;
    end
    
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denominator;
    u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / denominator;
    
    intersects = (t >= 0 && t <= 1 && u >= 0 && u <= 1);
end

function [value, isterminal, direction] = captureOrEscape(t, y, burrow)
    fox = [y(1), y(2)];
    rabbit = [y(3), y(4)];
    
    % Check capture (distance <= 0.1 meters)
    capture_dist = norm(fox - rabbit);
    % Check escape (reached burrow)
    escape_dist = norm(rabbit - burrow);
    
    value = min(capture_dist - 0.1, escape_dist - 0.1);
    isterminal = 1;  % Stop integration
    direction = 0;   % All crossings
end

function dist = calculateDistance(path)
    % Calculate total distance traveled from path points
    diff_vec = diff(path);
    segments = sqrt(sum(diff_vec.^2, 2));
    dist = sum(segments);
end

function at_point = isAtPoint(pos, point)
    % Check if position is very close to a point
    at_point = norm(pos - point) < 0.1;
end
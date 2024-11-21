# Fox and Rabbit Pursuit Simulation

This project simulates a pursuit-evasion scenario between a fox and a rabbit around a warehouse obstacle. The simulation is implemented in MATLAB and includes two variants: constant speeds and diminishing speeds due to fatigue.

## Problem Description
- A fox tries to catch a rabbit while navigating around a warehouse obstacle
- The rabbit attempts to reach its burrow while avoiding capture
- The warehouse creates a line-of-sight obstacle that affects the fox's pursuit strategy

## Files
- `ConstantSpeedChase.m`: Implementation with constant speeds
- `DiminishingSpeedChase.m`: Implementation with speed reduction due to fatigue

## Key Features
### Constant Speed Model
- Fox speed: 15 m/s
- Rabbit speed: 11 m/s
- Visibility-based pursuit strategy
- Warehouse collision detection
- Path tracking and visualization

### Diminishing Speed Model
- Initial fox speed: 15 m/s with decay rate μf = 0.0002 m⁻¹
- Initial rabbit speed: 11 m/s with decay rate μr = 0.0008 m⁻¹
- Speed reduction based on distance traveled
- Same pursuit strategy as constant speed model
- Animals will apply diminishing speed starting from the time they find each other and start running

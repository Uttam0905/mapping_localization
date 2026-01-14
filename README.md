# mapping_localization
## Overview

This project implements a complete autonomous navigation pipeline for a differential-drive mobile robot using ROS 2, SLAM Toolbox, Nav2, and Gazebo.

The system supports:

 * online map creation,

 * localization on a saved map,

 * goal-based navigation,

 * manual teleoperation with safe command arbitration.

The project emphasizes correct TF design, modular launch architecture, and reproducible simulation workflows, following real-world robotics system practices.

## Key Capabilities
### Online Mapping

 * Real-time SLAM using SLAM Toolbox in mapping mode.

 * LiDAR-based occupancy grid generation.

 * Map saving for later reuse.

### Localization

 * Localization using SLAM Toolbox localization mode on a pre-built map.

 * Robust pose estimation without AMCL.

 * Continuous map → odom → base TF correction.

### Autonomous Navigation

 * Global path planning and local control using Nav2.

 * Goal-based navigation in a known map.

 * Recovery behaviors handled by Nav2 lifecycle nodes.

### Command Arbitration

 * twist_mux used to arbitrate between:

  * Nav2 autonomous commands

  * Teleoperation keyboard input

 * Manual override has higher priority and safely preempts navigation.

### Simulation Environment

 * Gazebo-based differential-drive robot.

 * 2D LiDAR sensor for perception.

 * Custom world files for testing navigation scenarios.

## Modes of Operation
### Mapping Mode

 * SLAM Toolbox runs in mapping mode.

 * Robot is driven manually.

 * Occupancy grid map is built and saved.

### Localization Mode

 * SLAM Toolbox runs in localization mode.

 * Saved map is loaded.

 * Robot localizes itself in the environment.

### Navigation Mode

 * Nav2 is launched on top of localization.

 * User sends goal poses from RViz.

 * Robot plans and executes paths autonomously.

## How to Run

1️⃣ Mapping
~~~
ros2 launch mapping_robot mapping.launch.py
~~~
Drive the robot manually and save the map.

2️⃣ Localization
~~~
ros2 launch mapping_robot localization.launch.py
~~~

Verify robot pose aligns with the map in RViz.

3️⃣ Navigation
~~~
ros2 launch mapping_robot navigation.launch.py
~~~

Send goals from RViz.

4️⃣ Teleoperation Override
~~~
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_teleop
~~~

Manual control safely overrides Nav2.

## Design Decisions

### SLAM Toolbox localization instead of AMCL
 * Simplifies the stack while maintaining robust localization.

### Separated launch stages
 * Prevents race conditions and lifecycle failures.

### twist_mux for command arbitration
 * Ensures safe interaction between autonomy and teleoperation.

### Simulation-first validation
 * Allows reproducible testing and debugging.

## Limitations

 * Single-robot mapping (multi-robot SLAM not addressed).

 * Reactive obstacle avoidance handled by Nav2 local planner only.

 * Simulation-based evaluation (no hardware deployment).

## Future Extensions

 * Multi-robot map merging.

 * Advanced local planners and costmap tuning.

 * Integration with real hardware platforms.

 * Quantitative localization error evaluation.

## License

MIT License

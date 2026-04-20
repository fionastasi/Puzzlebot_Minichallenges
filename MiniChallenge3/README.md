# Puzzlebot_Minichallenges

This repository contains different minichallenges using ROS for Puzzlebot navigation with Lidar, RViz, and Gazebo.

## Mini Challenge 2 Plan

This branch implements a complete ROS2 pipeline for Simulation, Localisation, and Control using a differential-drive kinematic model.

### Robot Parameters

- Wheel radius (r): 0.05 m
- Wheelbase (l): 0.19 m

Kinematic model:

- x_dot = v * cos(theta)
- y_dot = v * sin(theta)
- theta_dot = omega

Wheel-speed relations:

- v = (r / 2) * (omega_r + omega_l)
- omega = (r / l) * (omega_r - omega_l)

### Phase Planning

1. Simulation:
- Build a pure kinematic simulator node (no Gazebo required).
- Publish simulated pose and wheel angular velocities.
- Validate with constant and varying linear/angular velocity commands.

2. Localisation:
- Implement dead-reckoning odometry from wheel velocities.
- Publish robot pose as nav_msgs/Odometry.
- Compare simulated pose vs odometry estimate in RViz and rqt_plot.

3. Control:
- Implement waypoint/trajectory generation (square, pentagon, circle).
- Implement controller to track the target trajectory.
- Validate trajectory tracking performance in RViz.

### Execution Order

1. ROS2 package structure and configuration.
2. URDF and meshes integration for RViz visualization.
3. Kinematic simulator node implementation.
4. Localization node implementation.
5. Trajectory generator + controller implementation.
6. Launch files and RViz configuration.
7. Validation with topic inspection and rqt_plot.

## Implemented Nodes

| Node | Role | Subscribes | Publishes |
|---|---|---|---|
| puzzlebot_sim2 | Differential-drive kinematic simulator | cmd_vel (geometry_msgs/Twist) | pose_sim (geometry_msgs/PoseStamped), wr (std_msgs/Float32), wl (std_msgs/Float32) |
| localization | Dead-reckoning odometry estimator | wr (std_msgs/Float32), wl (std_msgs/Float32) | odom (nav_msgs/Odometry), pose_odom (geometry_msgs/PoseStamped) |
| trajectory_generator | Polygon waypoint generator | None | target_pose (geometry_msgs/PoseStamped) |
| controller | PD tracking controller | odom (nav_msgs/Odometry), target_pose (geometry_msgs/PoseStamped) | cmd_vel (geometry_msgs/Twist) |

## Available Launch Files

- launch/simulation.launch.py: simulator + robot_state_publisher + RViz
- launch/localization.launch.py: simulator + localization + robot_state_publisher + RViz
- launch/control.launch.py: full pipeline (simulation + localisation + trajectory + control)
- launch/all.launch.py: alias for full system launch

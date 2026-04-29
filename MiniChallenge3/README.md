# Puzzlebot Sim Multi-Robot Refactor

This package was refactored to support two independent Puzzlebot robots in ROS 2 using namespaces, relative topics, and namespaced TF frames.

The goal was to move from a single-robot demo to a clean multi-robot launch setup where each robot has its own control, localization, kinematic simulation, and TF tree without frame collisions.

## What Changed

### 1. Reusable single-robot launch

Created [launch/robot.launch.py](launch/robot.launch.py) as the reusable launch entry point for one robot.

It accepts these arguments:

- `robot_name`
- `x0`
- `y0`
- `theta0`
- `goal_x`
- `goal_y`

Why this was needed:

- The same launch logic can now be reused for any robot instance.
- Each robot can receive its own namespace, initial pose, and goal without duplicating launch code.

### 2. Multi-robot main launch

Updated [launch/combined.launch.py](launch/combined.launch.py) so it launches two copies of the reusable robot launch file with different parameters.

Why this was needed:

- `robot1` and `robot2` must be launched independently.
- Each robot needs a different starting pose and a different goal.
- `IncludeLaunchDescription` keeps the setup modular and easier to maintain.

### 3. Relative topics everywhere

Refactored the main nodes to use relative topics instead of absolute ones.

Affected nodes:

- [puzzlebot_sim/control.py](puzzlebot_sim/control.py)
- [puzzlebot_sim/kinematic_model.py](puzzlebot_sim/kinematic_model.py)
- [puzzlebot_sim/localisation.py](puzzlebot_sim/localisation.py)
- [puzzlebot_sim/URDF_tfs.py](puzzlebot_sim/URDF_tfs.py)
- [puzzlebot_sim/robot_markers.py](puzzlebot_sim/robot_markers.py)

Examples of the change:

- `/cmd_vel` became `cmd_vel`
- `/odom` became `odom`
- `/wr` and `/wl` became `wr` and `wl`

Why this was needed:

- Absolute topics ignore namespaces and cause robots to share the same communication channels.
- Relative topics let ROS 2 automatically remap everything under `robot1` and `robot2`.

### 4. Parameters instead of hardcoded values

The simulation and controller nodes now receive values through parameters.

Parameters used:

- `x0`, `y0`, `theta0` for initial pose
- `goal_x`, `goal_y` for control goals

Why this was needed:

- Each robot can start in a different place.
- Each robot can move toward a different target.
- The code becomes reusable and easier to launch multiple times.

### 5. TF frames are namespaced

The TF broadcaster now prefixes frames with the robot namespace.

Examples:

- `robot1/odom`
- `robot1/base_footprint`
- `robot1/base_link`
- `robot2/odom`
- `robot2/base_footprint`
- `robot2/base_link`

Why this was needed:

- TF collisions happen when both robots publish the same frame names.
- RViz and tf2 need unique frame trees for each robot.

### 6. robot_state_publisher is also prefixed

The launch file uses `frame_prefix` in `robot_state_publisher` so the URDF links are published under the robot namespace.

Why this was needed:

- The URDF still defines the robot structure once.
- `frame_prefix` lets the same URDF be reused for both robots without editing the model.

### 7. robot_markers was made namespace-aware

Updated [puzzlebot_sim/robot_markers.py](puzzlebot_sim/robot_markers.py) so it does not publish global TF frame names.

Why this was needed:

- Even if it is not part of the main launch flow, it would create the same TF conflict if launched later.

## Why These Changes Fix the Problem

The original issue was that multiple nodes were still publishing shared frame names like:

- `odom`
- `base_link`
- `base_footprint`

That created TF conflicts, incomplete models in RViz, and one robot overwriting the other.

With this refactor:

- each robot has its own namespace
- each robot uses relative topics
- each robot publishes its own TF tree
- each robot has its own initial pose and goal

## Launch Architecture

The final launch flow is:

1. `combined.launch.py`
2. includes `robot.launch.py` for `robot1`
3. includes `robot.launch.py` for `robot2`
4. each instance launches:
- `robot_state_publisher`
- `URDF_tfs`
- `kinematic_model`
- `localisation`
- `control`

## How To Run

From the workspace root:

```bash
colcon build --packages-select puzzlebot_sim
source install/setup.bash
ros2 launch puzzlebot_sim combined.launch.py
```

## RViz Notes

To visualize the robots correctly:

- Set the Fixed Frame to `robot1/odom` or `robot2/odom`
- Make sure both namespaces are visible in TF
- If needed, add the robot models manually in RViz

## Expected ROS 2 Topics

You should see namespaced topics like:

- `/robot1/cmd_vel`
- `/robot1/odom`
- `/robot1/wr`
- `/robot1/wl`
- `/robot2/cmd_vel`
- `/robot2/odom`
- `/robot2/wr`
- `/robot2/wl`

There should not be a shared global `/odom` topic for the robots.

## Expected TF Frames

The TF tree should contain separate frames for each robot, for example:

- `robot1/map`
- `robot1/odom`
- `robot1/base_footprint`
- `robot1/base_link`
- `robot2/map`
- `robot2/odom`
- `robot2/base_footprint`
- `robot2/base_link`

## Notes

This refactor keeps the code modular and avoids external dependencies. The same package can now be extended to more robots by reusing [launch/robot.launch.py](launch/robot.launch.py) with different arguments.

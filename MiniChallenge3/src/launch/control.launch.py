"""
Launch file for Complete Control System
Starts:
  - puzzlebot_sim_node (kinematic simulator)
  - localization_node (dead reckoning)
  - trajectory_generator_node (waypoint generator)
  - controller_node (PD controller)
  - robot_state_publisher (for TF and visualization)
  - rviz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('puzzlebot_sim2')
    
    # Paths to config files
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'puzzlebot.rviz')
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    trajectory_shape_arg = DeclareLaunchArgument(
        'trajectory_shape',
        default_value='square',
        description='Trajectory shape: square, pentagon, circle'
    )
    
    trajectory_size_arg = DeclareLaunchArgument(
        'trajectory_size',
        default_value='1.0',
        description='Trajectory size in meters'
    )
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': open(urdf_file).read()}
        ]
    )
    
    # Puzzlebot Simulator node
    sim_node = Node(
        package='puzzlebot_sim2',
        executable='puzzlebot_sim2',
        name='puzzlebot_sim_node',
        output='screen',
    )
    
    # Localization node
    loc_node = Node(
        package='puzzlebot_sim2',
        executable='localization',
        name='localization_node',
        output='screen',
    )
    
    # Trajectory Generator node
    traj_node = Node(
        package='puzzlebot_sim2',
        executable='trajectory_generator',
        name='trajectory_generator_node',
        output='screen',
        parameters=[
            {
                'trajectory_shape': LaunchConfiguration('trajectory_shape'),
                'trajectory_size': LaunchConfiguration('trajectory_size'),
            }
        ]
    )
    
    # Controller node
    ctrl_node = Node(
        package='puzzlebot_sim2',
        executable='controller',
        name='controller_node',
        output='screen',
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        trajectory_shape_arg,
        trajectory_size_arg,
        robot_state_publisher,
        sim_node,
        loc_node,
        traj_node,
        ctrl_node,
        rviz_node,
    ])

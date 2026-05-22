from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('minichallenge5')
    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_name = LaunchConfiguration('robot_name')
    x0 = LaunchConfiguration('x0')
    y0 = LaunchConfiguration('y0')

    robot_group = GroupAction([
        PushRosNamespace(robot_name),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc, 
                'use_sim_time': True
            }],
            remappings=[('joint_states', '/joint_states')] 
        ),

        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='urdf_spawner',
                    output='screen',
                    arguments=[
                        '-file', urdf_file,
                        '-entity', robot_name,
                        '-x', x0,
                        '-y', y0,
                        '-z', '0.15'
                    ]
                )
            ]
        ),

        Node(
            package='minichallenge5',
            executable='localisation',
            name='pose_estimator',
            parameters=[{'initial_x': 0.0, 'initial_y': 0.0}]
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('x0', default_value='0.0'),
        DeclareLaunchArgument('y0', default_value='0.0'),
        robot_group
    ])

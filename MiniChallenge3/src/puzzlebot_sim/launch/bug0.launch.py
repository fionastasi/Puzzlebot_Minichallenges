from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_sim')
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'display.launch.py'))
    )

    kinematic_node = Node(
        package='puzzlebot_sim',
        executable='kinematic_model',
        name='kinematic_model',
        output='screen'
    )

    localisation_node = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation',
        output='screen'
    )

    bug0_node = Node(
        package='puzzlebot_sim',
        executable='bug0',
        name='bug0',
        output='screen',
        parameters=[
            {'goal_x': 2.0},
            {'goal_y': 0.0}
        ]
    )

    return LaunchDescription([
        display_launch,
        kinematic_node,
        localisation_node,
        bug0_node
    ])
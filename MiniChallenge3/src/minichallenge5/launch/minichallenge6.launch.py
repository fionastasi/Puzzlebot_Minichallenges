from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('minichallenge5')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'display.launch.py')
        )
    )

    kinematic_model_node = Node(
        package='minichallenge5',
        executable='kinematic_model',
        name='kinematic_model',
        output='screen'
    )

    localisation_node = Node(
        package='minichallenge5',
        executable='localisation',
        name='localisation',
        output='screen'
    )

    return LaunchDescription([
        display_launch,
        kinematic_model_node,
        localisation_node
    ])

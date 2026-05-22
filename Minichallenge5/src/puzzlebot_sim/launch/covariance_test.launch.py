from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('puzzlebot_sim')

    urdf_file = os.path.join(pkg_path, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'setup.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    urdf_tfs_node = Node(
        package='puzzlebot_sim',
        executable='URDF_tfs',
        name='URDF_tfs',
        output='screen'
    )

    kinematic_model_node = Node(
        package='puzzlebot_sim',
        executable='kinematic_model',
        name='kinematic_model',
        output='screen'
    )

    localisation_node = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[
            {'kr': 0.01},
            {'kl': 0.01}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        urdf_tfs_node,
        kinematic_model_node,
        localisation_node,
        rviz_node
    ])
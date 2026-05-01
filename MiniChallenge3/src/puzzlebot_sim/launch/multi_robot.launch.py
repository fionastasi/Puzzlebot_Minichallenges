from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def create_robot_nodes(robot_name, initial_x, initial_y, initial_theta, offset_x, offset_y, robot_desc):
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'frame_prefix': f'{robot_name}/'}
            ]
        ),

        Node(
            package='puzzlebot_sim',
            executable='URDF_tfs',
            namespace=robot_name,
            name='URDF_tfs',
            output='screen',
            parameters=[
                {'tf_prefix': robot_name}
            ]
        ),

        Node(
            package='puzzlebot_sim',
            executable='kinematic_model',
            namespace=robot_name,
            name='kinematic_model',
            output='screen',
            parameters=[
                {'initial_x': initial_x},
                {'initial_y': initial_y},
                {'initial_theta': initial_theta}
            ]
        ),

        Node(
            package='puzzlebot_sim',
            executable='localisation',
            namespace=robot_name,
            name='localisation',
            output='screen',
            parameters=[
                {'initial_x': initial_x},
                {'initial_y': initial_y},
                {'initial_theta': initial_theta},
                {'tf_prefix': robot_name}
            ]
        ),

        Node(
            package='puzzlebot_sim',
            executable='control',
            namespace=robot_name,
            name='control',
            output='screen',
            parameters=[
                {'offset_x': offset_x},
                {'offset_y': offset_y}
            ]
        ),
    ]


def generate_launch_description():
    pkg_path = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg_path, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(pkg_path, 'rviz', 'setup.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot1_nodes = create_robot_nodes(
        robot_name='robot1',
        initial_x=0.0,
        initial_y=0.0,
        initial_theta=0.0,
        offset_x=0.0,
        offset_y=0.0,
        robot_desc=robot_desc
    )

    robot2_nodes = create_robot_nodes(
        robot_name='robot2',
        initial_x=2.0,
        initial_y=0.0,
        initial_theta=0.0,
        offset_x=2.0,
        offset_y=0.0,
        robot_desc=robot_desc
    )

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription(
        robot1_nodes +
        robot2_nodes +
        [rviz_node]
    )
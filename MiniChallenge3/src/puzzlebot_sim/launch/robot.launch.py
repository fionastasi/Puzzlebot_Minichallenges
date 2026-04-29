from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg_path, 'urdf', 'puzzlebot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot1')
    x0_arg = DeclareLaunchArgument('x0', default_value='0.0')
    y0_arg = DeclareLaunchArgument('y0', default_value='0.0')
    theta0_arg = DeclareLaunchArgument('theta0', default_value='0.0')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='1.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')

    robot_name = LaunchConfiguration('robot_name')
    frame_prefix = PythonExpression(["'", robot_name, "' + '/'"])
    x0 = LaunchConfiguration('x0')
    y0 = LaunchConfiguration('y0')
    theta0 = LaunchConfiguration('theta0')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')

    common_pose_params = {
        'x0': x0,
        'y0': y0,
        'theta0': theta0,
    }

    # Each robot gets its own namespaced graph and prefixed TF frames.
    return LaunchDescription([
        robot_name_arg,
        x0_arg,
        y0_arg,
        theta0_arg,
        goal_x_arg,
        goal_y_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[
                {
                    'robot_description': robot_desc,
                    'frame_prefix': frame_prefix,
                }
            ]
        ),
        Node(
            package='puzzlebot_sim',
            executable='URDF_tfs',
            name='URDF_tfs',
            namespace=robot_name,
            output='screen',
            parameters=[common_pose_params]
        ),
        Node(
            package='puzzlebot_sim',
            executable='kinematic_model',
            name='kinematic_model',
            namespace=robot_name,
            output='screen',
            parameters=[common_pose_params]
        ),
        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            namespace=robot_name,
            output='screen',
            parameters=[common_pose_params]
        ),
        Node(
            package='puzzlebot_sim',
            executable='control',
            name='control',
            namespace=robot_name,
            output='screen',
            parameters=[
                {
                    'goal_x': goal_x,
                    'goal_y': goal_y,
                }
            ]
        ),
    ])
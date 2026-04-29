from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('puzzlebot_sim')
    robot_launch = os.path.join(pkg_path, 'launch', 'robot.launch.py')

    robot1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_launch
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'x0': '0.0',
            'y0': '0.0',
            'theta0': '0.0',
            'goal_x': '2.0',
            'goal_y': '0.0',
        }.items()
    )

    robot2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_launch
        ),
        launch_arguments={
            'robot_name': 'robot2',
            'x0': '-1.0',
            'y0': '1.0',
            'theta0': '1.57',
            'goal_x': '-2.0',
            'goal_y': '2.0',
        }.items()
    )

    return LaunchDescription([
        robot1_launch,
        robot2_launch,
    ])
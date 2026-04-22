"""
Launch file: All nodes together (alias for control.launch.py)
Complete system with simulator, localization, trajectory, and control.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('puzzlebot_sim')
    
    # Path to control launch
    control_launch = os.path.join(pkg_dir, 'launch', 'control.launch.py')
    
    # Include control launch
    control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )
    
    return LaunchDescription([
        control_launch_include,
    ])


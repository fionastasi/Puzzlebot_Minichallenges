import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_puzzlebot = get_package_share_directory('minichallenge5')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    algo_arg = DeclareLaunchArgument(
        'algo', 
        default_value='bug0', 
        description='Algoritmo a utilizar: bug0 o bug1'
    )
    algo = LaunchConfiguration('algo')

    world_file = os.path.join(pkg_puzzlebot, 'worlds', 'bug0_test.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_puzzlebot, 'launch', 'robot_spawn.launch.py')
        ),
        launch_arguments={'x0': '0.0', 'y0': '0.0'}.items()
    )

    bug0_node = Node(
        package='minichallenge5',
        executable='bug0',
        name='navigator_0',
        output='screen',
        condition=IfCondition(PythonExpression(["'", algo, "' == 'bug0'"]))
    )

    bug1_node = Node(
        package='minichallenge5',
        executable='bug1',
        name='navigator_1',
        output='screen',
        condition=IfCondition(PythonExpression(["'", algo, "' == 'bug1'"]))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        algo_arg,
        gazebo,
        spawn_robot,
        bug0_node,
        bug1_node,
        rviz_node
    ])

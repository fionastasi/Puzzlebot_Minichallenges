import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'minichallenge6_new_test'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    # 1. DECLARAR EL ARGUMENTO DEL MUNDO
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Ruta absoluta al archivo del mundo de Gazebo'
    )
    world_path = LaunchConfiguration('world')

    set_env_vars_resources = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, '..')
    )

    # 2. PASARLE EL MUNDO A GZSERVER
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items() # <--- ¡AQUÍ ESTABA EL ERROR!
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'puzzlebot',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05' 
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(world_arg) # Agregamos el argumento al Launch
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)

    return ld
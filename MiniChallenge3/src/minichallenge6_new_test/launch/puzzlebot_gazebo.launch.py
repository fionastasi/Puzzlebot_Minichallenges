import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Nombre de tu paquete
    pkg_name = 'minichallenge6_new_test'
    
    # Rutas a los archivos clave
    pkg_share = get_package_share_directory(pkg_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf') # Asegúrate de que esta ruta coincida con tu workspace

    set_env_vars_resources = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=os.path.join(pkg_share, '..')
    )

    # 1. Iniciar el servidor de Gazebo (gzserver) con un mundo vacío
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        )
    )

    # 2. Iniciar el cliente de Gazebo (gzclient) para la interfaz gráfica
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Leer el contenido del URDF para el robot_state_publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Nodo Robot State Publisher (Publica las TFs estáticas de tu URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # 4. Nodo para inyectar el modelo en Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'puzzlebot',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05' # Ligeramente elevado para que no colisione con el suelo al inicio
        ],
        output='screen'
    )

    # Agrupar y retornar las acciones
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)

    return ld
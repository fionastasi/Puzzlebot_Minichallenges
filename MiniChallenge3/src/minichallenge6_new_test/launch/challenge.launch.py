import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #==========================================================================
    # 1. CONFIGURACIÓN DE RUTAS LOCALES
    #==========================================================================
    package_name = 'minichallenge6_new_test' 
    pkg_share = get_package_share_directory(package_name)
    
    # Ruta absoluta hacia tu propio launch de Gazebo (el que tienes en tu carpeta)
    local_gazebo_launch_path = os.path.join(pkg_share, 'launch', 'puzzlebot_gazebo.launch.py')
    
    # Ruta absoluta hacia tu mundo personalizado en la carpeta 'worlds'
    world_file_path = os.path.join(pkg_share, 'worlds', 'demo_world2.world')
    
    #==========================================================================
    # 2. INCLUIR TU PROPIO LAUNCH DE GAZEBO
    #==========================================================================
    # Lanzamos el archivo que ya tienes e intentamos inyectarle tu mundo personalizado
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(local_gazebo_launch_path),
        launch_arguments={'world': world_file_path}.items()
    )

    #==========================================================================
    # 3. DEFINICIÓN DE NODOS (LOCALISATION Y BUG0)
    #==========================================================================
    localisation = Node(
        package=package_name,
        executable='localisation', 
        name='localisation',
        output='screen'
    )

    bug0_node = Node(
        package=package_name,
        executable='bug0_node',         
        name='bug0_node',
        output='screen'
    )

    #==========================================================================
    # 4. RETORNAR LA DESCRIPCIÓN DEL LAUNCH
    #==========================================================================
    return LaunchDescription([
        gazebo_launch,
        localisation,
        bug0_node
    ])
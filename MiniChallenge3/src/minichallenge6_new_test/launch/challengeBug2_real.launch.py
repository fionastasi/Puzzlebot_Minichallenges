import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'minichallenge6_new_test'
    pkg_share = get_package_share_directory(package_name)

    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    microros_port = LaunchConfiguration('microros_port', default='/dev/microros')
    rplidar_port = LaunchConfiguration('rplidar_port', default='/dev/rplidar')
    laser_frame = LaunchConfiguration('laser_frame', default='laser_frame')
    encoder_right_topic = LaunchConfiguration('encoder_right_topic', default='/VelocityEncR')
    encoder_left_topic = LaunchConfiguration('encoder_left_topic', default='/VelocityEncL')
    wl_topic = LaunchConfiguration('wl_topic', default='/wl')
    wr_topic = LaunchConfiguration('wr_topic', default='/wr')

    try:
        rplidar_share = get_package_share_directory('rplidar_ros')
    except Exception:
        raise RuntimeError(
            'Package rplidar_ros no encontrado. Instala ros-humble-rplidar-ros o ajusta el nombre del paquete en el launch file.'
        )

    rplidar_launch_dir = os.path.join(rplidar_share, 'launch')
    rplidar_launch_path = None
    if os.path.isdir(rplidar_launch_dir):
        for filename in os.listdir(rplidar_launch_dir):
            if filename.endswith('.launch.py') and 'rplidar' in filename:
                rplidar_launch_path = os.path.join(rplidar_launch_dir, filename)
                break
    if rplidar_launch_path is None:
        raise RuntimeError(
            f'No se encontró ningún launch de rplidar_ros en {rplidar_launch_dir}. '
            'Asegúrate de que el paquete contiene un archivo *.launch.py con rplidar.'
        )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path),
        launch_arguments={
            'frame_id': laser_frame,
            'port': rplidar_port,
        }.items()
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '-D', microros_port, '-b', '115200'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['cat', urdf_path]), 'use_sim_time': use_sim_time},
        ],
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        output='screen',
        arguments=['0.0', '0.0', '0.07', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    localisation = Node(
        package=package_name,
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[
            {'encoder_right_topic': encoder_right_topic,
             'encoder_left_topic': encoder_left_topic,
             'use_sim_time': use_sim_time},
        ],
    )

    urdf_tfs = Node(
        package=package_name,
        executable='URDF_tfs',
        name='URDF_tfs',
        output='screen',
        parameters=[
            {'topic_wl': wl_topic,
             'topic_wr': wr_topic,
             'use_sim_time': use_sim_time},
        ],
    )

    bug2_node = Node(
        package=package_name,
        executable='bug2_node',
        name='bug2_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('microros_port', default_value='/dev/microros'),
        DeclareLaunchArgument('rplidar_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),
        DeclareLaunchArgument('encoder_right_topic', default_value='/VelocityEncR'),
        DeclareLaunchArgument('encoder_left_topic', default_value='/VelocityEncL'),
        DeclareLaunchArgument('wl_topic', default_value='/wl'),
        DeclareLaunchArgument('wr_topic', default_value='/wr'),
        micro_ros_agent,
        rplidar_launch,
        robot_state_publisher,
        laser_tf,
        localisation,
        urdf_tfs,
        bug2_node,
    ])

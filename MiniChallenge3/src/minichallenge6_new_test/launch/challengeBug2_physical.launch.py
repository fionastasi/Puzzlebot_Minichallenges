from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'minichallenge6_new_test'

    use_localisation = LaunchConfiguration('use_localisation')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    wr_topic = LaunchConfiguration('wr_topic')
    wl_topic = LaunchConfiguration('wl_topic')
    require_scan = LaunchConfiguration('require_scan')
    require_odom = LaunchConfiguration('require_odom')
    front_stop_distance = LaunchConfiguration('front_stop_distance')
    avoidance_start_distance = LaunchConfiguration('avoidance_start_distance')
    wall_follow_start_distance = LaunchConfiguration('wall_follow_start_distance')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    scan_front_angle = LaunchConfiguration('scan_front_angle')

    common_parameters = [{'use_sim_time': False}]

    localisation = Node(
        package=package_name,
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=common_parameters,
        condition=IfCondition(use_localisation),
        remappings=[
            ('odom', odom_topic),
            ('VelocityEncR', wr_topic),
            ('VelocityEncL', wl_topic),
        ],
    )

    bug2_node = Node(
        package=package_name,
        executable='bug2_node',
        name='bug2_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'require_scan': ParameterValue(require_scan, value_type=bool)},
            {'require_odom': ParameterValue(require_odom, value_type=bool)},
            {'front_stop_distance': ParameterValue(front_stop_distance, value_type=float)},
            {'avoidance_start_distance': ParameterValue(avoidance_start_distance, value_type=float)},
            {'wall_follow_start_distance': ParameterValue(wall_follow_start_distance, value_type=float)},
            {'goal_tolerance': ParameterValue(goal_tolerance, value_type=float)},
            {'scan_front_angle': ParameterValue(scan_front_angle, value_type=float)},
        ],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('odom', odom_topic),
            ('scan', scan_topic),
            ('goal', goal_topic),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        DeclareLaunchArgument(
            'use_localisation',
            default_value='true',
            description='Genera odom desde encoders. Usa false si el robot publica odom confiable.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='Topico de velocidad del Puzzlebot fisico.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Topico de odometria del Puzzlebot fisico.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='Topico del RPLidar fisico.',
        ),
        DeclareLaunchArgument(
            'goal_topic',
            default_value='goal',
            description='Topico donde se publica la meta Pose2D.',
        ),
        DeclareLaunchArgument(
            'wr_topic',
            default_value='VelocityEncR',
            description='Topico de velocidad angular del encoder derecho.',
        ),
        DeclareLaunchArgument(
            'wl_topic',
            default_value='VelocityEncL',
            description='Topico de velocidad angular del encoder izquierdo.',
        ),
        DeclareLaunchArgument(
            'require_scan',
            default_value='true',
            description='Si es true, el robot se detiene cuando no hay RPLidar reciente.',
        ),
        DeclareLaunchArgument(
            'require_odom',
            default_value='true',
            description='Si es true, el robot se detiene cuando no hay odometria reciente.',
        ),
        DeclareLaunchArgument(
            'front_stop_distance',
            default_value='0.22',
            description='Distancia frontal para detener avance y seguir pared en pasillos estrechos.',
        ),
        DeclareLaunchArgument(
            'avoidance_start_distance',
            default_value='0.38',
            description='Distancia frontal para empezar a esquivar suavemente sin cambiar necesariamente de estado.',
        ),
        DeclareLaunchArgument(
            'wall_follow_start_distance',
            default_value='0.28',
            description='Distancia frontal para registrar impacto y cambiar a WALL_FOLLOWING.',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.05',
            description='Radio en metros para considerar alcanzada la meta.',
        ),
        DeclareLaunchArgument(
            'scan_front_angle',
            default_value='0.0',
            description='Angulo en grados que corresponde al frente del robot dentro del LaserScan.',
        ),
        localisation,
        bug2_node,
    ])

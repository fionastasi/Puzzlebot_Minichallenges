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
    rotate_in_place = LaunchConfiguration('rotate_in_place')
    require_scan = LaunchConfiguration('require_scan')
    require_odom = LaunchConfiguration('require_odom')
    laser_distance_topic = LaunchConfiguration('laser_distance_topic')
    servo_angle_topic = LaunchConfiguration('servo_angle_topic')
    laser_distance_scale = LaunchConfiguration('laser_distance_scale')
    front_stop_distance = LaunchConfiguration('front_stop_distance')
    servo_sweep_enabled = LaunchConfiguration('servo_sweep_enabled')
    obstacle_source = LaunchConfiguration('obstacle_source')
    scan_front_angle = LaunchConfiguration('scan_front_angle')
    goal_tolerance = LaunchConfiguration('goal_tolerance')

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

    bug0_node = Node(
        package=package_name,
        executable='bug0_node',
        name='bug0_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'rotate_in_place': ParameterValue(rotate_in_place, value_type=bool)},
            {'require_scan': ParameterValue(require_scan, value_type=bool)},
            {'require_odom': ParameterValue(require_odom, value_type=bool)},
            {'laser_distance_scale': ParameterValue(laser_distance_scale, value_type=float)},
            {'front_stop_distance': ParameterValue(front_stop_distance, value_type=float)},
            {'servo_sweep_enabled': ParameterValue(servo_sweep_enabled, value_type=bool)},
            {'obstacle_source': obstacle_source},
            {'scan_front_angle': ParameterValue(scan_front_angle, value_type=float)},
            {'goal_tolerance': ParameterValue(goal_tolerance, value_type=float)},
        ],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('odom', odom_topic),
            ('scan', scan_topic),
            ('goal', goal_topic),
            ('LaserDistance', laser_distance_topic),
            ('ServoAngle', servo_angle_topic),
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
            description='Topico del LiDAR del Puzzlebot fisico.',
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
            'rotate_in_place',
            default_value='false',
            description='Si es true, primero gira y luego avanza. Si es false, avanza en arco.',
        ),
        DeclareLaunchArgument(
            'require_scan',
            default_value='true',
            description='Si es true, el robot se detiene cuando no hay LiDAR reciente.',
        ),
        DeclareLaunchArgument(
            'require_odom',
            default_value='true',
            description='Si es true, el robot se detiene cuando no hay odometria reciente.',
        ),
        DeclareLaunchArgument(
            'laser_distance_topic',
            default_value='LaserDistance',
            description='Topico de distancia del sensor frontal/servo.',
        ),
        DeclareLaunchArgument(
            'servo_angle_topic',
            default_value='ServoAngle',
            description='Topico del angulo del servo del sensor de distancia.',
        ),
        DeclareLaunchArgument(
            'laser_distance_scale',
            default_value='1.0',
            description='Escala para LaserDistance. Usa 0.01 si el sensor publica centimetros.',
        ),
        DeclareLaunchArgument(
            'front_stop_distance',
            default_value='0.22',
            description='Distancia frontal para detener avance y entrar a seguimiento de pared.',
        ),
        DeclareLaunchArgument(
            'servo_sweep_enabled',
            default_value='false',
            description='Publica comandos en ServoAngle para barrer el sensor de distancia.',
        ),
        DeclareLaunchArgument(
            'obstacle_source',
            default_value='scan',
            description='Fuente de obstaculos: scan, laser_distance o auto.',
        ),
        DeclareLaunchArgument(
            'scan_front_angle',
            default_value='0.0',
            description='Angulo en grados que corresponde al frente del robot dentro del LaserScan.',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.05',
            description='Radio en metros para considerar alcanzada la meta.',
        ),
        localisation,
        bug0_node,
    ])

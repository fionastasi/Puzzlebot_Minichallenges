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
            default_value='false',
            description='Genera odom desde encoders. Usa true solo si el robot no publica odom.',
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
        localisation,
        bug0_node,
    ])

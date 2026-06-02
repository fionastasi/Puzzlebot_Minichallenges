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
    ekf_odom_topic = LaunchConfiguration('ekf_odom_topic')
    bug2_odom_topic = LaunchConfiguration('bug2_odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    wr_topic = LaunchConfiguration('wr_topic')
    wl_topic = LaunchConfiguration('wl_topic')
    require_scan = LaunchConfiguration('require_scan')
    require_odom = LaunchConfiguration('require_odom')
    front_stop_distance = LaunchConfiguration('front_stop_distance')
    avoidance_start_distance = LaunchConfiguration('avoidance_start_distance')
    wall_follow_start_distance = LaunchConfiguration('wall_follow_start_distance')
    wall_follow_side = LaunchConfiguration('wall_follow_side')
    start_with_wall_acquisition = LaunchConfiguration('start_with_wall_acquisition')
    wall_acquire_distance = LaunchConfiguration('wall_acquire_distance')
    wall_too_close = LaunchConfiguration('wall_too_close')
    wall_lost_distance = LaunchConfiguration('wall_lost_distance')
    wall_follow_speed = LaunchConfiguration('wall_follow_speed')
    wall_follow_kp = LaunchConfiguration('wall_follow_kp')
    wall_front_kp = LaunchConfiguration('wall_front_kp')
    wall_follow_deadband = LaunchConfiguration('wall_follow_deadband')
    wall_search_angular_speed = LaunchConfiguration('wall_search_angular_speed')
    wall_recovery_forward_distance = LaunchConfiguration('wall_recovery_forward_distance')
    wall_recovery_forward_speed = LaunchConfiguration('wall_recovery_forward_speed')
    wall_recovery_turn_angle = LaunchConfiguration('wall_recovery_turn_angle')
    wall_recovery_turn_speed = LaunchConfiguration('wall_recovery_turn_speed')
    wall_corner_angular_speed = LaunchConfiguration('wall_corner_angular_speed')
    wall_command_alpha = LaunchConfiguration('wall_command_alpha')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    wall_follow_goal_tolerance = LaunchConfiguration('wall_follow_goal_tolerance')
    goal_pass_margin = LaunchConfiguration('goal_pass_margin')
    goal_pass_lateral_tolerance = LaunchConfiguration('goal_pass_lateral_tolerance')
    goal_priority_distance = LaunchConfiguration('goal_priority_distance')
    near_goal_slow_distance = LaunchConfiguration('near_goal_slow_distance')
    near_goal_v_max = LaunchConfiguration('near_goal_v_max')
    scan_front_angle = LaunchConfiguration('scan_front_angle')
    use_aruco_tracker = LaunchConfiguration('use_aruco_tracker')
    use_aruco_monitor = LaunchConfiguration('use_aruco_monitor')
    use_ekf = LaunchConfiguration('use_ekf')
    use_aruco_correction = LaunchConfiguration('use_aruco_correction')
    aruco_cam_base_topic = LaunchConfiguration('aruco_cam_base_topic')
    aruco_marker_size = LaunchConfiguration('aruco_marker_size')
    aruco_detection_topic = LaunchConfiguration('aruco_detection_topic')
    aruco_detection_type = LaunchConfiguration('aruco_detection_type')
    aruco_pose_source_frame = LaunchConfiguration('aruco_pose_source_frame')
    max_marker_distance = LaunchConfiguration('max_marker_distance')
    max_aruco_innovation = LaunchConfiguration('max_aruco_innovation')
    max_aruco_raw_disagreement = LaunchConfiguration('max_aruco_raw_disagreement')
    aruco_measurement_std_x = LaunchConfiguration('aruco_measurement_std_x')
    aruco_measurement_std_y = LaunchConfiguration('aruco_measurement_std_y')
    ekf_process_noise_x = LaunchConfiguration('ekf_process_noise_x')
    ekf_process_noise_y = LaunchConfiguration('ekf_process_noise_y')
    ekf_process_noise_theta = LaunchConfiguration('ekf_process_noise_theta')
    ekf_diagnostic_period = LaunchConfiguration('ekf_diagnostic_period')
    odom_offset_x = LaunchConfiguration('odom_offset_x')
    odom_offset_y = LaunchConfiguration('odom_offset_y')
    odom_offset_theta = LaunchConfiguration('odom_offset_theta')
    publish_without_fresh_encoders = LaunchConfiguration('publish_without_fresh_encoders')
    camera_offset_x = LaunchConfiguration('camera_offset_x')
    camera_offset_y = LaunchConfiguration('camera_offset_y')
    camera_offset_z = LaunchConfiguration('camera_offset_z')

    localisation = Node(
        package=package_name,
        executable='localisation',
        name='localisation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'odom_offset_x': ParameterValue(odom_offset_x, value_type=float)},
            {'odom_offset_y': ParameterValue(odom_offset_y, value_type=float)},
            {'odom_offset_theta': ParameterValue(odom_offset_theta, value_type=float)},
            {'publish_without_fresh_encoders': ParameterValue(publish_without_fresh_encoders, value_type=bool)},
        ],
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
            {'wall_follow_side': wall_follow_side},
            {'start_with_wall_acquisition': ParameterValue(start_with_wall_acquisition, value_type=bool)},
            {'wall_acquire_distance': ParameterValue(wall_acquire_distance, value_type=float)},
            {'wall_too_close': ParameterValue(wall_too_close, value_type=float)},
            {'wall_lost_distance': ParameterValue(wall_lost_distance, value_type=float)},
            {'wall_follow_speed': ParameterValue(wall_follow_speed, value_type=float)},
            {'wall_follow_kp': ParameterValue(wall_follow_kp, value_type=float)},
            {'wall_front_kp': ParameterValue(wall_front_kp, value_type=float)},
            {'wall_follow_deadband': ParameterValue(wall_follow_deadband, value_type=float)},
            {'wall_search_angular_speed': ParameterValue(wall_search_angular_speed, value_type=float)},
            {'wall_recovery_forward_distance': ParameterValue(wall_recovery_forward_distance, value_type=float)},
            {'wall_recovery_forward_speed': ParameterValue(wall_recovery_forward_speed, value_type=float)},
            {'wall_recovery_turn_angle': ParameterValue(wall_recovery_turn_angle, value_type=float)},
            {'wall_recovery_turn_speed': ParameterValue(wall_recovery_turn_speed, value_type=float)},
            {'wall_corner_angular_speed': ParameterValue(wall_corner_angular_speed, value_type=float)},
            {'wall_command_alpha': ParameterValue(wall_command_alpha, value_type=float)},
            {'goal_tolerance': ParameterValue(goal_tolerance, value_type=float)},
            {'wall_follow_goal_tolerance': ParameterValue(wall_follow_goal_tolerance, value_type=float)},
            {'goal_pass_margin': ParameterValue(goal_pass_margin, value_type=float)},
            {'goal_pass_lateral_tolerance': ParameterValue(goal_pass_lateral_tolerance, value_type=float)},
            {'goal_priority_distance': ParameterValue(goal_priority_distance, value_type=float)},
            {'near_goal_slow_distance': ParameterValue(near_goal_slow_distance, value_type=float)},
            {'near_goal_v_max': ParameterValue(near_goal_v_max, value_type=float)},
            {'scan_front_angle': ParameterValue(scan_front_angle, value_type=float)},
        ],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('odom', bug2_odom_topic),
            ('scan', scan_topic),
            ('goal', goal_topic),
        ],
    )

    ekf_localisation = Node(
        package=package_name,
        executable='ekf_localisation',
        name='ekf_localisation',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[
            {'use_sim_time': False},
            {'aruco_detection_type': aruco_detection_type},
            {'aruco_pose_source_frame': aruco_pose_source_frame},
            {'camera_offset_x': ParameterValue(camera_offset_x, value_type=float)},
            {'camera_offset_y': ParameterValue(camera_offset_y, value_type=float)},
            {'camera_offset_z': ParameterValue(camera_offset_z, value_type=float)},
            {'max_marker_distance': ParameterValue(max_marker_distance, value_type=float)},
            {'max_aruco_innovation': ParameterValue(max_aruco_innovation, value_type=float)},
            {'max_aruco_raw_disagreement': ParameterValue(max_aruco_raw_disagreement, value_type=float)},
            {'aruco_measurement_std_x': ParameterValue(aruco_measurement_std_x, value_type=float)},
            {'aruco_measurement_std_y': ParameterValue(aruco_measurement_std_y, value_type=float)},
            {'process_noise_x': ParameterValue(ekf_process_noise_x, value_type=float)},
            {'process_noise_y': ParameterValue(ekf_process_noise_y, value_type=float)},
            {'process_noise_theta': ParameterValue(ekf_process_noise_theta, value_type=float)},
            {'diagnostic_period': ParameterValue(ekf_diagnostic_period, value_type=float)},
            {'use_aruco_correction': ParameterValue(use_aruco_correction, value_type=bool)},
        ],
        remappings=[
            ('odom_raw', odom_topic),
            ('odom_ekf', ekf_odom_topic),
            ('markers', aruco_detection_topic),
        ],
    )

    aruco_tracker = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker_autostart',
        output='screen',
        condition=IfCondition(use_aruco_tracker),
        parameters=[
            {'cam_base_topic': aruco_cam_base_topic},
            {'marker_size': ParameterValue(aruco_marker_size, value_type=float)},
        ],
    )

    aruco_monitor = Node(
        package=package_name,
        executable='aruco_detection_monitor',
        name='aruco_detection_monitor',
        output='screen',
        condition=IfCondition(use_aruco_monitor),
        parameters=[
            {'detection_topic': aruco_detection_topic},
            {'detection_type': aruco_detection_type},
            {'camera_offset_x': ParameterValue(camera_offset_x, value_type=float)},
            {'camera_offset_y': ParameterValue(camera_offset_y, value_type=float)},
            {'camera_offset_z': ParameterValue(camera_offset_z, value_type=float)},
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
            description='Topico de odometria cruda publicada por localisation.',
        ),
        DeclareLaunchArgument(
            'ekf_odom_topic',
            default_value='odom_ekf',
            description='Topico de odometria corregida publicada por el EKF.',
        ),
        DeclareLaunchArgument(
            'bug2_odom_topic',
            default_value='odom_ekf',
            description='Topico de odometria que escucha Bug2. Usa odom para desactivar EKF en navegacion.',
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
            'wall_follow_side',
            default_value='right',
            description='Lado de seguimiento de pared: right o left.',
        ),
        DeclareLaunchArgument(
            'start_with_wall_acquisition',
            default_value='true',
            description='Si es true, al recibir una meta primero se acerca a la pared configurada antes de comportamiento Bug2 normal.',
        ),
        DeclareLaunchArgument(
            'wall_acquire_distance',
            default_value='0.22',
            description='Distancia lateral para considerar adquirida la pared elegida antes de hacer seguimiento fino.',
        ),
        DeclareLaunchArgument(
            'wall_too_close',
            default_value='0.11',
            description='Distancia lateral minima antes de girar para separarse de la pared.',
        ),
        DeclareLaunchArgument(
            'wall_lost_distance',
            default_value='0.35',
            description='Distancia lateral a partir de la cual se considera que se perdio la pared.',
        ),
        DeclareLaunchArgument(
            'wall_follow_speed',
            default_value='0.10',
            description='Velocidad lineal durante WALL_FOLLOWING.',
        ),
        DeclareLaunchArgument(
            'wall_follow_kp',
            default_value='1.2',
            description='Ganancia proporcional para mantener distancia lateral a la pared.',
        ),
        DeclareLaunchArgument(
            'wall_front_kp',
            default_value='0.7',
            description='Ganancia extra para abrirse cuando la esquina frontal del lado elegido esta cerca.',
        ),
        DeclareLaunchArgument(
            'wall_follow_deadband',
            default_value='0.025',
            description='Banda muerta lateral para evitar oscilaciones pequenas siguiendo pared.',
        ),
        DeclareLaunchArgument(
            'wall_search_angular_speed',
            default_value='0.18',
            description='Velocidad angular para buscar la pared cuando se pierde lateralmente.',
        ),
        DeclareLaunchArgument(
            'wall_recovery_forward_distance',
            default_value='0.10',
            description='Distancia en metros que avanza al perder la pared elegida antes de girar.',
        ),
        DeclareLaunchArgument(
            'wall_recovery_forward_speed',
            default_value='0.035',
            description='Velocidad lineal durante el avance de recuperacion de pared perdida.',
        ),
        DeclareLaunchArgument(
            'wall_recovery_turn_angle',
            default_value='1.5708',
            description='Angulo en radianes del giro hacia la pared elegida durante recuperacion.',
        ),
        DeclareLaunchArgument(
            'wall_recovery_turn_speed',
            default_value='0.30',
            description='Velocidad angular del giro de recuperacion hacia la pared elegida.',
        ),
        DeclareLaunchArgument(
            'wall_corner_angular_speed',
            default_value='0.30',
            description='Velocidad angular al enfrentar esquina u obstaculo frontal en WALL_FOLLOWING.',
        ),
        DeclareLaunchArgument(
            'wall_command_alpha',
            default_value='0.35',
            description='Suavizado de comandos de pared: 1.0 sin suavizado, menor es mas suave.',
        ),
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.05',
            description='Radio en metros para considerar alcanzada la meta.',
        ),
        DeclareLaunchArgument(
            'wall_follow_goal_tolerance',
            default_value='0.08',
            description='Radio de captura de meta cuando Bug2 esta siguiendo pared.',
        ),
        DeclareLaunchArgument(
            'goal_pass_margin',
            default_value='0.02',
            description='Margen para detenerse si ya paso cerca de la meta y se empieza a alejar.',
        ),
        DeclareLaunchArgument(
            'goal_pass_lateral_tolerance',
            default_value='0.22',
            description='Distancia lateral maxima a la linea inicio-meta para detener si ya cruzo la meta.',
        ),
        DeclareLaunchArgument(
            'goal_priority_distance',
            default_value='0.35',
            description='Distancia a meta donde se prioriza llegar antes de cambiar a WALL_FOLLOWING salvo obstaculo critico.',
        ),
        DeclareLaunchArgument(
            'near_goal_slow_distance',
            default_value='0.35',
            description='Distancia a meta desde la que Bug2 reduce velocidad.',
        ),
        DeclareLaunchArgument(
            'near_goal_v_max',
            default_value='0.025',
            description='Velocidad maxima cerca de la meta.',
        ),
        DeclareLaunchArgument(
            'scan_front_angle',
            default_value='0.0',
            description='Angulo en grados que corresponde al frente del robot dentro del LaserScan.',
        ),
        DeclareLaunchArgument(
            'use_aruco_tracker',
            default_value='true',
            description='Arranca aruco_opencv/aruco_tracker_autostart desde este launch.',
        ),
        DeclareLaunchArgument(
            'use_aruco_monitor',
            default_value='true',
            description='Arranca monitor propio que reporta el ArUco mas cercano sin corregir odometria.',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Arranca el nodo EKF que fusiona odometria cruda con landmarks ArUco.',
        ),
        DeclareLaunchArgument(
            'use_aruco_correction',
            default_value='true',
            description='Si es true, el EKF corrige x,y usando ArUco; si es false, solo predice por odometria.',
        ),
        DeclareLaunchArgument(
            'odom_offset_x',
            default_value='0.295',
            description='Offset inicial x del robot dentro del mapa.',
        ),
        DeclareLaunchArgument(
            'odom_offset_y',
            default_value='-0.29',
            description='Offset inicial y del robot dentro del mapa.',
        ),
        DeclareLaunchArgument(
            'odom_offset_theta',
            default_value='0.0',
            description='Offset inicial de yaw del robot en radianes.',
        ),
        DeclareLaunchArgument(
            'publish_without_fresh_encoders',
            default_value='true',
            description='Mantiene odom publicado con velocidad cero aunque los encoders no lleguen recientes.',
        ),
        DeclareLaunchArgument(
            'aruco_cam_base_topic',
            default_value='/image_raw',
            description='Topico base de imagen usado por aruco_opencv.',
        ),
        DeclareLaunchArgument(
            'aruco_marker_size',
            default_value='0.06',
            description='Tamano del marcador ArUco en metros usado por aruco_opencv.',
        ),
        DeclareLaunchArgument(
            'aruco_detection_topic',
            default_value='/marker_publisher/markers',
            description='Topico de detecciones/markers que escucha el monitor ArUco.',
        ),
        DeclareLaunchArgument(
            'aruco_detection_type',
            default_value='aruco_msgs',
            description='Tipo de deteccion: markers_list, markers_list_u32, aruco_msgs, visualization_marker_array o aruco_opencv.',
        ),
        DeclareLaunchArgument(
            'aruco_pose_source_frame',
            default_value='camera',
            description='Frame geometrico de la pose ArUco recibida: camera aplica transformacion a base, base la usa directo.',
        ),
        DeclareLaunchArgument(
            'max_marker_distance',
            default_value='2.0',
            description='Distancia horizontal maxima para aceptar una deteccion ArUco en el EKF.',
        ),
        DeclareLaunchArgument(
            'max_aruco_innovation',
            default_value='1.5',
            description='Innovacion maxima en metros antes de rechazar una correccion ArUco.',
        ),
        DeclareLaunchArgument(
            'max_aruco_raw_disagreement',
            default_value='0.35',
            description='Diferencia maxima entre odom raw y pose calculada por ArUco antes de rechazarla; 0 desactiva esta compuerta.',
        ),
        DeclareLaunchArgument(
            'aruco_measurement_std_x',
            default_value='0.08',
            description='Desviacion estandar de medicion ArUco en x para el EKF.',
        ),
        DeclareLaunchArgument(
            'aruco_measurement_std_y',
            default_value='0.08',
            description='Desviacion estandar de medicion ArUco en y para el EKF.',
        ),
        DeclareLaunchArgument(
            'ekf_process_noise_x',
            default_value='0.003',
            description='Ruido de proceso EKF para x.',
        ),
        DeclareLaunchArgument(
            'ekf_process_noise_y',
            default_value='0.003',
            description='Ruido de proceso EKF para y.',
        ),
        DeclareLaunchArgument(
            'ekf_process_noise_theta',
            default_value='0.01',
            description='Ruido de proceso EKF para theta.',
        ),
        DeclareLaunchArgument(
            'ekf_diagnostic_period',
            default_value='2.0',
            description='Periodo en segundos para imprimir diagnosticos raw vs EKF.',
        ),
        DeclareLaunchArgument(
            'camera_offset_x',
            default_value='0.1241',
            description='Distancia de base_link a camara en x.',
        ),
        DeclareLaunchArgument(
            'camera_offset_y',
            default_value='0.0',
            description='Distancia de base_link a camara en y.',
        ),
        DeclareLaunchArgument(
            'camera_offset_z',
            default_value='0.067',
            description='Distancia de base_link a camara en z.',
        ),
        localisation,
        aruco_tracker,
        aruco_monitor,
        ekf_localisation,
        bug2_node,
    ])

#!/usr/bin/env python3 
import rclpy 
from rclpy import qos
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Pose2D 
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry 
import math 
import signal 
import sys 

def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class Bug0Node(Node):  
    def __init__(self):  
        super().__init__('bug0_node') 
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)
        self.create_subscription(Pose2D, 'goal', self.goal_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos.qos_profile_sensor_data)

        signal.signal(signal.SIGINT, self.shutdown_function) 

        self.state = "WAITING" 
        self.goal_received = False 
        
        self.x = 0.0 
        self.y = 0.0 
        self.theta = 0.0 
        self.target_x = 0.0 
        self.target_y = 0.0 
        self.last_odom_time = None
        self.last_scan_time = None
        self.last_diagnostic_time = self.get_clock().now()
        
        self.goal_tolerance = 0.15 
        self.d_thresh = 0.45  
        
        # Control proporcional hacia la meta
        self.declare_parameter('k_rho', 0.6)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('heading_tolerance', 0.15)
        self.declare_parameter('rotate_in_place', False)
        self.declare_parameter('min_forward_speed', 0.05)
        self.declare_parameter('require_scan', True)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('sensor_timeout', 1.0)

        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.rotate_in_place = self.get_parameter('rotate_in_place').value
        self.min_forward_speed = self.get_parameter('min_forward_speed').value
        self.require_scan = self.get_parameter('require_scan').value
        self.require_odom = self.get_parameter('require_odom').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value

        self.angle_ranges = []

        self.regions = {
            'front': 10.0, 'fleft': 10.0, 'left': 10.0, 'right': 10.0, 'fright': 10.0,
        }

        # Ejecuta el bucle de control a 20 Hz para reducir desfases durante las maniobras
        self.create_timer(0.05, self.control_loop) 
        self.get_logger().info("Nodo Bug 0 inicializado. Esperando meta en /goal...") 

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def change_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'Cambiando de estado: {self.state} -> {new_state}')
            self.state = new_state

    def sector_min(self, center_angle, half_width, default=10.0):
        values = [
            distance for angle, distance in self.angle_ranges
            if abs(self.normalize_angle(angle - center_angle)) <= half_width
        ]
        return min(values) if values else default

    def is_path_to_goal_clear(self, err_theta):
        return self.sector_min(err_theta, math.radians(15)) > (self.d_thresh + 0.15)

    def control_loop(self): 
        if not self.goal_received:
            return

        msg = Twist()
        now = self.get_clock().now()
        odom_age = self.message_age(now, self.last_odom_time)
        scan_age = self.message_age(now, self.last_scan_time)

        if not self.sensors_ready(odom_age, scan_age):
            self.cmd_pub.publish(msg)
            self.publish_diagnostics(msg, None, None, odom_age, scan_age)
            return
        
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta = self.normalize_angle(angle_to_goal - self.theta)

        if dist_to_goal < self.goal_tolerance:
            self.change_state("STOP")
            self.get_logger().info('Meta alcanzada con Bug 0.')
            self.cmd_pub.publish(Twist()) 
            self.goal_received = False 
            return

        # ---------------- TRANSICIONES DE ESTADO ---------------- #
        if self.state == "GO_TO_GOAL":
            if self.regions['front'] < self.d_thresh and abs(err_theta) < 0.5:
                self.change_state("WALL_FOLLOWING")
                
        elif self.state == "WALL_FOLLOWING":
            # Condición de retorno a GO_TO_GOAL cuando la ruta hacia la meta se libera
            if self.is_path_to_goal_clear(err_theta) and self.regions['front'] > (self.d_thresh + 0.05):
                self.get_logger().info('Ruta a la meta despejada. Cambiando a GO_TO_GOAL.')
                self.change_state("GO_TO_GOAL")

        # ---------------- ACCIONES DE ESTADO ---------------- #
        # Estructura de control independiente para evitar bloqueos de velocidad
        if self.state == "GO_TO_GOAL":
            if abs(err_theta) > self.heading_tolerance:
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
                if self.rotate_in_place:
                    msg.linear.x = 0.0
                else:
                    heading_factor = max(0.0, math.cos(err_theta))
                    if heading_factor < 0.2:
                        msg.linear.x = 0.0
                    else:
                        forward_speed = self.k_rho * dist_to_goal * heading_factor
                        msg.linear.x = self.clamp(forward_speed, self.min_forward_speed, self.v_max)
            else:
                msg.linear.x = self.clamp(self.k_rho * dist_to_goal, -self.v_max, self.v_max)
                msg.angular.z = 0.0  
                
        elif self.state == "WALL_FOLLOWING":
            # Algoritmo de seguimiento con ajustes basados en Bug 2
            # 1. Pared frontal: detiene y gira a la izquierda
            if self.regions['front'] < self.d_thresh:
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            
            # 2. Pared diagonal: alinea la trayectoria en paralelo
            elif self.regions['fright'] < self.d_thresh:
                msg.linear.x = 0.08
                msg.angular.z = 0.4
            
            # 3. Pared lateral derecha detectada
            elif self.regions['right'] < self.d_thresh:
                # Corrección suave cuando la distancia es demasiado corta
                if self.regions['right'] < (self.d_thresh - 0.15):
                    msg.linear.x = 0.12
                    msg.angular.z = 0.2
                # Distancia adecuada: mantiene el seguimiento de la pared
                else:
                    msg.linear.x = 0.15
                    msg.angular.z = -0.05  
            
            # 4. Esquina o pérdida de pared: gira a la derecha para recuperar el trayecto
            else:
                msg.linear.x = 0.04   
                msg.angular.z = -0.6  

        self.cmd_pub.publish(msg)
        self.publish_diagnostics(msg, dist_to_goal, err_theta, odom_age, scan_age)

    def sensors_ready(self, odom_age, scan_age):
        odom_missing = self.require_odom and (odom_age is None or odom_age > self.sensor_timeout)
        scan_missing = self.require_scan and (scan_age is None or scan_age > self.sensor_timeout)
        return not odom_missing and not scan_missing

    def publish_diagnostics(self, cmd_msg, dist_to_goal, err_theta, odom_age=None, scan_age=None):
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < 2.0e9:
            return

        self.last_diagnostic_time = now
        if odom_age is None:
            odom_age = self.message_age(now, self.last_odom_time)
        if scan_age is None:
            scan_age = self.message_age(now, self.last_scan_time)

        if odom_age is None or odom_age > 1.0:
            self.get_logger().warn(
                'No estoy recibiendo /odom reciente; revisa odometria o lanza use_localisation:=true.'
            )
        if scan_age is None or scan_age > 1.0:
            self.get_logger().warn(
                'No estoy recibiendo scan reciente; el robot se mantiene detenido por seguridad.'
            )

        cmd_subs = self.count_subscribers(self.cmd_pub.topic_name)
        dist_text = 'sin_odom' if dist_to_goal is None else f'{dist_to_goal:.2f}'
        err_text = 'sin_odom' if err_theta is None else f'{err_theta:.2f}'
        self.get_logger().info(
            f'cmd_vel: v={cmd_msg.linear.x:.2f}, w={cmd_msg.angular.z:.2f}, '
            f'dist={dist_text}, err_theta={err_text}, '
            f'odom_age={self.format_age(odom_age)}, scan_age={self.format_age(scan_age)}, '
            f'cmd_subs={cmd_subs}'
        )

    def message_age(self, now, last_time):
        if last_time is None:
            return None
        return (now - last_time).nanoseconds * 1.0e-9

    def format_age(self, age):
        if age is None:
            return 'nunca'
        return f'{age:.1f}s'

    def odom_callback(self, msg):  
        self.last_odom_time = self.get_clock().now()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):  
        self.target_x = msg.x 
        self.target_y = msg.y 
        self.goal_received = True 
        self.change_state("GO_TO_GOAL")
        self.get_logger().info(f"Bug 0 Metarecibida: x={self.target_x}, y={self.target_y}")

    def scan_callback(self, msg): 
        self.last_scan_time = self.get_clock().now()
        self.angle_ranges = []

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle = self.normalize_angle(angle)

            if math.isinf(distance) or math.isnan(distance) or distance > msg.range_max:
                clean_distance = 10.0
            elif distance < max(msg.range_min, 0.12):
                clean_distance = 0.01
            else:
                clean_distance = distance

            self.angle_ranges.append((angle, clean_distance))

        self.regions = {
            'right':  self.sector_min(math.radians(-90), math.radians(30)),
            'fright': self.sector_min(math.radians(-45), math.radians(25)),
            'front':  self.sector_min(0.0, math.radians(20)),
            'fleft':  self.sector_min(math.radians(45), math.radians(25)),
            'left':   self.sector_min(math.radians(90), math.radians(30)),
        }

    def shutdown_function(self, signum, frame): 
        self.cmd_pub.publish(Twist()) 
        rclpy.shutdown() 
        sys.exit(0) 

def main(args=None): 
    rclpy.init(args=args) 
    node = Bug0Node() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()

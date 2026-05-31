#!/usr/bin/env python3 
import rclpy 
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

class Bug2Node(Node):  
    def __init__(self):  
        super().__init__('bug2_node') 
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)  
        self.create_subscription(Pose2D, "/goal", self.goal_callback, 10) 
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        signal.signal(signal.SIGINT, self.shutdown_function) 

        self.state = "WAITING" 
        self.goal_received = False 
        
        self.x = 0.0 
        self.y = 0.0 
        self.theta = 0.0 
        
        self.target_x = 0.0 
        self.target_y = 0.0 
        self.start_x = 0.0
        self.start_y = 0.0
        
        self.hit_distance = float('inf') 
        self.left_m_line = False  # Indicador de si la línea M ha sido cruzada
        self.hit_x = 0.0  # Coordenada X del punto de impacto
        self.hit_y = 0.0  # Coordenada Y del punto de impacto
        
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('d_thresh', 0.45)
        self.declare_parameter('m_line_tolerance', 0.25)
        self.declare_parameter('k_rho', 0.6)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('lidar_front_index', 180)
        self.declare_parameter('waypoints', [1.0, 0.0, 1.0, 1.5, -0.5, 1.5, -0.5, 0.0])
        self.declare_parameter('auto_waypoints', True)

        self.goal_tolerance = self.get_parameter('goal_tolerance').value 
        self.d_thresh = self.get_parameter('d_thresh').value  
        self.m_line_tolerance = self.get_parameter('m_line_tolerance').value 
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.v_max = self.get_parameter('v_max').value  # Velocidad lineal máxima
        self.w_max = self.get_parameter('w_max').value  
        self.lidar_front_index = self.get_parameter('lidar_front_index').value
        self.waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)
        self.auto_waypoints = self.get_parameter('auto_waypoints').value
        self.wp_idx = 0

        if self.auto_waypoints and self.waypoints:
            self.select_waypoint(0)

        self.regions = {
            'front': 10.0, 'fleft': 10.0, 'left': 10.0, 'right': 10.0, 'fright': 10.0,
        }

        self.create_timer(0.05, self.control_loop)  # Ejecuta el bucle de control a 20 Hz
        self.get_logger().info("Nodo Bug2 inicializado. Esperando meta en /goal...") 

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

    def distance_to_m_line(self):
        num = abs((self.target_y - self.start_y) * self.x - 
                  (self.target_x - self.start_x) * self.y + 
                  self.target_x * self.start_y - 
                  self.target_y * self.start_x)
        den = math.sqrt((self.target_y - self.start_y)**2 + (self.target_x - self.start_x)**2)
        if den == 0:
            return 0.0
        return num / den

    def _angle_to_index(self, angle_deg):
        return int((self.lidar_front_index + angle_deg) % 360)

    def _sector(self, ranges, start_deg, end_deg):
        start = self._angle_to_index(start_deg)
        end = self._angle_to_index(end_deg)
        if start <= end:
            return ranges[start:end]
        return ranges[start:] + ranges[:end]

    def _parse_waypoints(self, raw_waypoints):
        if isinstance(raw_waypoints, list) and len(raw_waypoints) % 2 == 0:
            return [(float(raw_waypoints[i]), float(raw_waypoints[i + 1])) for i in range(0, len(raw_waypoints), 2)]
        return []

    def select_waypoint(self, idx):
        if not self.waypoints:
            return
        self.wp_idx = idx % len(self.waypoints)
        self.target_x, self.target_y = self.waypoints[self.wp_idx]
        self.start_x = self.x
        self.start_y = self.y
        self.goal_received = True
        self.change_state("GO_TO_GOAL")
        self.get_logger().info(f"Waypoint {self.wp_idx + 1}/{len(self.waypoints)} -> x={self.target_x}, y={self.target_y}")

    def advance_waypoint(self):
        if not self.waypoints:
            self.goal_received = False
            return
        self.select_waypoint(self.wp_idx + 1)

    def control_loop(self): 
        if not self.goal_received:
            return

        msg = Twist()
        
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta = self.normalize_angle(angle_to_goal - self.theta)

        if dist_to_goal < self.goal_tolerance:
            self.change_state("STOP")
            self.get_logger().info('Meta alcanzada. Deteniendo Bug2.')
            self.cmd_pub.publish(Twist()) 
            if self.auto_waypoints:
                self.advance_waypoint()
            else:
                self.goal_received = False 
            return

       # ---------------- TRANSICIONES DE ESTADO ---------------- #
        if self.state == "GO_TO_GOAL":
            if self.regions['front'] < self.d_thresh and abs(err_theta) < 0.5:
                self.hit_distance = dist_to_goal
                # Registra la posición del punto de impacto
                self.hit_x = self.x
                self.hit_y = self.y
                self.get_logger().info(f'Punto de impacto registrado en ({self.hit_x:.2f}, {self.hit_y:.2f}) a {self.hit_distance:.2f} m.')
                self.change_state("WALL_FOLLOWING")
                
        elif self.state == "WALL_FOLLOWING":
            dist_m_line = self.distance_to_m_line()
            # Distancia actual hasta el punto de impacto
            dist_to_hit = math.sqrt((self.x - self.hit_x)**2 + (self.y - self.hit_y)**2)
            
            # Condiciones de salida del seguimiento de pared:
            # 1. El robot vuelve a la línea M
            # 2. Está más cerca de la meta que en el momento del impacto
            # 3. Se ha alejado al menos 0.40 m del punto de impacto
            if dist_m_line < self.m_line_tolerance and dist_to_goal < (self.hit_distance - 0.15) and dist_to_hit > 0.40:
                self.get_logger().info(f'Línea M interceptada a {dist_to_goal:.2f} m. Cambio a GO_TO_GOAL.')
                self.change_state("GO_TO_GOAL")

        # ---------------- ACCIONES DE ESTADO ---------------- #
        # Evalúa el estado actual para seleccionar la acción de control correspondiente
        if self.state == "GO_TO_GOAL":
            if abs(err_theta) > 0.15:  
                msg.linear.x = 0.0  
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
            else:
                msg.linear.x = self.clamp(self.k_rho * dist_to_goal, -self.v_max, self.v_max)
                msg.angular.z = 0.0  
                
        elif self.state == "WALL_FOLLOWING":
            # 1. Objeto frontal detectado: detiene y gira a la izquierda
            if self.regions['front'] < self.d_thresh:
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            
            # 2. Objeto en diagonal frontal-derecha: gira a la izquierda para mantener paralelismo
            elif self.regions['fright'] < self.d_thresh:
                msg.linear.x = 0.08
                msg.angular.z = 0.4
            
            # 3. Pared a la derecha: regula la orientación y la velocidad de avance
            elif self.regions['right'] < self.d_thresh:
                if self.regions['right'] < (self.d_thresh - 0.15):
                    msg.linear.x = 0.12
                    msg.angular.z = 0.2
                else:
                    msg.linear.x = 0.15
                    msg.angular.z = -0.05  
            
            # 4. Sin referencia de pared: recupera la trayectoria con un giro suave
            else:
                msg.linear.x = 0.04   
                msg.angular.z = -0.6  

        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):  
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):  
        self.target_x = msg.x 
        self.target_y = msg.y 
        self.start_x = self.x
        self.start_y = self.y
        self.goal_received = True 
        self.change_state("GO_TO_GOAL")
        self.get_logger().info(f"Bug 2 Meta: x={self.target_x}, y={self.target_y}. Línea M trazada.")  

    def scan_callback(self, msg): 
        clean_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r > 10.0:
                clean_ranges.append(10.0)
            elif r < 0.12: 
                clean_ranges.append(0.01) 
            else:
                clean_ranges.append(r)
        
        num_rays = len(clean_ranges)
        
        if num_rays > 0:
            if num_rays >= 350: 
                self.regions = {
                    'right':  min(self._sector(clean_ranges, -130, -70) + [10.0]),   # Aprox -130° a -70°
                    'fright': min(self._sector(clean_ranges, -70, -20) + [10.0]),  # Aprox -70° a -20°
                    'front':  min(self._sector(clean_ranges, -20, 20) + [10.0]),  # Aprox -20° a +20° (frente del robot)
                    'fleft':  min(self._sector(clean_ranges, 20, 70) + [10.0]),  # Aprox +20° a +70°
                    'left':   min(self._sector(clean_ranges, 70, 130) + [10.0]),  # Aprox +70° a +130°
                }

    def shutdown_function(self, signum, frame): 
        self.cmd_pub.publish(Twist()) 
        rclpy.shutdown() 
        sys.exit(0) 

def main(args=None): 
    rclpy.init(args=args) 
    node = Bug2Node() 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()
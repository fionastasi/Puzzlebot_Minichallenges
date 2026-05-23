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
        self.left_m_line = False # Bandera anti-efecto escalera
        
        self.goal_tolerance = 0.15 
        self.d_thresh = 0.45  
        self.m_line_tolerance = 0.25 
        
        self.k_rho = 0.6
        self.k_alpha = 1.5
        self.v_max = 0.2  # Suave para no derrapar
        self.w_max = 0.6  

        self.regions = {
            'front': 10.0, 'fleft': 10.0, 'left': 10.0, 'right': 10.0, 'fright': 10.0,
        }

        self.create_timer(0.05, self.control_loop) # 20 Hz para pensar más rápido
        self.get_logger().info("Nodo BUG 2 inicializado. Esperando meta en /goal...") 

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

    def control_loop(self): 
        if not self.goal_received:
            return

        msg = Twist()
        
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta = self.normalize_angle(angle_to_goal - self.theta)

        if dist_to_goal < self.goal_tolerance:
            self.change_state("STOP")
            self.get_logger().info('¡Meta alcanzada exitosamente con Bug 2!')
            self.cmd_pub.publish(Twist()) 
            self.goal_received = False 
            return

        # ---------------- TRANSICIONES DE ESTADO ---------------- #
        if self.state == "GO_TO_GOAL":
            # Si ve pared enfrente y el ángulo está alineado a la meta
            if self.regions['front'] < self.d_thresh and abs(err_theta) < 0.5:
                self.hit_distance = dist_to_goal
                self.left_m_line = False # Reiniciamos la bandera
                self.get_logger().info(f'¡Hit Point! Registrado a {self.hit_distance:.2f}m.')
                self.change_state("WALL_FOLLOWING")
                
        elif self.state == "WALL_FOLLOWING":
            dist_m_line = self.distance_to_m_line()
            
            # 1. Asegurarnos de que el robot ya se alejó de la línea inicial
            if dist_m_line > 0.35:
                self.left_m_line = True
            
            # 2. Salir de la pared si cruzamos la línea M Y estamos más cerca de la meta.
            # Quitamos el -0.05 estricto porque el derrape odómetrico nos perjudica. 
            # Pedimos simplemente que la distancia actual sea menor a la distancia de choque.
            if self.left_m_line and dist_m_line < self.m_line_tolerance and dist_to_goal < self.hit_distance:
                self.get_logger().info(f'¡Línea M interceptada a {dist_to_goal:.2f}m! Abandonando pared...')
                self.left_m_line = False
                self.change_state("GO_TO_GOAL")

        # ---------------- ACCIONES DE ESTADO ---------------- #
        if self.state == "GO_TO_GOAL":
            if abs(err_theta) > 0.15:  
                msg.linear.x = 0.0  
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
            else:
                msg.linear.x = self.clamp(self.k_rho * dist_to_goal, -self.v_max, self.v_max)
                msg.angular.z = 0.0  
                
        elif self.state == "WALL_FOLLOWING":
            # Reglas Robustas de Wall Following con reflejo anti-choque
            if self.regions['front'] < 0.25:
                msg.linear.x = 0.0
                msg.angular.z = 0.8
            elif self.regions['front'] < self.d_thresh:
                msg.linear.x = 0.0
                msg.angular.z = 0.6
            elif self.regions['fright'] < self.d_thresh:
                msg.linear.x = 0.15
                msg.angular.z = 0.3
            elif self.regions['right'] < self.d_thresh:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.15
                msg.angular.z = -0.4

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
        
        # VISIÓN 360 GRADOS
        # En ROS, 0° es el frente. Aumenta hacia la izquierda (CCW).
        # Los índices negativos en Python leen el final de la lista (lado derecho).
        if len(clean_ranges) > 0:
            self.regions = {
                # Frente: ±20 grados 
                'front':  min(min(clean_ranges[0:20] + clean_ranges[-20:]), 10.0),
                # Diagonal Izquierda: 21 a 75 grados
                'fleft':  min(min(clean_ranges[21:75]), 10.0),
                # Izquierda pura: 76 a 105 grados
                'left':   min(min(clean_ranges[76:105]), 10.0),
                # Derecha pura: -105 a -76 grados
                'right':  min(min(clean_ranges[-105:-76]), 10.0),
                # Diagonal Derecha: -75 a -21 grados
                'fright': min(min(clean_ranges[-75:-21]), 10.0),
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
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
        
        # Suscriptores y Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)  
        self.create_subscription(Pose2D, "/goal", self.goal_callback, 10) 
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        signal.signal(signal.SIGINT, self.shutdown_function) 

        self.state = "WAITING" 
        self.goal_received = False 
        
        # Posición actual
        self.x = 0.0 
        self.y = 0.0 
        self.theta = 0.0 
        
        # Meta y Línea M (Inicio)
        self.target_x = 0.0 
        self.target_y = 0.0 
        self.start_x = 0.0
        self.start_y = 0.0
        
        # Variables core de Bug 2
        self.hit_distance = float('inf') # Distancia a la meta en el momento del choque
        
        # Parámetros de control general
        self.goal_tolerance = 0.15 
        self.d_thresh = 0.45  
        self.m_line_tolerance = 0.15 # Qué tan cerca debe estar de la línea M para detectarla
        
        # Control suave hacia la meta
        self.k_rho = 0.6
        self.k_alpha = 1.5
        self.v_max = 0.3 
        self.w_max = 1.0

        self.regions = {
            'front': 10.0, 'fleft': 10.0, 'left': 10.0, 'right': 10.0, 'fright': 10.0,
        }

        self.create_timer(0.1, self.control_loop) 
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

    # Función Matemática de la Línea M
    def distance_to_m_line(self):
        """Calcula la distancia perpendicular desde el robot a la línea M."""
        num = abs((self.target_y - self.start_y) * self.x - 
                  (self.target_x - self.start_x) * self.y + 
                  self.target_x * self.start_y - 
                  self.target_y * self.start_x)
        den = math.sqrt((self.target_y - self.start_y)**2 + (self.target_x - self.start_x)**2)
        if den == 0:
            return 0.0
        return num / den

    # --------- LÓGICA PRINCIPAL BUG 2 --------- #
    def control_loop(self): 
        if not self.goal_received:
            return

        msg = Twist()
        
        # Cálculos de distancia
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta = self.normalize_angle(angle_to_goal - self.theta)

        # Condición de Parada
        if dist_to_goal < self.goal_tolerance:
            self.change_state("STOP")
            self.get_logger().info('¡Meta alcanzada exitosamente con Bug 2!')
            self.cmd_pub.publish(Twist()) 
            self.goal_received = False 
            return

        # ---------------- TRANSICIONES DE ESTADO BUG 2 ---------------- #
        if self.state == "GO_TO_GOAL":
            # Si veo pared al frente Y estoy mirando hacia ella (Anti-Chattering)
            if self.regions['front'] < self.d_thresh and abs(err_theta) < 0.35:
                # Guardamos el Hit Point (Distancia a la meta en este instante)
                self.hit_distance = dist_to_goal
                self.get_logger().info(f'Hit Point registrado a {self.hit_distance:.2f}m de la meta.')
                self.change_state("WALL_FOLLOWING")
                
        elif self.state == "WALL_FOLLOWING":
            # BUG 2 REAL: Calcular distancia a la Línea M
            dist_m_line = self.distance_to_m_line()
            
            # Condición de salida: 
            # 1. Tocar la línea M (dist_m_line < tolerancia)
            # 2. Estar más cerca de la meta que cuando chocamos por primera vez
            if dist_m_line < self.m_line_tolerance and dist_to_goal < (self.hit_distance - 0.25):
                self.get_logger().info(f'Línea M interceptada a {dist_to_goal:.2f}m. ¡Abandonando pared!')
                self.change_state("GO_TO_GOAL")

        # ---------------- ACCIONES DE ESTADO ---------------- #
        if self.state == "GO_TO_GOAL":
            # Control Proporcional Suave
            if abs(err_theta) > 0.15:  
                msg.linear.x = 0.0  
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
            else:
                msg.linear.x = self.clamp(self.k_rho * dist_to_goal, -self.v_max, self.v_max)
                msg.angular.z = 0.0  
                
        elif self.state == "WALL_FOLLOWING":
            # Reglas Robustas de Wall Following con reflejo anti-choque
            if self.regions['front'] < 0.25:
                # Freno de emergencia anti-bulldozer
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

    # --------- CALLBACKS --------- #
    def odom_callback(self, msg):  
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):  
        self.target_x = msg.x 
        self.target_y = msg.y 
        
        # ESTO ES CRUCIAL PARA BUG 2: Registramos el inicio de la línea M
        self.start_x = self.x
        self.start_y = self.y
        
        self.goal_received = True 
        self.change_state("GO_TO_GOAL")
        self.get_logger().info(f"Bug 2 Meta: x={self.target_x}, y={self.target_y}. Línea M trazada.")  

    def scan_callback(self, msg): 
        # Limpieza de datos + Anti-Bulldozer
        clean_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r > 10.0:
                clean_ranges.append(10.0)
            elif r < 0.12: 
                clean_ranges.append(0.01) 
            else:
                clean_ranges.append(r)
        
        # Mapeo del LiDAR de 180 grados
        self.regions = {
            'right':  min(clean_ranges[0:71]),    
            'fright': min(clean_ranges[72:143]),  
            'front':  min(clean_ranges[144:215]), 
            'fleft':  min(clean_ranges[216:287]), 
            'left':   min(clean_ranges[288:359]), 
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
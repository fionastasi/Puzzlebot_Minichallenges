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

class Bug0Node(Node):  
    def __init__(self):  
        super().__init__('bug0_node') 
        
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
        
        self.goal_tolerance = 0.15 
        self.d_thresh = 0.45  
        
        # Control proporcional hacia la meta
        self.k_rho = 0.6
        self.k_alpha = 1.5
        self.v_max = 0.2   # Velocidad lineal máxima controlada
        self.w_max = 0.6   # Velocidad angular máxima para evitar derrapes

        self.clean_ranges = [10.0] * 360

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

    # Evalúa si la trayectoria hacia la meta está libre de obstáculos
    def is_path_to_goal_clear(self, err_theta):
        # El sensor LiDAR se indexa en 360° y el frente corresponde al índice 180
        idx = 180 + int(math.degrees(err_theta))
        idx = self.clamp(idx, 0, 359)
        
        # Evalúa el sector de ±15 grados respecto al vector hacia la meta
        inicio = max(0, idx - 15)
        fin = min(359, idx + 15)
        
        # Si la lectura mínima en el sector excede el umbral, la ruta se considera libre
        if min(self.clean_ranges[inicio:fin]) > (self.d_thresh + 0.15):
            return True
        return False

    def control_loop(self): 
        if not self.goal_received:
            return

        msg = Twist()
        
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

        # ---------------- ACCIONES DE ESTADO ---------------- #
        # Estructura de control independiente para evitar bloqueos de velocidad
        if self.state == "GO_TO_GOAL":
            if abs(err_theta) > 0.15:  
                msg.linear.x = 0.0  
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
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

    def odom_callback(self, msg):  
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
        clean_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r > 10.0:
                clean_ranges.append(10.0)
            elif r < 0.12: 
                clean_ranges.append(0.01) 
            else:
                clean_ranges.append(r)
        
        self.clean_ranges = clean_ranges
        
        # Actualiza la indexación del LiDAR para el formato de 360° del robot
        self.regions = {
            'right':  min(clean_ranges[50:110] + [10.0]),   
            'fright': min(clean_ranges[110:160] + [10.0]),  
            'front':  min(clean_ranges[160:200] + [10.0]),  
            'fleft':  min(clean_ranges[200:250] + [10.0]),  
            'left':   min(clean_ranges[250:310] + [10.0]),  
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
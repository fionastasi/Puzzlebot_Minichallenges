#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D 
import numpy as np 
import math
import signal 
import sys 

class Bug0Node(Node):  
    def __init__(self):  
        super().__init__('bug0_node') 
        
        # Publishers y Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10) 
        # Ahora nos suscribimos a /odom que viene de tu nodo de localización
        self.create_subscription(Odometry, "odom", self.odom_cb, 10)  
        self.create_subscription(Pose2D, "goal", self.goal_cb, 10) 
        self.create_subscription(LaserScan, "scan", self.lidar_cb, 10)

        # Handle shutdown gracefully 
        signal.signal(signal.SIGINT, self.shutdown_function) 

        # Variables de estado
        self.state = "WAITING_FOR_GOAL" 
        self.goal_received = False 
        self.lidar = LaserScan()
        
        # Coordenadas y variables
        self.goal_x = 0.0 
        self.goal_y = 0.0 
        self.x = 0.0 
        self.y = 0.0 
        self.theta = 0.0 
        self.goal_tolerance = 0.15 # Tolerancia para llegar a la meta [m]

        # Constantes de control (De tu control.py)
        self.k_rho = 0.8
        self.k_alpha = 1.5
        self.v_max = 0.3 # Reducido un poco para mayor seguridad con el Bug0
        self.w_max = 1.0

        # Constantes de Wall Following
        self.v_wall = 0.1     
        self.kw_wall = 0.8    
        self.d_start = 0.5    # Distancia para empezar a esquivar [m]
        self.d_clear = 1.0    # Distancia para considerar que el camino está libre [m]

        self.cmd_vel = Twist() 
        timer_period = 0.05  # 20 Hz para control más suave
        self.timer = self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Nodo Bug 0 inicializado!! Esperando meta...") 

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def main_timer_cb(self): 
        if not self.goal_received or not self.lidar.ranges:
            return

        # 1. Cálculos de control polar hacia la meta (de tu control.py)
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        rho = math.sqrt(dx**2 + dy**2)
        alpha = math.atan2(dy, dx) - self.theta
        alpha = self.normalize_angle(alpha)
        
        # Obtener info del obstáculo más cercano
        closest_range, theta_closest = self.get_closest_object()

        # ---------------- MÁQUINA DE ESTADOS ---------------- #

        # Condición de parada (Prioridad 1)
        if rho < self.goal_tolerance:
            if self.state != "STOP":
                self.get_logger().info("¡Meta alcanzada!")
                self.state = "STOP"
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        # Estado: GO TO GOAL
        elif self.state == "GO_TO_GOAL":
            # Si hay un obstáculo cerca Y está frente al robot
            if closest_range < self.d_start and abs(theta_closest) < np.pi/2:
                self.get_logger().info("¡Obstáculo detectado! Cambiando a WALL_FOLLOWING")
                self.state = "WALL_FOLLOWING"
            else:
                # Lógica Polar: Primera etapa rotar, segunda avanzar
                if abs(alpha) > 0.1:  
                    self.cmd_vel.linear.x = 0.0  
                    self.cmd_vel.angular.z = self.clamp(self.k_alpha * alpha, -self.w_max, self.w_max)
                else:
                    self.cmd_vel.linear.x = self.clamp(self.k_rho * rho, -self.v_max, self.v_max)
                    self.cmd_vel.angular.z = 0.0  

        # Estado: WALL FOLLOWING
        elif self.state == "WALL_FOLLOWING":
            # Guardamos el ángulo directo a la meta para saber si está libre
            etheta = math.atan2(dy, dx) - self.theta
            etheta = self.normalize_angle(etheta)

            if self.is_path_to_goal_clear(etheta):
                self.get_logger().info("¡Camino libre! Cambiando a GO_TO_GOAL")
                self.state = "GO_TO_GOAL"
            else:
                # Lógica de Wall Following Counter-Clockwise
                theta_ao = self.get_theta_ao(theta_closest) 
                theta_fw = self.get_theta_fw(theta_ao, direction="fwcc") 
                self.cmd_vel.linear.x = self.v_wall
                self.cmd_vel.angular.z = self.kw_wall * theta_fw

        self.cmd_vel_pub.publish(self.cmd_vel) 

    # --------- FUNCIONES DEL LIDAR Y OBSTÁCULOS --------- #
    def get_closest_object(self): 
        closest_range = min(self.lidar.ranges) 
        closest_index = self.lidar.ranges.index(closest_range) 
        theta_closest = self.lidar.angle_min + closest_index * self.lidar.angle_increment 
        theta_closest = self.normalize_angle(theta_closest) 
        return closest_range, theta_closest 

    def get_theta_ao(self, theta_closest): 
        theta_ao = theta_closest + math.pi 
        return self.normalize_angle(theta_ao)

    def get_theta_fw(self, theta_ao, direction="fwcc"): 
        if direction == "fwcc":  
            theta_fw = theta_ao - math.pi / 2  
        elif direction == "fwc":  
            theta_fw = theta_ao + math.pi / 2  
        return self.normalize_angle(theta_fw)

    def is_path_to_goal_clear(self, etheta):
        idx = int((etheta - self.lidar.angle_min) / self.lidar.angle_increment)
        if 0 <= idx < len(self.lidar.ranges):
            margen_grados = 25
            idx_margen = int(math.radians(margen_grados) / self.lidar.angle_increment)
            inicio = max(0, idx - idx_margen)
            fin = min(len(self.lidar.ranges) - 1, idx + idx_margen)
            distancias_hacia_meta = self.lidar.ranges[inicio:fin]
            
            if min(distancias_hacia_meta) > self.d_clear:
                return True
        return False

    # --------- CALLBACKS --------- #
    def odom_cb(self, msg):  
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

    def goal_cb(self, goal):  
        self.goal_x = goal.x 
        self.goal_y = goal.y 
        self.goal_received = True 
        self.state = "GO_TO_GOAL"
        self.get_logger().info(f"¡Meta Recibida! Vamos a x:{self.goal_x}, y:{self.goal_y}")  

    def lidar_cb(self, lidar_msg): 
        self.lidar = lidar_msg  

    def shutdown_function(self, signum, frame): 
        self.get_logger().info("Apagando. Deteniendo robot...") 
        stop_twist = Twist()  
        self.cmd_vel_pub.publish(stop_twist) 
        rclpy.shutdown() 
        sys.exit(0) 

def main(args=None): 
    rclpy.init(args=args) 
    my_node = Bug0Node() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 
     
if __name__ == '__main__': 
    main()
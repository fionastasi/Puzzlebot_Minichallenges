import rclpy 
import numpy as np

from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist
 

class LaserScanSub(Node): 

    def __init__(self): 

        super().__init__('laser_scan_subscriber') 

        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.robot_vel = Twist() #Required robot velocity

        self.lidar = LaserScan() # Data from lidar will be stored here. 
        self.d_safety = 0.45 # Distance to keep from the closes obj [m]
        self.d_start = 0.80 # Distance to start avoiding the object [m]
        self.kv = 1.0 # Linear speed gain
        self.kw = 1.0 # Angular Speed Proportional Gain
        timer_period = 0.1 #10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        self.get_logger().info("Node initialized!!!") 

    
    def get_closest_object_info(self):
        ranges = self.lidar.ranges
        closest_range = min(ranges)
        closest_index = ranges.index(closest_range)
        theta_closest = self.lidar.angle_min + closest_index * self.lidar.angle_increment
        theta_closest = np.atan2(np.sin(theta_closest), np.cos(theta_closest)) # Normalize angle to [-pi, pi]

        return closest_range, theta_closest
     

    def timer_callback(self): 

    #print("NO ESTAMOS NI ENTRANDO AL IF") 

        #### ADD YOUR CODE ### 

        """"
        print("Angle min: ", self.lidar.angle_min)
        print("Angle max: ", self.lidar.angle_max)
        print("Range min: ", self.lidar.range_min)
        print("Range max: ", self.lidar.range_max)
        print("Header frame id: ", self.lidar.header.frame_id)
        print("First component inside the ranges[] array: ", self.lidar.ranges[0])
        print("Last component inside the intensities[] array: ", self.lidar.intensities[-1])
        """

        if self.lidar.ranges:
            closest_range, theta_closest = self.get_closest_object_info()

            theta_avoidance = theta_closest - np.pi if theta_closest > 0 else theta_closest + np.pi
            theta_avoidance = np.atan2(np.sin(theta_avoidance), np.cos(theta_avoidance)) # Normalize angle to [-pi, pi]

            # Just take in consideration the front 180 degrees of the lidar scan for obstacle avoidance.
            if abs(theta_closest) > np.pi / 2:
                print("Object is behind the robot. Ignoring it for obstacle avoidance.")
                self.robot_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.robot_vel)
                return
            else:
                if closest_range < self.d_safety:
                    print("Too close to the object! Stopping the robot.")
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.robot_vel)
                    return
                elif closest_range < self.d_start:
                    print("Object detected within the start distance. Avoiding it.")
                    d_diff = closest_range - self.d_safety
                    v = self.kv *d_diff
                    w = self.kw * theta_avoidance
                    self.robot_vel.linear.x = v
                    self.robot_vel.angular.z = w
                    self.cmd_vel_pub.publish(self.robot_vel)
                    return
                else:
                    print("No obstacles within the start distance. Moving forward.")
                    self.robot_vel.linear.x = 0.30
                    self.cmd_vel_pub.publish(self.robot_vel)
                    return

        else:
            print("No valid range data received yet.")

 
    def lidar_cb(self, lidar_msg): 

        ## This function receives the ROS LaserScan message 

        self.lidar =  lidar_msg  

 

def main(args=None): 

    rclpy.init(args=args) 

    m_p=LaserScanSub() 

    rclpy.spin(m_p) 

    m_p.destroy_node() 

    rclpy.shutdown() 

     

if __name__ == '__main__': 

    main() 
import rclpy 

from rclpy.node import Node 

from sensor_msgs.msg import LaserScan 

from geometry_msgs.msg import Twist 

import numpy as np  

import signal  

import sys 

 

class LaserScanSub(Node): 

    def __init__(self): 

        super().__init__('avoid_obstacle') 

        # Handle shutdown gracefully 

        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

         

        self.sub = self.create_subscription(LaserScan, "scan", self.lidar_cb, 10) 

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 

        self.lidar = LaserScan() # Data from lidar will be stored here. 

        self.d_safety = 0.2 # Stop the robot whenever an object is this close 

        self.v = 0.2 # Linear speed to the front 

        self.kw = 1.9 # Angular speed proportional gain 

        self.robot_vel = Twist() # The required robot velocity  

        timer_period = 0.1 #10 Hz 

        self.timer = self.create_timer(timer_period, self.timer_callback) 

        self.get_logger().info("Node initialized!!!") 

     

    def timer_callback(self): 

        if self.lidar.ranges: #Check that you have received at least one message from the lidar  

            #Get the closest object 

            closest_range, theta_closest = self.get_closest_object() 

            print("closest_range: ", closest_range) 

            print("theta_closest: ", theta_closest) 

            if np.isinf(closest_range): 

                print("There are no objects around") 

                v = self.v #Move to the front 

                w = 0.0 

            else:  

                #logic code here

                # Counter-clockwise wall-following logic
                theta_ao = self.get_theta_ao(theta_closest)  # Angle to avoid the obstacle
                theta_fw = self.get_theta_fw(theta_ao, direction="fwcc")  # Angle for wall-following (counter-clockwise)

                # Proportional controller for angular velocity
                w = self.kw * theta_fw

                # Maintain a constant linear velocity
                v = self.v

            self.robot_vel.linear.x = v 

            self.robot_vel.angular.z = w 

            self.cmd_vel_pub.publish(self.robot_vel) 

        else: 

            print("No lidar data received") 

 

    def get_closest_object(self): 

        # This function uses the self.lidar data and returns  

        # the range and angle to the closest object detected by the lidar. 

        closest_range = min(self.lidar.ranges) 

        closest_index = self.lidar.ranges.index(closest_range) 

        theta_closest = self.lidar.angle_min + closest_index*self.lidar.angle_increment 

        # Crop this angle to (-pi, pi] 

        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest)) 

        return closest_range, theta_closest 

 

    def get_theta_ao(self, theta_closest): 

        #This function takes as input the angle to the closest object (theta_closest) 

        # returns the angle to avoid the obstacle theta_ao 

        theta_ao = theta_closest+np.pi 

        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao)) 

        return theta_ao 

 

 

    def get_theta_fw(self, theta_ao, direction="fwcc"): 
        # This function calculates the desired wall-following angle
        # direction="fwcc" for counter-clockwise, "fwc" for clockwise

        if direction == "fwcc":  # Counter-clockwise wall-following
            theta_fw = theta_ao - np.pi / 2  # Offset by -90 degrees
        elif direction == "fwc":  # Clockwise wall-following
            theta_fw = theta_ao + np.pi / 2  # Offset by +90 degrees
        else:
            raise ValueError("Invalid direction. Use 'fwcc' or 'fwc'.")

        # Normalize the angle to the range (-pi, pi]
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
        return theta_fw

     

    def lidar_cb(self, lidar_msg): 

        ## This function receives the ROS LaserScan message 

        self.lidar =  lidar_msg  

 

    def shutdown_function(self, signum, frame): 

        # Handle shutdown gracefully 

        # This function will be called when Ctrl+C is pressed 

        # It will stop the robot and shutdown the node 

        self.get_logger().info("Shutting down. Stopping robot...") 

        stop_twist = Twist()  # All zeros to stop the robot 

        self.cmd_vel_pub.publish(stop_twist) # publish it to stop the robot before shutting down 

        rclpy.shutdown() # Shutdown the node 

        sys.exit(0) # Exit the program 

 

def main(args=None): 

    rclpy.init(args=args) 

    m_p=LaserScanSub() 

    rclpy.spin(m_p) 

    m_p.destroy_node() 

    rclpy.shutdown() 

     

if __name__ == '__main__': 

    main() 
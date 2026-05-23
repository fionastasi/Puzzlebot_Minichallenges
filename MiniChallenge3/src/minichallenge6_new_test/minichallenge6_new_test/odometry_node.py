#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32 
from rclpy import qos 
import numpy as np 

 

#This class will compute the pose of the robot from the encoder readings. 
# This node subscribes to the /VelocityEncR and /VelocityEncL topics 
# This node publishes the pose of the robot to the /pose topic.  

class Odometry(Node):  
    def __init__(self):  
        super().__init__('odometry_node') 
        ###########  INIT PUBLISHERS ################ 
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)  
        ############## SUBSCRIBERS ##################  
        self.create_subscription(Float32, "VelocityEncR",  self.wr_cb, qos.qos_profile_sensor_data)  
        self.create_subscription(Float32, "VelocityEncL",  self.wl_cb, qos.qos_profile_sensor_data)  
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius for our simulated robot[m] 
        self.L=0.19 #wheel separation for our simulated robot [m] 
        self.wl = 0.0 #Left wheel speed [rad/s] 
        self.wr = 0.0 #Right wheel speed [rad/s] 
        self.x = 0.0 #Robot position in x-axis [m] 
        self.y = 0.0 #Robot position in y-axis [m] 
        self.theta = 0.0 #Robot orientation [rad] 
        self.robot_pose = Pose2D() 
        self.prev_time_ns = self.get_clock().now().nanoseconds
        timer_period = 0.05 
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 

 

 

             

    def main_timer_cb(self): 
        v, w = self.get_robot_vel(self.wr, self.wl)
        self.update_robot_pose(v, w) #update the robot pose (self.x, self.y, self.theta)

        print("xr: " + str(self.x))
        print("yr: " + str(self.y))
        print("theta_r: " + str(self.theta))

        self.robot_pose.x = self.x
        self.robot_pose.y = self.y
        self.robot_pose.theta = self.theta


        self.pub_pose.publish(self.robot_pose) #publish the robot's speed  

 
    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data 

         
    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data 

    def get_robot_vel(self, wr, wl):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        print("v: " + str(v))
        print("w: " + str(w))
        return v, w
    
    def update_robot_pose(self, v, w):
        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns) *10**-9 #get delta time in seconds
        self.prev_time_ns = self.get_clock().now().nanoseconds
        self.x = self.x + v *np.cos(self.theta) * dt
        self.y = self.y + v *np.sin(self.theta) * dt
        self.theta = self.theta + w * dt
        #limit theta to be between -pi and pi
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        print("x: " + str(self.x))
        print("y: " + str(self.y))
        print("theta: " + str(self.theta))

def main(args=None): 
    rclpy.init(args=args) 
    my_node=Odometry() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 
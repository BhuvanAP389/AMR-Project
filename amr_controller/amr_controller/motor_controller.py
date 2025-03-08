#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Declare parameters
        self.declare_parameter("wheel_radius_front_left", 0.065)
        self.declare_parameter("wheel_radius_middle_left", 0.065)
        self.declare_parameter("wheel_radius_back_left", 0.055)
        self.declare_parameter("wheel_radius_front_right", 0.065)
        self.declare_parameter("wheel_radius_middle_right", 0.055)
        self.declare_parameter("wheel_radius_back_right", 0.055)
        self.declare_parameter("wheel_separation", 0.253)
        self.declare_parameter("wheel_separation_front_middle", 0.155)
        self.declare_parameter("wheel_separation_middle_back", 0.130362)
       

        # Retrieve parameters
        self.wheel_radius_front_left_ = self.get_parameter("wheel_radius_front_left").value
        self.wheel_radius_middle_left_ = self.get_parameter("wheel_radius_middle_left").value
        self.wheel_radius_back_left_ = self.get_parameter("wheel_radius_back_left").value
        self.wheel_radius_front_right_ = self.get_parameter("wheel_radius_front_right").value
        self.wheel_radius_middle_right_ = self.get_parameter("wheel_radius_middle_right").value
        self.wheel_radius_back_right_ = self.get_parameter("wheel_radius_back_right").value
        self.wheel_separation_ = self.get_parameter("wheel_separation").value
        self.wheel_separation_front_middle_ = self.get_parameter("wheel_separation_front_middle").value
        self.wheel_separation_middle_back_ = self.get_parameter("wheel_separation_middle_back").value
        

        # Create publisher and subscriber
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.corner_wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_position_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "amr_controller/cmd_vel", self.vel_callback, 10)

    def vel_callback(self, msg):
        """
        Callback function that converts linear velocity to wheel speeds and publishes commands.
        """
        linear_speed = msg.twist.linear.x  
        angular_speed = msg.twist.angular.z
        epsilon = 1e-6  # Small value to prevent division by zero

        a = linear_speed + (angular_speed * self.wheel_separation_ * 0.5)
        b = linear_speed - (angular_speed * self.wheel_separation_ * 0.5)
        c = angular_speed * self.wheel_separation_front_middle_
        d = angular_speed * self.wheel_separation_middle_back_

        # Define vectors
        Vfl = np.array([b, c])  
        Vfr = np.array([a, c])  
        Vml = np.array([b, 0]) 
        Vmr = np.array([a, 0]) 
        Vbl = np.array([b, -d]) 
        Vbr = np.array([a, -d]) 

        front_left_speed = np.linalg.norm(Vfl) / self.wheel_radius_front_left_
        middle_left_speed = np.linalg.norm(Vml) / self.wheel_radius_middle_left_
        back_left_speed = np.linalg.norm(Vbl) / self.wheel_radius_back_left_
        front_right_speed = np.linalg.norm(Vfr) / self.wheel_radius_front_right_
        middle_right_speed = np.linalg.norm(Vmr) / self.wheel_radius_middle_right_
        back_right_speed = np.linalg.norm(Vbr) / self.wheel_radius_back_right_

        front_left_speed *= np.sign(b)
        middle_left_speed *= np.sign(b)
        back_left_speed *= np.sign(b)
        front_right_speed *= np.sign(a)
        middle_right_speed *= np.sign(a)
        back_right_speed *= np.sign(a)

        front_left_position = math.atan(c / (b + epsilon))
        back_left_position = math.atan(-d / (b + epsilon))
        front_right_position = math.atan(c / (a + epsilon))
        back_right_position = math.atan(-d / (a + epsilon))

        # Publish wheel speeds and positions
        msg_out_speed = Float64MultiArray()
        msg_out_corner = Float64MultiArray()
        msg_out_speed.data = [front_left_speed, middle_left_speed, back_left_speed,
                              front_right_speed, middle_right_speed, back_right_speed]
        msg_out_corner.data = [front_left_position, front_right_position,
                               back_left_position, back_right_position]

        self.corner_wheel_cmd_pub_.publish(msg_out_corner)
        self.wheel_cmd_pub_.publish(msg_out_speed)

        self.get_logger().info(f"Published wheel speeds: {msg_out_speed.data}")
        self.get_logger().info(f"Published wheel position: {msg_out_corner.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

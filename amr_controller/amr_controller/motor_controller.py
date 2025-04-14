#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, TransformStamped
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Declare parameters
        self.declare_parameter("wheel_radius_front_left", 0.065)
        self.declare_parameter("wheel_radius_middle_left", 0.055)
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
        
        self.middle_left_wheel_prev_pos_ = 0.0
        self.middle_right_wheel_prev_pos_  = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Create publisher and subscriber
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.corner_wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_position_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "amr_controller/cmd_vel", self.vel_callback, 10)
        self.joint_sub = self.create_subscription(JointState,"joint_states",self.jointCallback, 10)

        self.odom_pub_ = self.create_publisher(Odometry,"amr_controller/odom", 10)

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

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

        #self.get_logger().info(f"Published wheel speeds: {msg_out_speed.data}")
        #self.get_logger().info(f"Published wheel position: {msg_out_corner.data}")

    
    def jointCallback(self, msg):
        dp_middle_left = msg.position[2] - self.middle_left_wheel_prev_pos_
        dp_middle_right = msg.position[4] - self.middle_right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.middle_left_wheel_prev_pos_ = msg.position[2]
        self.middle_right_wheel_prev_pos_ = msg.position[4]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_middle_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_middle_right / (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius_middle_right_* fi_right + self.wheel_radius_middle_left_ * fi_left) /2
        angular = (self.wheel_radius_middle_right_* fi_right - self.wheel_radius_middle_left_ * fi_left) / self.wheel_separation_

        d_s = (self.wheel_radius_middle_right_ * dp_middle_right + self.wheel_radius_middle_left_ * dp_middle_left) /2
        d_theta = (self.wheel_radius_middle_right_ * dp_middle_right - self.wheel_radius_middle_left_ * dp_middle_left)/ self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
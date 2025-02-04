#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        #  Declare parameters
        self.declare_parameter("wheel_radius_front_left", 0.065)
        self.declare_parameter("wheel_radius_middle_left", 0.065)
        self.declare_parameter("wheel_radius_rear_left", 0.055)
        self.declare_parameter("wheel_radius_front_right", 0.065)
        self.declare_parameter("wheel_radius_middle_right", 0.055)
        self.declare_parameter("wheel_radius_rear_right", 0.055)
        self.declare_parameter("wheel_separation", 0.233)

        #  Corrected parameter retrieval
        self.wheel_radius_front_left_ = self.get_parameter("wheel_radius_front_left").value
        self.wheel_radius_middle_left_ = self.get_parameter("wheel_radius_middle_left").value
        self.wheel_radius_rear_left_ = self.get_parameter("wheel_radius_rear_left").value
        self.wheel_radius_front_right_ = self.get_parameter("wheel_radius_front_right").value
        self.wheel_radius_middle_right_ = self.get_parameter("wheel_radius_middle_right").value
        self.wheel_radius_rear_right_ = self.get_parameter("wheel_radius_rear_right").value
        self.wheel_separation_ = self.get_parameter("wheel_separation").value

        # Create publisher and subscriber
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "amr_controller/cmd_vel", self.vel_callback, 10)

    def vel_callback(self, msg):
        """
        Callback function that converts linear velocity to wheel speeds and publishes commands.
        """
        linear_speed = msg.twist.linear.x  # Only forward/backward velocity

        #  Compute wheel speeds
        front_left_speed = linear_speed / self.wheel_radius_front_left_
        middle_left_speed = linear_speed / self.wheel_radius_middle_left_
        rear_left_speed = linear_speed / self.wheel_radius_rear_left_

        front_right_speed = linear_speed / self.wheel_radius_front_right_
        middle_right_speed = linear_speed / self.wheel_radius_middle_right_
        rear_right_speed = linear_speed / self.wheel_radius_rear_right_

        # Convert NumPy values to standard Python floats
        msg_out = Float64MultiArray()
        msg_out.data = [
            float(front_left_speed), float(middle_left_speed), float(rear_left_speed),
            float(front_right_speed), float(middle_right_speed), float(rear_right_speed)
        ]
        self.wheel_cmd_pub_.publish(msg_out)

        self.get_logger().info(f"Published wheel speeds: {msg_out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

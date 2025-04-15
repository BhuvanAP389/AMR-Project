#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np

class DeliveryBoxStabilizer(Node):
    def __init__(self):
        super().__init__("delivery_box_stabilizer")


        self.del_angle_cmd_pub_ = self.create_publisher(Float64MultiArray, "/delivery_box_controller/commands", 10)
        self.imu_sub_ = self.create_subscription(Imu, "/imu/out", self.imu_callback, 10)

        self.get_logger().info(" Delivery Box Stabilizer Node Started")

    def imu_callback(self, msg):

        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w

        # Convert to pitch
        pitch = self.quaternion_to_pitch(q_x, q_y, q_z, q_w)

        # Publish correction angle
        self.publish_angle_command(-pitch)

    def quaternion_to_pitch(self, x, y, z, w):
        # Compute pitch from quaternion
        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)  # Clip to valid range for arcsin
        pitch = np.arcsin(sin_pitch)
        return pitch

    def publish_angle_command(self, angle):
        msg = Float64MultiArray()
        msg.data = [angle]
        self.del_angle_cmd_pub_.publish(msg)
        self.get_logger().info(f"Published correction angle: {angle:.2f} radians")

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryBoxStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

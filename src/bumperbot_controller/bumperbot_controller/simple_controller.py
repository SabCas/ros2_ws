#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_seperation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_seperation_ = self.get_parameter("wheel_seperation").get_parameter_value().double_value

        self.get_logger().info(f"wheel_radius_: {self.wheel_radius_}")
        self.get_logger().info(f"wheel_seperation_: {self.wheel_seperation_}")

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,
            "simple_velocity_controller/commands",
            10,
        )
        self.velocity_sub_ = self.create_subscription(
            TwistStamped,
            "bumperbot_controller/cmd_vel",
            self.velocity_callback,
            10,
        )

        self.speed_conerversion_ = np.array(
            [
                [self.wheel_radius_/2, self.wheel_radius_/2],
                [self.wheel_radius_/ self.wheel_seperation_, -self.wheel_radius_/ self.wheel_seperation_],
            ]
        )
        self.get_logger().info(f"speed_conerversion_: {self.speed_conerversion_}")
    def velocity_callback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x], [msg.twist.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conerversion_), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = wheel_speed.flatten().tolist()
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
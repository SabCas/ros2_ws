import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")
        self.turtle1_pose_sub_ = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.turtle1_pose_callback,
            10,
        )
        self.turtle2_pose_sub_ = self.create_subscription(
            Pose,
            "/turtle2/pose",
            self.turtle2_pose_callback,
            10,
        )

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1_pose_callback(self, msg):
        self.last_turtle1_pose_ = msg

    def turtle2_pose_callback(self, msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y
        theta_rad =  self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg =  theta_rad * 180 / 3.14159265358979323846
        R11 = math.cos(theta_rad)
        R12 = -math.sin(theta_rad)
        R21 = math.sin(theta_rad)
        R22 = math.cos(theta_rad)
        self.get_logger().info(f"Tx: {Tx}, Ty: {Ty}/n")
        self.get_logger().info(f"theta_deg: {theta_deg}/n")
        self.get_logger().info(f"R11    R12: {R11} {R12}/n")
        self.get_logger().info(f"R21    R22: {R21} {R22}/n")

def main(args=None):
            rclpy.init(args=args)
            simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
            rclpy.spin(simple_turtlesim_kinematics)
            simple_turtlesim_kinematics.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
        main()




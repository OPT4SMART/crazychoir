import rclpy
from rclpy.node import Node
from crazychoir.guidance import GoToGuidance
import time


def main():
    rclpy.init()

    take_ff_time = 5.0

    guidance = GoToGuidance(pose_handler='pubsub', pose_topic='odom', pose_callback=None, takeoff_time=take_ff_time)

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

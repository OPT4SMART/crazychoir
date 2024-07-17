import rclpy
from rclpy.node import Node
from crazychoir.guidance import GoToGuidance
import time


def main():
    rclpy.init()

    takeoff_time = 5.0

    guidance = GoToGuidance(pose_handler='pubsub', pose_topic='cf_odom', pose_callback=None, takeoff_time=takeoff_time)

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

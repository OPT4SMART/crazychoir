import rclpy
from rclpy.node import Node
from crazychoir.guidance import BearingFormation
import time


def main():
    rclpy.init()

    frequency = 100

    guidance = BearingFormation(update_frequency=frequency, pose_handler='pubsub', pose_topic='odom')

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

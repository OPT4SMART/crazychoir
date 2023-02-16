import rclpy
from crazychoir.guidance import SimpleGuidance
import time


def main():
    rclpy.init()

    frequency = 100

    guidance = SimpleGuidance(pose_handler='pubsub', pose_topic='odom')

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

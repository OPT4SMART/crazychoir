import rclpy
from crazychoir.guidance import SimpleGuidance

def main():
    rclpy.init()

    guidance = SimpleGuidance(pose_handler='pubsub', pose_topic='odom')

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

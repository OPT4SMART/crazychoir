import rclpy
from crazychoir.planner.trajectory_handler import SplinePosition

def main():
    rclpy.init()

    frequency = 10

    trajectory = SplinePosition(update_frequency = frequency, pose_handler = 'pubsub', pose_topic = 'odom')
    
    trajectory.get_logger().info('Go!')

    rclpy.spin(trajectory)
    rclpy.shutdown()

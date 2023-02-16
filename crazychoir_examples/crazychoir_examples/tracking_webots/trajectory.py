import rclpy
from crazychoir.planner.trajectory_handler import Spline

def main():
    rclpy.init()

    frequency = 100 

    trajectory = Spline(update_frequency = frequency, pose_handler = 'pubsub', pose_topic = 'odom')
    
    trajectory.get_logger().info('Go!')

    rclpy.spin(trajectory)
    rclpy.shutdown()

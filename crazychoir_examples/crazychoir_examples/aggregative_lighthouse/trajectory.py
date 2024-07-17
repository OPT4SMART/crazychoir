import rclpy
from crazychoir.planner.trajectory_handler import Spline, SplinePosition
from rclpy.node import Node

def main():
    rclpy.init()

    params = Node('traj_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    frequency = params.get_parameter('freq').value
    
    trajectory = SplinePosition(update_frequency = frequency, pose_handler = 'pubsub', pose_topic = 'cf_odom')
    
    trajectory.get_logger().info('Go!')

    rclpy.spin(trajectory)
    rclpy.shutdown()

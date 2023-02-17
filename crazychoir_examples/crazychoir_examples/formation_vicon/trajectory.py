import rclpy
from rclpy.node import Node

from crazychoir.planner.trajectory_handler import Spline
from crazychoir.filtering import Estimator

def main():
    rclpy.init()

    trajectory_params = Node('trajectory_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    vicon_id = trajectory_params.get_parameter('vicon_id').value

    frequency = 100
    callback = Estimator()
    trajectory = Spline(update_frequency = frequency, pose_handler = 'vicon', pose_topic = '/vicon/cf{}/cf{}'.format(vicon_id,vicon_id), pose_callback = callback)

    callback.set_pose(trajectory.current_pose) 

    trajectory.get_logger().info('Go!')

    rclpy.spin(trajectory)
    rclpy.shutdown()

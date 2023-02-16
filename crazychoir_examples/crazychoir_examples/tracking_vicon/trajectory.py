import rclpy
from crazychoir.planner.trajectory_handler import Spline
from crazychoir.filtering import Estimator


def main():
    rclpy.init()

    frequency = 100 
    callback = Estimator()

    trajectory = Spline(update_frequency = frequency, pose_handler='vicon', pose_topic='/vicon/cf1/cf1', pose_callback=callback)

    callback.set_pose(trajectory.current_pose)

    trajectory.get_logger().info('Go!')

    rclpy.spin(trajectory)
    rclpy.shutdown()

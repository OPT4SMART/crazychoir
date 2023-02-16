import rclpy
from crazychoir.guidance import SimpleGuidance
from crazychoir.filtering import Estimator

def main():
    rclpy.init()

    frequency = 100
    
    callback = Estimator()

    guidance = SimpleGuidance(pose_handler='vicon', pose_topic='/vicon/cf1/cf1', pose_callback=callback)
    
    callback.set_pose(guidance.current_pose)

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from crazychoir.guidance import BearingFormation
from crazychoir.filtering import Estimator

def main():
    rclpy.init()

    guidance_params = Node('guidance_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    vicon_id = guidance_params.get_parameter('vicon_id').value

    frequency = 100
    callback = Estimator()
    guidance = BearingFormation(update_frequency=frequency, pose_handler='vicon', pose_topic='/vicon/cf{}/cf{}'.format(vicon_id,vicon_id), pose_callback=callback)

    callback.set_pose(guidance.current_pose) 

    rclpy.spin(guidance)
    rclpy.shutdown()

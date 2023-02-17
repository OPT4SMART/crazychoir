import rclpy
from rclpy.node import Node
from crazychoir.guidance import SimpleGuidance

def main():
    rclpy.init()

    simple_guidance_params = Node('simple_guidance_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    vicon_id = simple_guidance_params.get_parameter('vicon_id').value

    simple_guidance = SimpleGuidance(pose_handler='vicon', pose_topic='/vicon/cf{}/cf{}'.format(vicon_id,vicon_id))
    simple_guidance.get_logger().info('Go!')

    rclpy.spin(simple_guidance)
    rclpy.shutdown()

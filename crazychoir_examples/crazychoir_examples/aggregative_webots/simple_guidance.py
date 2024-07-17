import rclpy
from crazychoir.guidance import SimpleGuidance
from rclpy.node import Node

def main():
    rclpy.init()

    node = Node('simpl_guid_cf', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value
    
    # height = 0.5 + agent_id*0.15
    height = 1.0
    takeoff_time = 3.0

    guidance = SimpleGuidance(pose_handler='pubsub', pose_topic='odom', takeoff_time = takeoff_time, height=height)

    guidance.get_logger().info('Go!')

    rclpy.spin(guidance)
    rclpy.shutdown()

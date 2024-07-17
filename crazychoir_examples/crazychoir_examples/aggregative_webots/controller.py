import rclpy
from crazychoir.controller import VelocityController, VelocityCtrlStrategy
from crazychoir.planner import PositionTraj, FullStateTraj
from crazychoir.crazyflie_communication import VelSender
import time
from rclpy.node import Node

def main():
    rclpy.init()

    params = Node('contr_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    frequency = params.get_parameter('freq').value

    velocity_strategy = VelocityCtrlStrategy(frequency)
    sender = VelSender(frequency)
    # desired_reference = FullStateTraj()
    desired_reference = PositionTraj()

    controller = VelocityController(pose_handler='pubsub', pose_topic='odom', pose_callback=None,
                velocity_strategy=velocity_strategy,
                command_sender=sender, traj_handler=desired_reference)

    controller.get_logger().info('Go!')

    rclpy.spin(controller)
    rclpy.shutdown()
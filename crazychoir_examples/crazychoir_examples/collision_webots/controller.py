import rclpy
from crazychoir.controller import HierarchicalController, GeometryAttitudeCtrl, FlatnessPositionCtrl, SafeFlatnessPositionCtrl, SafeHierarchicalController
from crazychoir.planner import FullStateTraj
from crazychoir.crazyflie_communication import FPQRSender

from choirbot.collision.safety_filters import DoubleIntegratorCBF

from rclpy.node import Node

import time

def main():
    rclpy.init()


    planner_params = Node('controller_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = planner_params.get_parameter('agent_id').value


    frequency = 100
    # position_ctrl = FlatnessPositionCtrl(frequency)

    safe_distance = 0.2             # m
    gamma = 5.0                   # safety parameter: The lower, the safer
    kappa = 1.0                     # safety parameter {1,2}: 
    max_acceleration = 0.005        # m/s^2
    max_braking_acceleration = 0.005  # m/s^2

    safety_filter = DoubleIntegratorCBF(
        distance = safe_distance, 
        gamma = gamma, 
        kappa = kappa,
        max_acceleration = max_acceleration,
        max_braking_acceleration = max_braking_acceleration,              # m/s^2
        )


    position_ctrl = SafeFlatnessPositionCtrl(agent_id, frequency, safety_filter = safety_filter, max_acceleration=max_acceleration)
    desired_traj = FullStateTraj()
    attitude_ctrl = GeometryAttitudeCtrl(frequency)
    sender = FPQRSender(frequency)

    controller = SafeHierarchicalController(pose_handler='pubsub', pose_topic='odom', pose_callback=None,
                position_strategy=position_ctrl, attitude_strategy=attitude_ctrl, 
                command_sender=sender, traj_handler=desired_traj)

    controller.get_logger().info('Go!')

    rclpy.spin(controller)
    rclpy.shutdown()


import rclpy
from crazychoir.controller import HierarchicalController, GeometryAttitudeCtrl, FlatnessPositionCtrl
from crazychoir.planner import FullStateTraj
from crazychoir.crazyflie_communication import FPQRSender
import time

def main():
    rclpy.init()

    frequency = 100
    position_ctrl = FlatnessPositionCtrl(frequency)
    desired_traj = FullStateTraj()
    attitude_ctrl = GeometryAttitudeCtrl(frequency)
    sender = FPQRSender(frequency)

    controller = HierarchicalController(pose_handler='pubsub', pose_topic='odom', pose_callback=None,
                position_strategy=position_ctrl, attitude_strategy=attitude_ctrl, 
                command_sender=sender, traj_handler=desired_traj)

    controller.get_logger().info('Go!')

    rclpy.spin(controller)
    rclpy.shutdown()


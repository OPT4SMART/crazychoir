import rclpy
from crazychoir.controller import HierarchicalController, GeometryAttitudeCtrl, FlatnessPositionCtrl
from crazychoir.planner import FullStateTraj
from crazychoir.crazyflie_communication import FPQRSender
from crazychoir.filtering import Estimator

def main():
    rclpy.init()

    frequency = 100
    position_ctrl = FlatnessPositionCtrl(update_frequency=frequency, vicon=True)
    desired_traj = FullStateTraj()
    attitude_ctrl = GeometryAttitudeCtrl(update_frequency=frequency, vicon=True)
    sender = FPQRSender(frequency)

    callback = Estimator()

    controller = HierarchicalController(pose_handler='vicon', pose_topic='/vicon/cf1/cf1', pose_callback=callback,
                position_strategy=position_ctrl, attitude_strategy=attitude_ctrl, 
                command_sender=sender, traj_handler=desired_traj)

    callback.set_pose(controller.current_pose)

    controller.get_logger().info('Go!')

    rclpy.spin(controller)
    rclpy.shutdown()


import rclpy
from crazychoir.controller import HierarchicalController, FlatnessAccelerationCtrl, GeometryAttitudeCtrl
from crazychoir.planner import AccelerationTraj
from crazychoir.crazyflie_communication import FPQRSender

def main():
    rclpy.init()

    frequency = 100
    position_ctrl = FlatnessAccelerationCtrl(frequency)
    attitude_ctrl = GeometryAttitudeCtrl(frequency)
    sender = FPQRSender(frequency)
    desired_acceleration = AccelerationTraj()

    controller = HierarchicalController(pos_handler='pubsub', pos_topic='odom', 
                position_strategy=position_ctrl, attitude_strategy=attitude_ctrl, 
                command_sender=sender, traj_handler=desired_acceleration)

    rclpy.spin(controller)
    rclpy.shutdown()

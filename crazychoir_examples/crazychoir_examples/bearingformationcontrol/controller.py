import rclpy
from crazychoir.controller import HierarchicalController, FlatnessAccelerationCtrl, GeometryAttitudeCtrl
from crazychoir.planner import AccelerationTraj
from crazychoir.crazyflie_communication import FPQRSender

def main():
    rclpy.init()

    vicon = False
    frequency = 100
    position_ctrl = FlatnessAccelerationCtrl(vicon, frequency)
    attitude_ctrl = GeometryAttitudeCtrl(vicon, frequency)
    sender = FPQRSender(frequency)
    desired_acceleration = AccelerationTraj()

    controller = HierarchicalController(pose_handler='pubsub', pose_topic='odom', 
                position_strategy=position_ctrl, attitude_strategy=attitude_ctrl, 
                command_sender=sender, traj_handler=desired_acceleration)

    rclpy.spin(controller)
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from crazychoir.controller import HierarchicalController, FlatnessAccelerationCtrl, GeometryAttitudeCtrl, FlatnessPositionCtrl
from crazychoir.planner import AccelerationTraj, FullStateTraj
from crazychoir.crazyflie_communication import FPQRSender, FRPYSender
from crazychoir.filtering import Estimator

def main():
    rclpy.init()

    controller_params = Node('controller_params', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    vicon_id = controller_params.get_parameter('vicon_id').value
    is_leader = controller_params.get_parameter('is_leader').value

    frequency = 100

    if is_leader:
        position_ctrl = FlatnessPositionCtrl(update_frequency= frequency, vicon= True)
        desired_acceleration = FullStateTraj()
    else:
        position_ctrl = FlatnessAccelerationCtrl(update_frequency=frequency, vicon=True)
        desired_acceleration = AccelerationTraj()    

    attitude_ctrl = GeometryAttitudeCtrl(update_frequency=frequency, vicon=True)
    sender = FPQRSender(frequency)

    callback = Estimator()
    controller = HierarchicalController(pose_handler='vicon', pose_topic='/vicon/cf{}/cf{}'.format(vicon_id,vicon_id), pose_callback=callback,
                position_strategy=position_ctrl, attitude_strategy=attitude_ctrl, 
                command_sender=sender, traj_handler=desired_acceleration)

    callback.set_pose(controller.current_pose) 

    controller.get_logger().info('Go!')

    rclpy.spin(controller)
    rclpy.shutdown()


import numpy as np
from typing import Callable
from scipy.constants import g
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist 
from choirbot.controller import Controller
from ..crazyflie_communication import SenderStrategy
from ..planner import TrajHandlerStrategy


class PositionCtrlStrategy:
    def __init__(self, update_frequency):
        self.update_frequency = update_frequency

    def control(self, agent_id, current_pose, desired_reference):
        raise NotImplementedError


class AttitudeCtrlStrategy:
    def __init__(self, update_frequency):
        self.update_frequency = update_frequency

    def control(self, agent_id, current_pose, desired_attitude, desired_reference):
        raise NotImplementedError


class CrazyflieController(Controller):
    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None,
                command_sender: SenderStrategy=None, traj_handler: TrajHandlerStrategy=None):
        
        super().__init__(pose_handler, pose_topic, pose_callback)
        
        self.mass = 0.038
        self.traj_handler = traj_handler

        if self.traj_handler is not None:
            self.subscriber = self.create_subscription(self.traj_handler.msg_type, self.traj_handler.topic, self.traj_handler.handler_func, 1)

        self.command_sender = command_sender
        if self.command_sender is None:
            raise ValueError("A command sender is required ")
        

class HierarchicalController(CrazyflieController):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None,
                position_strategy: PositionCtrlStrategy=None, attitude_strategy: AttitudeCtrlStrategy=None, 
                command_sender: SenderStrategy=None, traj_handler: TrajHandlerStrategy=None):
        
        super().__init__(pose_handler, pose_topic, pose_callback, command_sender, traj_handler)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        self.position_strategy = position_strategy
        self.attitude_strategy = attitude_strategy

        if self.position_strategy is None:
            raise ValueError("A position controller is required ")
        
        if self.attitude_strategy is not None and not np.isclose(self.attitude_strategy.update_frequency, self.command_sender.send_freq):
            raise ValueError("AttitudeCtrlStrategy.update_frequency and SenderStrategy.send_freq must be equal")

        if self.attitude_strategy is not None and np.isclose(self.position_strategy.update_frequency, self.attitude_strategy.update_frequency):
            self.ctrl_timer = self.create_timer(1.0/self.position_strategy.update_frequency, self.controller)
        else:
            self.position_ctrl_timer = self.create_timer(1.0/self.position_strategy.update_frequency, self.position_controller)
            self.attitude_send_timer = self.create_timer(1.0/self.command_sender.send_freq, self.attitude_send_controller)

        self.thrust = self.mass*g
        self.desired_attitude = R.from_euler('xyz', np.zeros(3))
        self.pqr = np.zeros(3)

        self.set_steady_reference = True
        self.desired_reference = {}

        self.desired_reference["position"] = np.zeros(3)
        self.desired_reference["velocity"] = np.zeros(3)
        self.desired_reference["acceleration"] = np.zeros(3)
        self.desired_reference["yaw"] = 0.0

    def controller(self):
        self.position_controller()
        self.attitude_send_controller()

    def position_controller(self):
        if self.current_pose.position is not None and self.current_pose.velocity is not None:
            if self.traj_handler is not None and bool(self.traj_handler.get_desired_reference()): # #NOTE and (there is a desired reference)
                self.desired_reference = self.traj_handler.get_desired_reference()
                self.set_steady_reference = True
            else:
                if self.set_steady_reference:
                    self.set_steady_reference = False
                    self.desired_reference["position"] = np.copy(self.current_pose.position)
                    self.desired_reference["velocity"] = np.zeros(3)
                    self.desired_reference["acceleration"] = np.zeros(3)
                    self.desired_reference["yaw"] = 0.0

            self.thrust, self.desired_attitude = self.position_strategy.control(self.current_pose, self.desired_reference)

    def attitude_send_controller(self):
        if self.current_pose.position is not None and self.current_pose.velocity is not None:
            if self.traj_handler is not None and bool(self.traj_handler.get_desired_reference()): # #NOTE and (there is a desired reference)
                self.desired_reference = self.traj_handler.get_desired_reference()
            if self.attitude_strategy is not None:
                self.pqr = self.attitude_strategy.control(self.current_pose, self.desired_attitude, self.desired_reference)

            crazyflie_input = self.command_sender.command_sender(self.thrust, self.desired_attitude, self.pqr)
            self.send_input(crazyflie_input)

    def send_input(self, u):
        msg = Twist()
        msg.linear.z  = u[0]
        msg.angular.x = u[1]
        msg.angular.y = u[2]
        msg.angular.z = u[3]
        self.publisher_.publish(msg)



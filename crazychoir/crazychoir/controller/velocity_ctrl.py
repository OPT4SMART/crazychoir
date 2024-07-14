from .hierarchical_control import CrazyflieController

from ..crazyflie_communication import SenderStrategy
from ..planner import TrajHandlerStrategy

from typing import Callable

import numpy as np

from geometry_msgs.msg import Twist 


class VelocityCtrlStrategy:
    def __init__(self, update_frequency):
        self.update_frequency = update_frequency
        self.K_p = np.array([  0.4,  0.4,  0.4])         
        self.K_v = np.array([  0.01,  0.01,  0.01]) 


    def control(self, current_pose, desired_reference):
        ep = desired_reference["position"] - current_pose.position
        if not "velocity" in desired_reference:
            desired_reference["velocity"] = np.zeros(3)
        ev = desired_reference["velocity"] - current_pose.velocity
        u_vel = self.K_p * ep + self.K_v * ev
        return u_vel

class VelocityLQRStrategy:
    pass

class VelocityController(CrazyflieController):
    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None,
                velocity_strategy: VelocityCtrlStrategy=None, 
                command_sender: SenderStrategy=None, traj_handler: TrajHandlerStrategy=None):
        
        super().__init__(pose_handler, pose_topic, pose_callback, command_sender, traj_handler)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        self.velocity_strategy = velocity_strategy

        if self.velocity_strategy is None:
            raise ValueError("A velocity controller is required ")

        self.ctrl_timer = self.create_timer(1.0/self.velocity_strategy.update_frequency, self.controller)

        self.reference_lost = True
        self.desired_reference = {}

        self.desired_reference["position"] = None
        self.desired_reference["velocity"] = None
        self.desired_reference["acceleration"] = None
        self.desired_reference["yaw"] = 0.0

    def controller(self):
        if self.current_pose.position is not None and self.current_pose.velocity is not None:
            if self.traj_handler is not None and bool(self.traj_handler.get_desired_reference()): # #NOTE and (there is a desired reference)
                self.desired_reference = self.traj_handler.get_desired_reference()
                self.reference_lost = True 
            else:
                if self.reference_lost and self.desired_reference["position"] is not None:
                    self.get_logger().warn('Reference lost - keeping hovering')
                    self.reference_lost = False
                    self.desired_reference["position"] = self.current_pose.position
                    self.desired_reference["velocity"] = np.zeros(3)

            if self.desired_reference["position"] is None:
                crazyflie_input = self.command_sender.command_sender(np.zeros(3))
                self.send_input(crazyflie_input)
                return

            u_vel = self.velocity_strategy.control(self.current_pose, self.desired_reference)
            crazyflie_input = self.command_sender.command_sender(u_vel)
            # print(f'u_vel= {crazyflie_input} | p_des= {self.desired_reference["position"]}')
            self.send_input(crazyflie_input)

        else:
            self.get_logger().warn('Pose not available - stopping the drone')
            crazyflie_input = self.command_sender.command_sender(np.zeros(3))
            self.send_input(crazyflie_input)

    
    def send_input(self, u):
        msg = Twist()
        msg.linear.x  = u[0]
        msg.linear.y  = u[1]
        msg.linear.z  = u[2]
        self.publisher_.publish(msg)
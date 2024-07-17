from geometry_msgs.msg import Vector3

from choirbot.guidance.aggregative import AggregativeGuidance
from choirbot.optimizer import AggregativeOptimizer
import numpy as np

from std_msgs.msg import Empty
from datetime import datetime
import dill
import os
import time

from typing import Callable



class AggregativeGuidanceCrazyflie(AggregativeGuidance):
    def __init__(self, optimizer: AggregativeOptimizer, height: float=1.0, freq_reference: float = 10.0, 
                 pose_handler: str = None, pose_topic: str = None, pose_callback: Callable=None,
                 msg_topic: str='position', msg_type=Vector3, seed: int=5):
        super().__init__(optimizer, freq_reference, pose_handler, pose_topic, pose_callback, msg_topic, msg_type)
        
        self.height = height
        self.seed = seed
        np.random.seed(self.seed)

        self.target = self.get_parameter_or('target').value
        if self.target is None:
            self.target = False
            self.get_logger().warn('Target not set. Using default value False')
        else:
            self.target = np.array(self.target).reshape(-1,1)
        
        self.intruder = self.get_parameter_or('intruder').value
        if self.intruder is None:
            self.intruder = False
            self.get_logger().warn('Intruder not set. Using default value False')
        else:
            self.intruder = np.array(self.intruder).reshape(-1,1)
  
        self.land_trigger_subscription = self.create_subscription(Empty, '/land', self.end_experiment, 10)

    def end_experiment(self, _):
        self.get_logger().info('End of the experiment. Landing...')
        self.save_results()
        self.optimization_ended = True

    def get_initial_condition(self):
        if self.current_pose.position is not None:
            return self.current_pose.position[:2].reshape(-1,1)
        else:
            raise ValueError('Pose callback not ready')        

    def get_intruder(self):
        if self.intruder is not False:
            return self.intruder[:2].reshape(-1,1)
        else:
            return False

    def get_target(self):
        if self.target is not False:
            return self.target[:2].reshape(-1,1)
        else:
            return False
        
    def save_results(self):
        print('save_results() not implemented yet :(')
    
    def generate_reference_msg(self, msg_type, x_des):

        # skip if not ready    
        if self.current_pose.position is None or self.optimizer.algorithm is None or self.optimization_ended:
            return
        
        # get reference
        x_des = self.optimizer.get_result()

        x = x_des[0,0]
        y = x_des[1,0]
        z = self.height

        msg = msg_type()

        msg.x    = x
        msg.y    = y
        msg.z    = z

        return msg
  
from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe

class AggregativeGuidanceCrazyflieMovingIntruder(AggregativeGuidanceCrazyflie):

    def __init__(self, optimizer: AggregativeOptimizer, height: float=1.0, freq_reference: float = 10.0, 
                 pose_handler: str = None, pose_topic: str = None, pose_callback: Callable=None,
                 msg_topic: str='position', msg_type=Vector3, seed: int=5):

        super().__init__(optimizer, height, freq_reference, pose_handler, pose_topic, pose_callback, msg_topic, msg_type, seed)

        self.intruder_pose = Pose(None, None,None, None)


        handler = 'pubsub'
        topic = f'/agent_{self.agent_id+self.n_agents}/odom'
        callback = None
        intruder_subscription = pose_subscribe(handler, topic, self, self.intruder_pose, callback)


    def get_intruder(self):
        if self.intruder_pose.position is not None:
            return self.intruder_pose.position[:2].reshape(-1,1)   
        else:
            self.get_logger().warn('Pose Intruder not available - returning False')
            return False


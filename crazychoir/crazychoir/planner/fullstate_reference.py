from crazychoir_interfaces.msg import FullState
import numpy as np
from .trajectory_strategy import TrajHandlerStrategy
from scipy.spatial.transform import Rotation as R

class FullStateTraj(TrajHandlerStrategy):
    def __init__(self, msg_type=FullState, topic='fullstate'):
        super().__init__(msg_type, topic)

    def handler_func(self, msg):
        self.des_ref["position"] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.des_ref["velocity"] = np.array([msg.vel.linear.x, msg.vel.linear.y, msg.vel.linear.z])
        self.des_ref["acceleration"] = np.array([msg.acc.linear.x, msg.acc.linear.y, msg.acc.linear.z])
        
        _, _, self.des_ref["yaw"] = R.from_quat(np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])).as_euler('xyz')
        # self.des_ref["yaw_rate"] = ...
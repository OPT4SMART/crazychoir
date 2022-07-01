from geometry_msgs.msg import Vector3
import numpy as np
from .trajectory_strategy import TrajHandlerStrategy

class AccelerationTraj(TrajHandlerStrategy):
    def __init__(self, msg_type=Vector3, topic='acceleration'):
        super().__init__(msg_type, topic)

    def handler_func(self, msg):
        self.des_ref["acceleration"] = np.array([msg.x, msg.y, msg.z])

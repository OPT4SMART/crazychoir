from geometry_msgs.msg import Vector3
import numpy as np
from .trajectory_strategy import TrajHandlerStrategy

class PositionTraj(TrajHandlerStrategy):
    def __init__(self, msg_type=Vector3, topic='position'):
        super().__init__(msg_type, topic)

    def handler_func(self, msg):
        self.des_ref["position"] = np.array([msg.x, msg.y, msg.z])
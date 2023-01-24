from .sender_strategy import SenderStrategy
import numpy as np
from scipy.spatial.transform import Rotation as R

class FRPYSender(SenderStrategy):
    def __init__(self, send_freq):
        super().__init__(send_freq)
    
    def command_sender(self, thrust, desired_attitude, pqr):
        rpy = R.from_matrix(desired_attitude).as_euler('xyz')
        u = np.append(thrust, rpy)
        return u


from .sender_strategy import SenderStrategy
import numpy as np

class FPQRSender(SenderStrategy):
    def __init__(self, send_freq):
        super().__init__(send_freq)
    
    def command_sender(self, thrust, desired_attitude, pqr):
        u = np.append(thrust, pqr)
        return u
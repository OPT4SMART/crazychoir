from .sender_strategy import SenderStrategy
import numpy as np

class VelSender(SenderStrategy):
    def __init__(self, send_freq):
        super().__init__(send_freq)
    
    def command_sender(self, vel_input):
        return vel_input
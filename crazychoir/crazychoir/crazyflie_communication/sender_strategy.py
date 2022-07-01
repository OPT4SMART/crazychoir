class SenderStrategy:
    def __init__(self, send_freq):
        self.send_freq = send_freq
    
    def command_sender(self, thrust, desired_attitude, pqr):
        raise NotImplementedError
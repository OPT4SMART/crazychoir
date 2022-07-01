class TrajHandlerStrategy:
    def __init__(self, msg_type, topic):
        self.msg_type = msg_type
        self.topic = topic
        self.des_ref = {}

    def handler_func(self, msg):
        raise NotImplementedError

    def get_desired_reference(self):
        return self.des_ref
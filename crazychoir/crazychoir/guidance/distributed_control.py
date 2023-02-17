from choirbot.guidance import Guidance
from geometry_msgs.msg import Vector3
from typing import Callable

class DistributedControl(Guidance):

    def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None, input_topic: str = 'acceleration'):
        super().__init__(pose_handler, pose_topic, pose_callback)
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.control)

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose, self.in_neighbors, self.out_neighbors, False)

        # compute input
        u = self.evaluate_input(data)

        # send input to planner/controller
        self.send_reference(u)


    def send_reference(self, ref):
        msg = Vector3()

        msg.x       = ref[0]
        msg.y       = ref[1]
        msg.z       = ref[2]
        
        self.publisher_.publish(msg)

    def evaluate_input(self, neigh_data):
        raise NotImplementedError


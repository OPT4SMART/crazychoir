from rclpy.node import Node

from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe
from crazychoir_interfaces.msg import FullState
from geometry_msgs.msg import Vector3
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint

from typing import Callable

class TrajectoryHandler(Node):
    """
    #TODO
    """

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None):
        """
        Args:
            pose_handler (str, optional): Pose handler (see
                :func:`~choirbot.utils.position_getter.pose_subscribe`). Defaults to None.
            pose_topic (str, optional): Topic where pose is published. Defaults to None.
        """
        super().__init__('trajectory', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.agent_id = self.get_parameter('agent_id').value

        # initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)

        # starting flags
        self.start_sender = False
        self.first_evaluation = False

        # initialize trajectory parameters subscription
        self.traj_params_topic = 'traj_params'
        self.create_subscription(Trajectory, self.traj_params_topic, self.traj_params_callback, 1)

    def traj_params_callback(self):
        raise NotImplementedError

    def handler(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        if self.start_sender:
            # compute reference
            ref = self.evaluate_reference()

            # send reference to controller
            self.send_reference(ref)

    def send_reference(self, ref):
        raise NotImplementedError

    def evaluate_reference(self):
        raise NotImplementedError


class AccelerationTrajHandler(TrajectoryHandler):
    '''
    #TODO
    '''

    def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None, input_topic: str = 'acceleration'):
        
        super().__init__(pose_handler, pose_topic, pose_callback)
        
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.handler)
        self.get_logger().info('Acceleration reference {} started'.format(self.agent_id))

    def send_reference(self, ref):
        msg = Vector3()

        msg.x       = ref[0]
        msg.y       = ref[1]
        msg.z       = ref[2]
        
        self.publisher_.publish(msg)


class FullStateTrajHandler(TrajectoryHandler):
    '''
    #TODO
    '''

    def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None, input_topic: str = 'fullstate'):
        
        super().__init__(pose_handler, pose_topic, pose_callback)
        
        self.publisher_ = self.create_publisher(FullState, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.handler)
        self.get_logger().info('FullState reference {} started'.format(self.agent_id))


    def send_reference(self, ref):
        msg = FullState()

        msg.pose.position.x     = ref[0]
        msg.pose.position.y     = ref[1]
        msg.pose.position.z     = ref[2]

        msg.pose.orientation.x  = ref[3]
        msg.pose.orientation.y  = ref[4]
        msg.pose.orientation.z  = ref[5]
        msg.pose.orientation.w  = ref[6]

        msg.vel.linear.x        = ref[7]
        msg.vel.linear.y        = ref[8]
        msg.vel.linear.z        = ref[9]

        msg.acc.linear.x        = ref[10]
        msg.acc.linear.y        = ref[11]
        msg.acc.linear.z        = ref[12]
        
        self.publisher_.publish(msg)
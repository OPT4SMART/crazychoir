from typing import Callable
import numpy as np

from rclpy.node import Node
from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe

from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint


class SimpleGuidance(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None, takeoff_time: float=5.0, height: float=1.0):

        super().__init__('simple_guidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.agent_id = self.get_parameter('agent_id').value
        self.init_pos = self.get_parameter('init_pos').value

        # initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)

        # button subscription
        self.takeoff_trigger_subscription = self.create_subscription(Empty, '/takeoff', self.takeoff, 10)
        self.land_trigger_subscription = self.create_subscription(Empty, '/land', self.landing, 10)

        # subscription to trajectory topic
        self.publishers_traj_params = self.create_publisher(Trajectory, 'traj_params', 1)

        # Set default height and takeoff time
        self.height = height
        self.takeoff_time = takeoff_time

        self.get_logger().info('Simple Guidance {} started'.format(self.agent_id))


    def takeoff(self, _):
        self.get_logger().info('Starting takeoff')

        msg = Trajectory()
        name = 'Agent_{}'.format(self.agent_id)

        point = TrajectoryPoint()
        x = self.init_pos[0]
        y = self.init_pos[1]
        z = self.height
        duration_s = self.takeoff_time
        point.positions = [x,y,z]
        point.velocities = [0.0,0.0,0.0]
        point.accelerations = [0.0,0.0,0.0]
        point.effort = [0.0,0.0,0.0]
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(0)

        msg.joint_names.append(name)
        msg.points.append(point)

        self.publishers_traj_params.publish(msg)

    def landing(self, _):
        self.get_logger().info('Starting landing')

        msg = Trajectory()
        name = 'Agent_{}'.format(self.agent_id)

        point = TrajectoryPoint()
        x = self.current_pose.position[0]
        y = self.current_pose.position[1]
        z = 0.01
        duration_s = self.takeoff_time
        point.positions = [x,y,z]
        point.velocities = [0.0,0.0,0.0]
        point.accelerations = [0.0,0.0,0.0]
        point.effort = [0.0,0.0,0.0]
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(0)

        msg.joint_names.append(name)
        msg.points.append(point)

        self.publishers_traj_params.publish(msg)
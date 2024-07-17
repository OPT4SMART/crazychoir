from typing import Callable
import numpy as np

from rclpy.node import Node
from choirbot import Pose
from choirbot.utils.position_getter import pose_subscribe

from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint

from time import sleep

class GoToGuidance(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None, takeoff_time: float=5.0):

        super().__init__('goto_guidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.agent_id = self.get_parameter('agent_id').value
        self.N = self.get_parameter('N').value
        self.init_pos = self.get_parameter('init_pos').value

        # list of goals: [time_0, x_0, y_0, z_0, time_1, x_1, y_1, z_1, ...]
        self.goals = self.get_parameter('goals').value 
        self.goals = [self.goals[i:i+4] for i in range(0, len(self.goals), 4)]

        # takeoff time
        self.takeoff_time = takeoff_time

        # initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)

        # button subscription
        self.experiment_trigger_subscription = self.create_subscription(Empty, '/experiment_trigger', self.execute_goal_list, 10)

        # subscription to trajectory topic
        self.publishers_traj_params = self.create_publisher(Trajectory, 'traj_params', 1)

        self.get_logger().info('GoToGuidance {} started'.format(self.agent_id))

    def execute_goal_list(self, _):
        self.get_logger().info('Starting goto')

        for goal in self.goals:
            goal_time = goal[0]
            goal_pos = goal[1:]
            self.goto(goal_time, goal_pos)
            print(f'agent {self.agent_id} going to {goal_pos}. Wait {goal[0]}')
            sleep(goal_time)

    def goto(self, goal_time, goal_pos):
        msg = Trajectory()
        name = 'Agent_{}'.format(self.agent_id)

        point = TrajectoryPoint()
        x = goal_pos[0]
        y = goal_pos[1]
        z = goal_pos[2]
        duration_s = goal_time
        point.positions = [x,y,z]
        point.velocities = [0.0,0.0,0.0]
        point.accelerations = [0.0,0.0,0.0]
        point.effort = [0.0,0.0,0.0]
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(0)

        msg.joint_names.append(name)
        msg.points.append(point)

        self.publishers_traj_params.publish(msg)
from choirbot.planner import PointToPointPlanner
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint


class PointToPointPlanner_2D(PointToPointPlanner):

    def __init__(self, goal_tolerance: float=0.05, pose_handler: str=None, pose_topic: str=None):
        super().__init__(goal_tolerance, pose_handler, pose_topic)
        
        self.publisher_traj_params = self.create_publisher(Trajectory, 'traj_params', 1)
        
        # Set setpoint height
        self.height = 0.5 + 0.15*self.agent_id   # meter

        # Set setpoint duration
        self.duration = 20  # sec

    def send_to_controller(self):

        if self.current_pose.position is not None:
            msg = Trajectory()
            name = 'Agent_{}'.format(self.agent_id)

            point = TrajectoryPoint()
            x = self.goal_point[0]
            y = self.goal_point[1]
            z = self.height
            point.positions = [x,y,z]
            point.velocities = [0.0,0.0,0.0]
            point.accelerations = [0.0,0.0,0.0]
            point.effort = [0.0,0.0,0.0]
            point.time_from_start.sec = int(self.duration)
            point.time_from_start.nanosec = int(0)

            # publish message
            self.get_logger().info('Goal {} sent'.format([x,y,z]))
            msg.joint_names.append(name)
            msg.points.append(point)
            self.publisher_traj_params.publish(msg)
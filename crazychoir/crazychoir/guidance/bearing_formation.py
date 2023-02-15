from typing import Callable
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint

from .distributed_control import DistributedControl

class BearingFormation(DistributedControl):
    """
    Formation Control 

    Implements a bearing-based formation control law for second order systems.
    """
    def __init__(self, update_frequency: float,
                 pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None, input_topic = 'acceleration'):
        super().__init__(update_frequency, pose_handler, pose_topic, pose_callback, input_topic)

        self.is_leader = self.get_parameter('is_leader').value
        self.agent_dim = self.get_parameter('dd').value
        self.ort_proj_array = self.get_parameter('ort_proj').value
        self.init_pos = self.get_parameter('init_pos').value
        self.vicon = self.get_parameter('vicon').value

        # reshape ortogonal projection Matrix
        self.ort_proj = np.reshape(self.ort_proj_array, ((self.agent_dim,self.n_agents*self.agent_dim)))

        if self.vicon:
            # Vicon Parameters
            self.bearing_prop_gain = np.array([3,3,10])
            self.bearing_deriv_gain = np.array([3,3,3])
            self.bearing_int_gain = np.array([0,0,1])

            self.prop_gain = np.array([7200, 7200, 30000])*1e-3
            self.deriv_gain = np.array([36, 36, 120])*1e-3
            self.int_gain = np.array([0, 0, 1000])*1e-3
        else:
            # Webots Parameters
            self.bearing_prop_gain = np.array([1,1,1])*0.1*2
            self.bearing_deriv_gain = np.array([1,1,1])*0.5*2
            self.bearing_int_gain = np.array([0,0,0])

            self.prop_gain = np.array([400, 400, 1250])*1e-3
            self.deriv_gain = np.array([200, 200, 400])*1e-3
            self.int_gain = np.array([0, 0, 0])*1e-3
            
        # Subscription to gui commands
        self.take_off_trigger_subscription = self.create_subscription(Empty, '/takeoff', self.start_take_off, 10)
        self.land_trigger_subscription = self.create_subscription(Empty, '/land', self.start_land, 10)
        self.experiment_trigger_subscription = self.create_subscription(Empty, '/experiment_trigger', self.start_experiment, 10)

        # Subscription to trajectory topic
        self.publishers_traj_params = self.create_publisher(Trajectory, 'traj_params', 1)

        # FSM Variables
        self.height = 1.0
        self.formation = False
        self.takeoff = False
        self.takeoff_started = False
        self.takeoff_pos = np.zeros(3)
        self.landing = False
        self.land_started = False
        self.landing_pos = np.zeros(3)

        # Integral action settings
        self.delta_int = 0.01 # 100MHz
        self.err_int_bearinig = np.zeros(3)
        self.err_int = np.zeros(3)


    def evaluate_input(self, neigh_data):
        u = np.zeros(3)

        if self.current_pose.position is not None and self.current_pose.velocity is not None:
            if self.takeoff:
                if self.takeoff_started:
                    if self.is_leader:
                        self.leaders_takeoff(self.height)

                    self.err_int = np.zeros(3)
                    self.takeoff_started = False
                    x_des, y_des, z_des = self.current_pose.position
                    self.takeoff_pos = np.array([x_des,y_des,z_des])

                if self.takeoff_pos[2] <= self.height:
                    self.takeoff_pos[2] += 1/self.update_frequency*0.2

                err_pos = self.current_pose.position - self.takeoff_pos
                err_vel = self.current_pose.velocity
                self.err_int += self.delta_int*err_pos

                # Sent only if is follower -> See self.send_input()
                u = - (self.prop_gain*err_pos + self.deriv_gain*err_vel + self.int_gain * self.err_int)

            elif self.landing:
                if self.land_started:
                    if self.is_leader:
                        self.leaders_landing()

                    self.land_started = False
                    x_des, y_des, z_des = self.current_pose.position
                    self.landing_pos = np.array([x_des,y_des,z_des])
            
                if self.landing_pos[2] > 0.01:
                    self.landing_pos[2] -= 1/self.update_frequency*0.2

                err_pos = self.current_pose.position - self.landing_pos
                err_vel = self.current_pose.velocity
                self.err_int += self.delta_int*err_pos
                
                # Sent only if is follower -> See self.send_input()
                u = - (self.prop_gain*err_pos + self.deriv_gain*err_vel + self.int_gain * self.err_int)

            elif self.formation:
                if not self.is_leader:
                    i = self.agent_id
                    dd = self.agent_dim
                    N = self.n_agents
                    for j, neigh_pose in neigh_data.items():
                        err_pos = self.current_pose.position - neigh_pose.position
                        err_vel = self.current_pose.velocity - neigh_pose.velocity
                        self.err_int_bearinig += self.delta_int*err_pos

                        P_ij = self.ort_proj[:,j*dd:(j+1)*dd]
                        u += - P_ij @ (self.bearing_prop_gain * err_pos + self.bearing_deriv_gain * err_vel + self.bearing_int_gain * self.err_int_bearinig)

        else:
            self.get_logger().warn('Pose or Velocity are None')

        return u


    def start_experiment(self, _):
        self.get_logger().info('Starting experiment')
        self.landing = False
        self.takeoff = False
        self.formation = True

    def start_take_off(self, _):
        self.get_logger().info('Starting Take-Off')
        self.landing = False
        self.formation = False
        self.takeoff = True
        self.takeoff_started = True

    def start_land(self, _):
        self.get_logger().info('Starting Landing')
        self.takeoff = False
        self.formation = False
        self.landing = True
        self.land_started = True

    def leaders_takeoff(self, z_des):
        msg = Trajectory()
        name = 'Agent_{}'.format(self.agent_id)

        point = TrajectoryPoint()
        x = self.init_pos[0]
        y = self.init_pos[1]
        z = z_des
        duration_s = 5
        point.positions = [x,y,z]
        point.velocities = [0.0,0.0,0.0]
        point.accelerations = [0.0,0.0,0.0]
        point.effort = [0.0,0.0,0.0]
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(0)

        msg.joint_names.append(name)
        msg.points.append(point)

        self.publishers_traj_params.publish(msg)

    def leaders_landing(self):
        msg = Trajectory()
        name = 'Agent_{}'.format(self.agent_id)

        point = TrajectoryPoint()
        x = self.current_pose.position[0]
        y = self.current_pose.position[1]
        z = 0.01
        duration_s = 5
        point.positions = [x,y,z]
        point.velocities = [0.0,0.0,0.0]
        point.accelerations = [0.0,0.0,0.0]
        point.effort = [0.0,0.0,0.0]
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int(0)

        msg.joint_names.append(name)
        msg.points.append(point)

        self.publishers_traj_params.publish(msg)

    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]
        if not self.is_leader:
            self.publisher_.publish(msg)
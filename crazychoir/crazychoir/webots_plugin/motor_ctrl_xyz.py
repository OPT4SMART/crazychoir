import rclpy
from rclpy.time import Time
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint

from math import degrees
from scipy.spatial.transform import Rotation as R
import numpy as np

from .motor_ctrl import MotorCtrl
from ..utils import cffirmware


class MotorCtrlXYZ(MotorCtrl):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        print('MotorCrtlXYZ Started!')

        # Declare Subscriptions
        msg_type = Trajectory
        self.cf_driver.create_subscription(msg_type, '/{}/traj_params'.format(self.namespace), self.setpoint_callback, 10)
        self.odom_publisher = self.cf_driver.create_publisher(Odometry, '/{}/odom'.format(self.namespace), 10)
        self.stop_subscription = self.cf_driver.create_subscription(Empty, '/stop', self.stop, 10)

        # Initialize cf classes
        self.cf_planner = cffirmware.planner()
        cffirmware.plan_init(self.cf_planner)
        cffirmware.controllerPidInit()

        self.setpoint_received = False

    def setpoint_callback(self, msg):
        self.setpoint_received = True
        setpoint = msg.points[-1]
        position = cffirmware.mkvec(setpoint.positions[0],setpoint.positions[1],setpoint.positions[2])
        yaw = float(0.0)
        duration = float(setpoint.time_from_start.sec + setpoint.time_from_start.nanosec*1e-9)
        self.setpoint_start_time = float(self.robot.getTime())
        relative = False
        current_eval = cffirmware.traj_eval_zero()
        current_eval.pos = cffirmware.mkvec(self.current_pose.position[0],self.current_pose.position[1],self.current_pose.position[2])

        string = "Received setpoint: curr_pos = {}\tpos = {}\tyaw = {}\tduration = {}\tinit = {}".format(current_eval.pos,position, yaw, duration, self.setpoint_start_time)
        # print(string)
        #  planner *p, const struct traj_eval *curr_eval, bool relative, struct vec take_off_pos, float take_off_yaw, float duration, float t)
        cffirmware.plan_go_to_from(self.cf_planner, current_eval, relative, position, yaw, duration, self.setpoint_start_time)

    
    def step(self):

        rclpy.spin_once(self.cf_driver, timeout_sec=0)
        self.time = self.robot.getTime()

        self.update_current_pose()
        self.publish_odometry()
        self.update_cf_state()
        self.update_cf_sensors()
        self.check_safety_area()

        if self.initialization:
            self.initialization = False
            self.init_setpoint()

        if self.setpoint_received:
            self.compute_setpoint()
            if cffirmware.plan_is_finished(self.cf_planner,self.time):
                self.setpoint_received = False
            string = "[{:.4f}]\t setpoint.pos = x:{:.4f} y:{:.4f} z:{:.4f}".format(self.time, self.setpoint.position.x,self.setpoint.position.y,self.setpoint.position.z)
            # print(string)


        ## Firmware PID bindings
        tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
        cffirmware.controllerPid(self.control, self.setpoint, self.sensors, self.state, tick)

        ## 
        cmd_roll    =  self.control.roll  # rad/s 
        cmd_pitch   =  self.control.pitch # rad/s
        cmd_yaw     = -self.control.yaw   # rad/s
        cmd_thrust  =  self.control.thrust         # uint (PWM)

        if self.emergency_stop:
            cmd_roll    = 0.0
            cmd_pitch   = 0.0
            cmd_yaw     = 0.0
            cmd_thrust  = 0.0
                   
        string = "control  -> cmd t:{:.0f}\tr: {:.4f}\tp: {:.4f}\ty: {:.4f}\n".format(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw)
        # print(string)
        
        self.send_motor_cmd(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw)

        self.past_time = self.robot.getTime()
        self.past_position = self.current_pose.position

 
    def init_setpoint(self):

        # Initialize setpoint modes
        self.setpoint.mode.x = cffirmware.modeAbs
        self.setpoint.mode.y = cffirmware.modeAbs
        self.setpoint.mode.z = cffirmware.modeAbs
        self.setpoint.mode.roll = cffirmware.modeDisable
        self.setpoint.mode.pitch = cffirmware.modeDisable
        self.setpoint.mode.yaw = cffirmware.modeAbs
        self.setpoint.mode.quat = cffirmware.modeDisable

        # Initialize setpoint references
        # NOTE: minus @ pitch
        self.setpoint.attitude.roll  = 0.0
        self.setpoint.attitude.pitch = -0.0 # WARNING: This needs to be negated
        self.setpoint.attitude.yaw   = 0.0

        self.setpoint.position.x = self.current_pose.position[0]
        self.setpoint.position.y = self.current_pose.position[1]
        self.setpoint.position.z = self.current_pose.position[2]

        self.setpoint.velocity.x = 0.0
        self.setpoint.velocity.y = 0.0
        self.setpoint.velocity.z = 0.0

        self.setpoint.acceleration.x = 0.0
        self.setpoint.acceleration.y = 0.0
        self.setpoint.acceleration.z = 0.0

    def compute_setpoint(self):
        planner_setpoint = cffirmware.plan_current_goal(self.cf_planner, self.time)

        self.setpoint.position.x = planner_setpoint.pos.x
        self.setpoint.position.y = planner_setpoint.pos.y
        self.setpoint.position.z = planner_setpoint.pos.z

        self.setpoint.velocity.x = planner_setpoint.vel.x
        self.setpoint.velocity.y = planner_setpoint.vel.y
        self.setpoint.velocity.z = planner_setpoint.vel.z

        self.setpoint.attitude.yaw = degrees(planner_setpoint.yaw)

        self.setpoint.attitudeRate.roll = degrees(planner_setpoint.omega.x)
        self.setpoint.attitudeRate.pitch = degrees(planner_setpoint.omega.y)
        self.setpoint.attitudeRate.yaw = degrees(planner_setpoint.omega.z)

        self.setpoint.acceleration.x = planner_setpoint.acc.x
        self.setpoint.acceleration.y = planner_setpoint.acc.y
        self.setpoint.acceleration.z = planner_setpoint.acc.z


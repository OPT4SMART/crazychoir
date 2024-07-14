import rclpy
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
from math import degrees, radians

from .motor_ctrl import MotorCtrl
from ..utils import cffirmware

class MotorCtrlVel(MotorCtrl):

    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        print('MotorCrtlFPQR Started!')

        # Declare Subscriptions
        msg_type = Twist()
        self.target_cmd_vel = msg_type
        self.cf_driver.create_subscription(msg_type, '/{}/cmd_vel'.format(self.namespace), self.setpoint_callback, 1)
        self.odom_publisher = self.cf_driver.create_publisher(Odometry, '/{}/odom'.format(self.namespace), 10)
        self.stop_subscription = self.cf_driver.create_subscription(Empty, '/stop', self.stop, 10)
        
        # Initialize cf classes
        cffirmware.controllerPidInit()
    
    def setpoint_callback(self, twist):
        self.target_cmd_vel = twist

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

        self.compute_setpoint()

        ## Firmware PID bindings
        tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
        string = "vx = {:.4f}\tvy = {:.4f}\tvz = {:.4f}\tYaw = {:.4f}".format(self.setpoint.velocity.x, self.setpoint.velocity.y, self.setpoint.velocity.z, self.setpoint.attitudeRate.yaw)
        # print(string)

        cffirmware.controllerPid(self.control, self.setpoint, self.sensors, self.state, tick)
        

        ## 
        cmd_roll    =  radians(self.control.roll)  # rad/s 
        cmd_pitch   =  radians(self.control.pitch) # rad/s
        cmd_yaw     = -radians(self.control.yaw)  # rad/s
        cmd_thrust  = self.control.thrust         # uint (PWM)

        if self.emergency_stop:
            cmd_roll    = 0.0
            cmd_pitch   = 0.0
            cmd_yaw     = 0.0
            cmd_thrust  = 0.0

        string = "Thrust = {:.0f}\tRoll = {:.4f}\tPitch = {:.4f}\tYaw = {:.4f}".format(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw)
        # print(string)
        
        self.send_motor_cmd(cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw)

        self.past_time = self.robot.getTime()
        self.past_position = self.current_pose.position


    def init_setpoint(self):
        #from modules/src/crtl_commander_generic.c VelocityDecoder

        # Initialize setpoint modes
        self.setpoint.mode.x  = cffirmware.modeVelocity
        self.setpoint.mode.y = cffirmware.modeVelocity
        self.setpoint.mode.z   = cffirmware.modeVelocity
        self.setpoint.mode.yaw     = cffirmware.modeVelocity  
        self.setpoint.velocity_body = False

    def compute_setpoint(self):

        self.setpoint.velocity.x  = self.target_cmd_vel.linear.x    #m/s
        # NOTE: minus @ pitch
        self.setpoint.velocity.y = self.target_cmd_vel.linear.y    #m/s
        self.setpoint.velocity.z   = self.target_cmd_vel.linear.z    #m/s
        self.setpoint.attitudeRate.yaw = self.target_cmd_vel.angular.z   # deg/s


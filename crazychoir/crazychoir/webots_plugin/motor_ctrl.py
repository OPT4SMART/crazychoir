import rclpy
from rclpy.time import Time
from nav_msgs.msg import Odometry

from math import degrees
from scipy.spatial.transform import Rotation as R
import numpy as np

from choirbot import Pose

from ..utils import cffirmware

class MotorCtrl:
    def init(self, webots_node, properties):
        
        # Declare the robot name and fix the timestep
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_name = self.robot.getName()

        # Initialize webots driver node
        rclpy.init(args=None)
        self.namespace = str(self.robot_name)
        self.cf_driver = rclpy.create_node(
                            'cf_driver',
                            namespace=self.namespace,
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)

        ## Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)
        
        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        ## Intialize Variables
        self.current_pose = Pose(None, None, None, None)
        self.past_position = None
        self.past_time = self.robot.getTime()

        # Initialize cf classes
        self.state = cffirmware.state_t()
        self.sensors = cffirmware.sensorData_t()
        self.setpoint = cffirmware.setpoint_t()
        self.control = cffirmware.control_t()

        # Initialize flags
        self.initialization = True
        self.emergency_stop = False

        # Safe zone limits
        self.safezone_limits = [np.infty,np.infty,np.infty] # [x,y,z] in meters

    def step(self):
        raise NotImplementedError

    def init_setpoint(self):
        raise NotImplementedError

    def setpoint_callback(self):
        raise NotImplementedError

    def compute_setpoint(self):
        raise NotImplementedError

    def stop(self,_):
        self.emergency_stop = True

    def check_safety_area(self):
        safezone_ck = [np.abs(self.current_pose.position.item(i)) > self.safezone_limits[i] for i in range(3)]
        if any(safezone_ck):
            self.emergency_stop = True
            self.cf_driver.get_logger().warn('{} KILLED: OUT OF SAFE ZONE'.format(self.robot_name))        
        

    def update_current_pose(self):
        ## Get measurements
        self.current_rpy = np.array(self.imu.getRollPitchYaw())
        self.current_pose.orientation = np.array(R.from_euler('xyz', self.current_rpy).as_quat())
        self.current_pose.angular = np.array(self.gyro.getValues())
        self.current_pose.position = np.array(self.gps.getValues())

        # TODO use velocity_low_pass
        dt = self.robot.getTime() - self.past_time
        if self.past_position is None:
            self.past_position = np.copy(self.current_pose.position)
        self.current_pose.velocity = (self.current_pose.position - self.past_position)/dt

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.current_pose.position[0]
        odom.pose.pose.position.y = self.current_pose.position[1]
        odom.pose.pose.position.z = self.current_pose.position[2]

        odom.twist.twist.linear.x = self.current_pose.velocity[0]
        odom.twist.twist.linear.y = self.current_pose.velocity[1]
        odom.twist.twist.linear.z = self.current_pose.velocity[2]

        odom.pose.pose.orientation.x = self.current_pose.orientation[0]
        odom.pose.pose.orientation.y = self.current_pose.orientation[1]
        odom.pose.pose.orientation.z = self.current_pose.orientation[2]
        odom.pose.pose.orientation.w = self.current_pose.orientation[3]

        odom.twist.twist.angular.x = self.current_pose.angular[0]
        odom.twist.twist.angular.y = self.current_pose.angular[1]
        odom.twist.twist.angular.z = self.current_pose.angular[2]
        self.odom_publisher.publish(odom)

    def update_cf_state(self):
        self.state.attitude.roll     = degrees(self.current_rpy[0])
        self.state.attitude.pitch    = -degrees(self.current_rpy[1])
        self.state.attitude.yaw      = degrees(self.current_rpy[2])
        self.state.position.x        = self.current_pose.position[0]
        self.state.position.y        = self.current_pose.position[1]
        self.state.position.z        = self.current_pose.position[2]
        self.state.velocity.x        = self.current_pose.velocity[0]
        self.state.velocity.y        = self.current_pose.velocity[1]
        self.state.velocity.z        = self.current_pose.velocity[2]

        string = "state    -> pos x: {:.4f}\ty: {:.4f}\tz: {:.4f}\n            vel x: {:.4f}\ty: {:.4f}\tz: {:.4f}".format(self.state.position.x,
                                                                                                self.state.position.y,
                                                                                                self.state.position.z,
                                                                                                self.state.velocity.x,
                                                                                                self.state.velocity.y,
                                                                                                self.state.velocity.z,
                                                                                                )
        # print(string)

    def update_cf_sensors(self):
        self.sensors.gyro.x = degrees(self.current_pose.angular[0])
        self.sensors.gyro.y = degrees(self.current_pose.angular[1])
        self.sensors.gyro.z = degrees(self.current_pose.angular[2])

    def send_motor_cmd(self, cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw):
        ## Motor mixing
        # Power distribution from ~/crazyflie-firmware/src/modules/src/power_distribution_quadrotor.c

        motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw

        scaling = 800 #700 # 1000 ##Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)

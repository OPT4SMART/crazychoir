import time
from math import degrees
import numpy as np
from .radio_handler import RadioHandler

class RadioHandlerFRPY(RadioHandler):

    def __init__(self):
        super().__init__()
        
        print('Radio FRPY Started!')

        # Conversion factors
        self.mg_newton = 0.39
        self.mg_pwm = 42000
        self.newton2pwm = self.mg_pwm/self.mg_newton

        # cmd limits
        self.thrust_lim_max = 60000
        self.thrust_lim_min = 10001
        self.rp_lim = 720
        self.y_lim  = 450

        # takeoff permission
        self.takeoff = False

        
    def cmd_sender(self, msg, uri, cf):

        # TODO: do something during cmd_vel initialization -> cf needs a cmd every 1s

        # Check safety area
        self.check_safety_area()


        cmd_thrust  = np.clip(int(msg.linear.z*self.newton2pwm), self.thrust_lim_min, self.thrust_lim_max) # uint
        cmd_roll    = np.clip(degrees( msg.angular.x)          , -self.rp_lim, self.rp_lim) #deg/s
        cmd_pitch   = np.clip(degrees( msg.angular.y)          , -self.rp_lim, self.rp_lim) #no minus
        cmd_yaw     = np.clip(degrees(-msg.angular.z)          , -self.y_lim , self.y_lim)

        if self.emergency_stop or not self.takeoff:
            cmd_thrust  = 0
            cmd_roll    = 0.0
            cmd_pitch   = 0.0
            cmd_yaw     = 0.0

        # string = "Commands: p={:.4f} q={:.4f} r={:.4f} thrust = {}".format(cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust)
        # print(string)        

        cf.commander.send_setpoint(cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust)

    def cmd_stop(self, _):
        self.emergency_stop = True

    def cmd_take_off(self, _):
        self.takeoff = True

    def set_values(self,uri, cf):
        # cf.param.set_value('commander.enHighLevel', 0)                                      -> default 0
        # cf.param.set_value('stabilizer.controller', 1)      # PID(1), Mellinger(2)          -> default 1
        # cf.param.set_value('stabilizer.estimator', 1)       # Complementary(1), Kalman(2)   -> default 1
        # cf.param.set_value('flightmode.yawMode',2)          # plus_mode(1), x_mode(2)       -> default 1
        # cf.param.set_value('flightmode.stabModeRoll',1)     # rate(0) angle(1)              -> default 1
        # cf.param.set_value('flightmode.stabModePitch', 1)   # rate(0) angle(1)              -> default 1
        cf.param.set_value('flightmode.stabModeYaw', 1)     # rate(0) angle(1)              -> default 0

        print("[{}] \t  cf{} - parameters setted!".format(time.time(), int(uri[-1])))        

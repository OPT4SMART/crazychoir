import time
from math import degrees
import numpy as np
from .radio_handler import RadioHandler

class RadioHandlerVel(RadioHandler):

    def __init__(self):
        super().__init__()
        
        print('Radio Vel Started!')

        # cmd limits
        self.vel_lim_max = 1.0
        self.vel_lim_min = -1.0

        # takeoff permission
        self.takeoff = False

    def cmd_sender(self, msg, uri, cf):

        # TODO: do something during cmd_vel initialization -> cf needs a cmd every 1s

        # Check safety area
        self.check_safety_area()
        cmd_vx = np.clip(msg.linear.x, self.vel_lim_min, self.vel_lim_max)
        cmd_vy = np.clip(msg.linear.y, self.vel_lim_min, self.vel_lim_max)
        cmd_vz = np.clip(msg.linear.z, self.vel_lim_min, self.vel_lim_max)
        cmd_yawrate = 0.0

        if self.emergency_stop or np.linalg.norm([cmd_vx, cmd_vy, cmd_vz]) < 1e-3: # or not self.takeoff:
            return


        string = "Commands: vx={:.4f} vy={:.4f} vz={:.4f}".format(cmd_vx, cmd_vy, cmd_vz)
        print(string)        

        cf.commander.send_velocity_world_setpoint(cmd_vx, cmd_vy, cmd_vz, cmd_yawrate)

    # def cmd_stop(self, _):
    #     self.emergency_stop = True

    def cmd_stop(self, _):
        self.emergency_stop = True
        for uri, scf in self.swarm._cfs.items():
            cf = scf.cf
            cf.commander.send_stop_setpoint()
            print("[{}] \t  cf{} - STOP!".format(time.time(), int(uri[-1])))  

    def cmd_take_off(self, _):
        self.takeoff = True


    def set_values(self,uri, cf):
        # cf.param.set_value('commander.enHighLevel', 0)                                      -> default 0
        # cf.param.set_value('stabilizer.controller', 1)      # PID(1), Mellinger(2)          -> default 1
        cf.param.set_value('stabilizer.estimator', 2)       # Complementary(1), Kalman(2)   -> default 1
        # cf.param.set_value('flightmode.yawMode',2)          # plus_mode(1), x_mode(2)       -> default 1
        # cf.param.set_value('flightmode.stabModeRoll',0)     # rate(0) angle(1)              -> default 1
        # cf.param.set_value('flightmode.stabModePitch', 0)   # rate(0) angle(1)              -> default 1
        # cf.param.set_value('flightmode.stabModeYaw', 0)     # rate(0) angle(1)              -> default 0

        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        cf.param.set_value('locSrv.extQuatStdDev', 0.06)

        print("[{}] \t  cf{} - parameters set!".format(time.time(), int(uri[-1])))  

        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

        print("[{}] \t  cf{} - kalman filter reset done!".format(time.time(), int(uri[-1])))   
 

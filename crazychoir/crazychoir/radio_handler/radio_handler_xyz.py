import time
from .radio_handler import RadioHandler

class RadioHandlerXYZ(RadioHandler):

    def __init__(self):
        super().__init__()        
        print('Radio XYZ Started!')

    def cmd_sender(self, msg, uri, cf):

        if self.emergency_stop:
            print("[{}] \t  cf{} - STOP PUSHED".format(time.time(), int(uri[-1])))  
            return

        # TODO: do something during cmd_vel initialization -> cf needs a cmd every 1s
        
        setpoint = msg.points[-1]
        x = setpoint.positions[0]
        y = setpoint.positions[1]
        z = setpoint.positions[2]
        yaw = 0.0
        duration = float(setpoint.time_from_start.sec + setpoint.time_from_start.nanosec*1e-9)
        relative = False

        # string = "[{}] \t  cf{} goto: x={:.4f} y={:.4f} z={:.4f} yaw = {:.4f}".format(time.time(), int(uri[-1]), x, y, z, yaw)
        # print(string)        
        cf.high_level_commander.go_to(x, y, z, yaw, duration, relative)

    def cmd_take_off(self, _):
        pass

    def cmd_stop(self, _):
        self.emergency_stop = True
        for uri, scf in self.swarm._cfs.items():
            cf = scf.cf
            cf.high_level_commander.stop()
            print("[{}] \t  cf{} - STOP!".format(time.time(), int(uri[-1])))  


    def set_values(self,uri, cf):
        cf.param.set_value('commander.enHighLevel', 1)      #                               -> default 0
        # cf.param.set_value('stabilizer.controller', 1)      # PID(1), Mellinger(2)          -> default 1
        cf.param.set_value('stabilizer.estimator', 2)       # Complementary(1), Kalman(2)   -> default 1
        # cf.param.set_value('flightmode.yawMode',2)          # plus_mode(1), x_mode(2)       -> default 1
        # cf.param.set_value('flightmode.stabModeRoll',0)     # rate(0) angle(1)              -> default 1
        # cf.param.set_value('flightmode.stabModePitch', 0)   # rate(0) angle(1)              -> default 1
        # cf.param.set_value('flightmode.stabModeYaw', 0)     # rate(0) angle(1)              -> default 0

        # Set the std deviation for the quaternion data pushed into the
        # kalman filter. The default value seems to be a bit too low.
        cf.param.set_value('locSrv.extQuatStdDev', 0.06)

        print("[{}] \t  cf{} - parameters setted!".format(time.time(), int(uri[-1])))  

        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

        print("[{}] \t  cf{} - kalman filter reset done!".format(time.time(), int(uri[-1])))  


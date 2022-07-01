import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.constants import g
from .hierarchical_control import AttitudeCtrlStrategy

class GeometryAttitudeCtrl(AttitudeCtrlStrategy):

    def __init__(self, update_time):
        super().__init__(update_time)

        K_r_z = 10**(-4)
        K_w_z = 10**(-4)
        K_r_x = 80*10**(-4)
        K_w_x = 8*10**(-4)
        self.K_r = 150*np.array([K_r_x, K_r_x, K_r_z])
        self.K_w = 100*np.array([K_w_x, K_w_x, K_w_z])


    def control(self, current_pose, desired_attitude, desired_reference):
        RR = R.from_quat(current_pose.orientation).as_matrix()
        error = 0.5*(np.dot(desired_attitude.transpose(),RR) - (np.dot(RR.transpose(),desired_attitude)))
        e_r = np.array([error[2,1], error[0,2], error[1,0]])
        # dumping angular velocity
        e_w = current_pose.angular
        # angular velocity inputs
        pqr = - self.K_r*e_r #- self.K_w*e_w
        return pqr


import numpy as np
from numpy import cos, sin
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as R
from scipy.constants import g
from .hierarchical_control import PositionCtrlStrategy

class FlatnessPositionCtrl(PositionCtrlStrategy):

    def __init__(self, update_time):
        super().__init__(update_time)

        self.K_p = 5*np.array([0.2, 0.2, 1.0])
        self.K_v = 5*np.array([0.3, 0.3, 0.8])
        
        self.mass = 0.027
        

    def control(self, current_pose, desired_reference):
        # implement flatness based control scheme
        # compute desired thrust vector
        F_des = self.thrust_dir(current_pose, desired_reference)
        # compute thrust
        RR = R.from_quat(current_pose.orientation).as_matrix()
        e_3 = np.array([0,0,1])
        thrust = np.dot(np.dot(RR,F_des),e_3)

        # compute desired attitude
        z_b_des = F_des/norm(F_des)
        x_c_des = np.array([cos(desired_reference["yaw"]),sin(desired_reference["yaw"]),0]).transpose()
        y_b_des = np.cross(z_b_des, x_c_des)/norm(np.cross(z_b_des, x_c_des))
        x_b_des = np.cross(y_b_des,z_b_des)
        R_des = np.array([x_b_des, y_b_des, z_b_des]).transpose()

        return thrust, R_des

    def thrust_dir(self, current_pose, desired_reference):
        # compute desired thrust vector
        e_3 = np.array([0,0,1])
        e_p =  current_pose.position - desired_reference["position"]
        e_v =  current_pose.velocity - desired_reference["velocity"]
        F_des = self.mass*(g*e_3 - desired_reference["acceleration"] + self.K_p*e_p + self.K_v*e_v)
        return F_des


class FlatnessAccelerationCtrl(FlatnessPositionCtrl):

    def __init__(self, update_time):
        super().__init__(update_time)

    def thrust_dir(self, current_pose, desired_reference):
        # compute desired thrust vector
        e_3 = np.array([0,0,1])
        F_des = self.mass*(g*e_3 + desired_reference["acceleration"])
        return F_des


import numpy as np
from scipy.spatial.transform import Rotation as R
from .hierarchical_control import AttitudeCtrlStrategy
from numpy import cos, sin


class GeometryAttitudeCtrl(AttitudeCtrlStrategy):

    def __init__(self, vicon: False, update_time):
        super().__init__(update_time)

        # Vicon flag
        self.vicon = vicon

        if self.vicon:
            # Vicon parameters
            self.K_r = np.array([ 12000, 12000, 8000])*1e-3
            self.K_w = np.array([ 100, 100, 80])*1e-3
        else:
            # Webots parameters
            self.K_r = np.array([16000, 16000, 200])*1e-3
            self.K_w = np.array([ 560,  560,  70])*1e-3

    def control(self, current_pose, desired_attitude, desired_reference):
        RR = R.from_quat(current_pose.orientation).as_matrix()
        error = 0.5*(np.dot(desired_attitude.transpose(),RR) - (np.dot(RR.transpose(),desired_attitude)))
        e_r = np.array([error[2,1], error[0,2], error[1,0]])
        # dumping angular velocity
        e_w = current_pose.angular
        # angular velocity inputs
        omega = - self.K_r*e_r - self.K_w*e_w
        # angular rates
        pqr = np.dot(self.change_matrix(current_pose), omega)
        return pqr

    def change_matrix(self, current_pose):
        # convert angular velocities to euler rates
        angles = R.from_quat(current_pose.orientation).as_euler('xyz')
        cphi = cos(angles[0])
        ctheta = cos(angles[1])
        sphi = sin(angles[0])
        stheta = sin(angles[1])
        ttheta = stheta/ctheta
        Rot = np.array([[1,sphi*ttheta,cphi*ttheta],[0,cphi,-stheta],[0,sphi/ctheta,cphi/ctheta]])
        return Rot




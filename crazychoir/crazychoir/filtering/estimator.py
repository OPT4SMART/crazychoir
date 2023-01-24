import numpy as np
from numpy import cos, sin
from scipy.spatial.transform import Rotation as R
from typing import Callable

class Estimator(Callable):

    def __init__(self):
        # self.counter = 0
        self.filter_order = 3
        self.poses = {
            'position'              : [np.zeros(3) for i in range(self.filter_order + 1)], # +1 due to current pose
            'pre_filtered_rpy'      : [np.zeros(3) for i in range(self.filter_order + 1)],
            'rpy'                   : [np.zeros(3) for i in range(self.filter_order + 1)],
            'pre_filtered_velocity' : [np.zeros(3) for i in range(self.filter_order + 1)],
            'velocity'              : [np.zeros(3) for i in range(self.filter_order + 1)],
            'pre_filtered_rpy_dot'  : [np.zeros(3) for i in range(self.filter_order + 1)],
            'rpy_dot'               : [np.zeros(3) for i in range(self.filter_order + 1)],
        }

    def set_pose(self, pose):
        self.current_pose = pose

    def __call__(self):

        if self.current_pose.orientation is not None:
            self.poses["rpy"].pop(0)

            if np.linalg.norm(self.current_pose.orientation) < 0.8:
                # self.get_logger().warn('[cf{}] -- Vicon sample lost'.format(self.agent_id))

                print('WARN: Vicon sample lost')

                self.current_pose.position = np.copy(self.poses["position"][-1])
                self.current_pose.orientation = R.from_euler('xyz',self.poses["rpy"][-1]).as_quat()

            self.poses["rpy"].append(R.from_quat(self.current_pose.orientation).as_euler('xyz'))
            self.current_pose.angular = self.angular_low_pass()        

        if self.current_pose.position is not None:
            self.poses["position"].pop(0)

            self.poses["position"].append(self.current_pose.position)
            self.current_pose.velocity = self.velocity_low_pass()


        return self.current_pose

    def velocity_low_pass(self):

        gain = 200.0
        zeros = [ 1.0, 0.0, 0.0]
        poles = [-1.0, 0.0, 0.0]

        low_pass_3rd_order(gain, zeros, poles, self.poses['position'], self.poses['pre_filtered_velocity'])

        gain = 0.2
        zeros = [-1.0, 0.0, 0.0]
        poles = [ 0.6, 0.0, 0.0]   

        low_pass_3rd_order(gain, zeros, poles, self.poses['pre_filtered_velocity'],self.poses['velocity'])
        return self.poses['velocity'][-1]

    def angular_low_pass(self):

        gain = 200.0
        zeros = [ 1.0, 0.0, 0.0]
        poles = [-1.0, 0.0, 0.0]

        low_pass_3rd_order(gain, zeros, poles, self.poses['rpy'], self.poses['pre_filtered_rpy_dot'])

        gain = 0.2
        zeros = [-1.0, 0.0, 0.0]
        poles = [ 0.6, 0.0, 0.0]   

        low_pass_3rd_order(gain, zeros, poles, self.poses['pre_filtered_rpy_dot'],self.poses['rpy_dot'])

        return self.poses['rpy_dot'][-1]


def low_pass_3rd_order(gain, zeros, poles, input, output):

    if len(input)<4 or len(output)<4:
        raise ValueError("Low-pass filter input needs at least 3 past samples pus the current one")

    N = np.array([
            -(zeros[0]*zeros[1]*zeros[2]),
              zeros[0]*zeros[1] + zeros[1]*zeros[2] + zeros[0]*zeros[2],
            -(zeros[0] + zeros[1] + zeros[2]),
              1.0,
        ]).reshape(1,4)

    D = np.array([
              0.0,
              poles[0]*poles[1]*poles[2],
            -(poles[0]*poles[1] + poles[1]*poles[2] + poles[0]*poles[2]),
              poles[0] + poles[1] + poles[2],
        ]).reshape(1,4)

    output.append((D@np.array(output) + gain*N@np.array(input)).reshape(3,))
    output.pop(0)

def change_matrix(rpy):
    # convert angular velocities to euler rates
    cphi = cos(rpy[0])
    ctheta = cos(rpy[1])
    sphi = sin(rpy[0])
    stheta = sin(rpy[1])
    ttheta = stheta/ctheta
    R = np.array([[1,sphi*ttheta,cphi*ttheta],[0,cphi,-stheta],[0,sphi/ctheta,cphi/ctheta]])
    return R


import numpy as np
from .trajectory_handler import FullStateTrajHandler
from scipy.spatial.transform import Rotation as R
from numpy.polynomial import Polynomial as poly


class Polynomial7th(FullStateTrajHandler):
    """
    From "Trajectory Planning for Automatic Machines and Robots" - L. Biagiotti, C. Melchiorri
    
    Compute a point-to-point trajectory reference based on a Polynomials of degree seven
    """
    def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, pose_callback: str = None, input_topic = 'fullstate'):

        super().__init__(update_frequency, pose_handler, pose_topic, pose_callback, input_topic)
        
        # get_parameter_or return None if not declared
        traj_params = self.get_parameter_or('traj_params').value

        if traj_params is not None:
        
            traj_params = self.get_parameter('traj_params').value

            self.traj_params = {}
            self.traj_params['time']          = traj_params[0:2]
            self.traj_params['position']      = [np.array(traj_params[2:5]),    np.array(traj_params[5:8])]
            self.traj_params['velocity']      = [np.array(traj_params[8:11]),   np.array(traj_params[11:14])]
            self.traj_params['acceleration']  = [np.array(traj_params[14:17]),  np.array(traj_params[17:20])]
            self.traj_params['jerk']          = [np.array(traj_params[20:23]),  np.array(traj_params[23:26])]
        
            self.start_sender = True
            self.first_evaluation = True

        else:
            self.get_logger().warn('No trajectory parameters passed by argument.')
            self.get_logger().warn('Wait for topic: "{}/{}"'.format(self.get_namespace(),self.traj_params_topic))



        self.trajectory_coeff = np.zeros((8,3))

    def traj_params_callback(self, msg):

        traj_params = msg.points[-1]
        time_traj = traj_params.time_from_start.sec + traj_params.time_from_start.nanosec*1e-9

        self.traj_params = {}
        self.traj_params['time']          = [0, time_traj]
        self.traj_params['position']      = [self.current_pose.position,    np.array(traj_params.positions)]
        self.traj_params['velocity']      = [self.current_pose.velocity,   np.array(traj_params.velocities)]
        self.traj_params['acceleration']  = [np.zeros(3),  np.array(traj_params.accelerations)]
        self.traj_params['jerk']          = [np.zeros(3),  np.array(traj_params.effort)]
    
        self.start_sender = True
        self.first_evaluation = True


    def evaluate_reference(self):
        if self.first_evaluation:
            self.start_time_sec = self.get_time()

            self.traj_params['position'][0] = np.copy(self.current_pose.position)

            # Notation from 'Trajectory planning' Melchiorri
            # Time duration [s]
            self.T = self.traj_params['time'][1] - self.traj_params['time'][0]
            
            # Displacement [m]
            self.h = self.traj_params['position'][1] - self.traj_params['position'][0]

            # Coefficient computation
            self.compute_coefficients()

            self.first_evaluation = False


        time = self.get_time() - self.start_time_sec
        
        if time > self.traj_params['time'][1]:
            self.start_sender = False


        ref = np.zeros(13)

        # position
        ref[0] = poly(self.trajectory_coeff[:,0])(time)
        ref[1] = poly(self.trajectory_coeff[:,1])(time)
        ref[2] = poly(self.trajectory_coeff[:,2])(time)

        # attitde
        attitude_reference = np.zeros(3) # Roll, Pitch, Yaw
        ref[3:7] = R.from_euler('xyz',attitude_reference).as_quat()

        # velocity
        ref[7] = poly(self.trajectory_coeff[1:,0]*np.arange(1,8))(time)
        ref[8] = poly(self.trajectory_coeff[1:,1]*np.arange(1,8))(time)
        ref[9] = poly(self.trajectory_coeff[1:,2]*np.arange(1,8))(time)

        # acceleration
        ref[10] = poly(self.trajectory_coeff[2:,0]*np.arange(2,8)*np.arange(1,7))(time)
        ref[11] = poly(self.trajectory_coeff[2:,1]*np.arange(2,8)*np.arange(1,7))(time)
        ref[12] = poly(self.trajectory_coeff[2:,2]*np.arange(2,8)*np.arange(1,7))(time)

        return ref


    def get_time(self):
        sec = self.get_clock().now().to_msg().sec
        nsec = self.get_clock().now().to_msg().nanosec/1e9
        return sec + nsec

    def compute_coefficients(self):
        self.trajectory_coeff[0] = self.traj_params['position'][0]
        self.trajectory_coeff[1] = self.traj_params['velocity'][0]
        self.trajectory_coeff[2] = self.traj_params['acceleration'][0]/2
        self.trajectory_coeff[3] = self.traj_params['jerk'][0]/6

        self.trajectory_coeff[4] = (210*self.h - self.T * (
                                        (120*self.traj_params['velocity'][0] + 90*self.traj_params['velocity'][1])+
                                        (30*self.traj_params['acceleration'][0] - 15*self.traj_params['acceleration'][1])*self.T +
                                        (4*self.traj_params['jerk'][0] + 1*self.traj_params['jerk'][1])*self.T**2
                                    ))/6/self.T**4

        self.trajectory_coeff[5] = (-168*self.h + self.T * (
                                        (90*self.traj_params['velocity'][0] + 78*self.traj_params['velocity'][1])+
                                        (20*self.traj_params['acceleration'][0] - 14*self.traj_params['acceleration'][1])*self.T +
                                        (2*self.traj_params['jerk'][0] + 1*self.traj_params['jerk'][1])*self.T**2
                                    ))/2/self.T**5

        self.trajectory_coeff[6] = (420*self.h - self.T * (
                                        (216*self.traj_params['velocity'][0] + 204*self.traj_params['velocity'][1])+
                                        (45*self.traj_params['acceleration'][0] - 39*self.traj_params['acceleration'][1])*self.T +
                                        (4*self.traj_params['jerk'][0] + 3*self.traj_params['jerk'][1])*self.T**2
                                    ))/6/self.T**6

        self.trajectory_coeff[7] = (-120*self.h + self.T * (
                                        (60*self.traj_params['velocity'][0] + 60*self.traj_params['velocity'][1])+
                                        (12*self.traj_params['acceleration'][0] - 12*self.traj_params['acceleration'][1])*self.T +
                                        (1*self.traj_params['jerk'][0] + 1*self.traj_params['jerk'][1])*self.T**2
                                    ))/6/self.T**7


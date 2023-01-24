import numpy as np
from .trajectory_handler import FullStateTrajHandler
from scipy.spatial.transform import Rotation as R
from numpy.polynomial import Polynomial as poly
from scipy.interpolate import CubicSpline,InterpolatedUnivariateSpline
import matplotlib.pyplot as plt


class Spline(FullStateTrajHandler):
    """
    From "Trajectory Planning for Automatic Machines and Robots" - L. Biagiotti, C. Melchiorri
    
    Compute a multi-point trajectory reference based on cubic spline interpolation with fixed initial and final velocities and accelerations. 
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
        
            # print(self.traj_params)

            self.start_sender = True
            self.first_evaluation = True

        else:
            self.get_logger().warn('No trajectory parameters passed by argument.')
            self.get_logger().warn('Wait for topic: "{}/{}"'.format(self.get_namespace(),self.traj_params_topic))



        self.trajectory_coeff = np.zeros((8,3))

    def traj_params_callback(self, msg):
        traj_params = msg.points
        self.traj_params = {}
        self.traj_params['time']          = []
        self.traj_params['position']      = []
        self.traj_params['velocity']      = []
        self.traj_params['acceleration']  = []
        self.traj_params['jerk']          = []

        if len(traj_params) < 2:
            # Goto point scenario -> extend the reference parameters with initial and intermediate point

            # Initial point
            self.traj_params['time'].insert(0,0.0)
            self.traj_params['position'].insert(0,list(self.current_pose.position))
            self.traj_params['velocity'].insert(0,list(self.current_pose.velocity))
            self.traj_params['acceleration'].insert(0,[0,0,0])
            self.traj_params['jerk'].insert(0,[0,0,0])

            # Final Point
            self.traj_params['time'].append(traj_params[0].time_from_start.sec + traj_params[0].time_from_start.nanosec*1e-9)
            self.traj_params['position'].append(traj_params[0].positions)
            self.traj_params['velocity'].append(traj_params[0].velocities)
            self.traj_params['acceleration'].append(traj_params[0].accelerations)
            self.traj_params['jerk'].append(traj_params[0].effort)

            # Intermediate Point
            inter_time = self.traj_params['time'][-1]*0.5
            inter_pos = list((self.current_pose.position + np.array(self.traj_params['position'][-1]))*0.5)
            inter_vel = list((self.current_pose.velocity + np.array(self.traj_params['velocity'][-1]))*0.5)
            inter_acc = list(np.array(self.traj_params['acceleration'][-1])*0.5)
            inter_jerk = list(np.array(self.traj_params['jerk'][-1])*0.5)

            self.traj_params['time'].insert(1,inter_time)
            self.traj_params['position'].insert(1,inter_pos)
            self.traj_params['velocity'].insert(1,inter_vel)
            self.traj_params['acceleration'].insert(1,inter_acc)
            self.traj_params['jerk'].insert(1,inter_jerk)


        else:
            self.get_logger().warn('[cf{}] -- Reference trajectory fixed at current height z= {}m'.format(self.agent_id,self.current_pose.position[2]))
            # Spline Trajectory -> set relative position
            for i in range(len(traj_params)):
                self.traj_params['time'].append(traj_params[i].time_from_start.sec + traj_params[i].time_from_start.nanosec*1e-9)
                relative_position = list(np.array(self.current_pose.position[0:2] - traj_params[i].positions[0:2]))
                relative_position.append(self.current_pose.position[2])
                self.traj_params['position'].append(relative_position)
                self.traj_params['velocity'].append(traj_params[i].velocities)
                self.traj_params['acceleration'].append(traj_params[i].accelerations)
                self.traj_params['jerk'].append(traj_params[i].effort)
        
        self.start_sender = True
        self.first_evaluation = True

    def evaluate_reference(self):
        if self.first_evaluation:
            self.start_time_sec = self.get_time()

            self.traj_params['position'][0] = list(self.current_pose.position)
            self.traj_params['velocity'][0] = list(self.current_pose.velocity)

            # Set initial and final velocity
            velocity_borders = [0,0] 
            # Set initial and final acceleration
            acceleration_borders = [0,0]
            # Coefficient computation
            self.spline_coefficients = self.compute_3D_spline_coefficients(velocity_borders, acceleration_borders)

            self.first_evaluation = False

        time = self.get_time() - self.start_time_sec
        if time >= self.traj_params['time'][-1]:
            time = self.traj_params['time'][-1] # clamp the computation time
            self.start_sender = False

        ref = np.zeros(13)
        # Attitide reference
        attitude_reference = np.zeros(3) # Roll, Pitch, Yaw
        ref[3:7] = R.from_euler('xyz',attitude_reference).as_quat()

        n_points = len(self.traj_params['time'])
        time_q = self.traj_params['time']
        
        for q in range(n_points-1):
            if time> time_q[q] and time <= time_q[q+1]:
                # position
                ref[0] = poly(self.spline_coefficients[0,q,:])(time-time_q[q])
                ref[1] = poly(self.spline_coefficients[1,q,:])(time-time_q[q])
                ref[2] = poly(self.spline_coefficients[2,q,:])(time-time_q[q])

                # velocity
                ref[7] = poly(self.spline_coefficients[0,q,1:].T*np.arange(1,4))(time-time_q[q])
                ref[8] = poly(self.spline_coefficients[1,q,1:].T*np.arange(1,4))(time-time_q[q])
                ref[9] = poly(self.spline_coefficients[2,q,1:].T*np.arange(1,4))(time-time_q[q])

                # acceleration
                ref[10] = poly(self.spline_coefficients[0,q,2:].T*np.arange(2,4)*np.arange(1,3))(time-time_q[q])
                ref[11] = poly(self.spline_coefficients[1,q,2:].T*np.arange(2,4)*np.arange(1,3))(time-time_q[q])
                ref[12] = poly(self.spline_coefficients[2,q,2:].T*np.arange(2,4)*np.arange(1,3))(time-time_q[q])
                break    

        return ref

    def get_time(self):
        sec = self.get_clock().now().to_msg().sec
        nsec = self.get_clock().now().to_msg().nanosec/1e9
        return (sec + nsec)

    def compute_3D_spline_coefficients(self, velocity_borders, acceleration_borders):
        n_points = len(self.traj_params['time']) + 1
        t_extra = [
                0.5*(self.traj_params['time'][0]+self.traj_params['time'][1]),
                0.5*(self.traj_params['time'][-2]+self.traj_params['time'][-1])
            ] 
        
        self.traj_params['time'].insert(1,t_extra[0])
        self.traj_params['time'].insert(-1,t_extra[1])

        T = [self.traj_params['time'][i+1]-self.traj_params['time'][i] for i in range(n_points)]

        coeffs = []
        positions = np.array(self.traj_params['position'])
        for i in range(3):
            spline_coefficients = compute_spline_coefficients(T,list(positions[:,i]),velocity_borders, acceleration_borders)
            coeffs.append(spline_coefficients)

        return np.array(coeffs)

def compute_spline_coefficients(T,q,a,v):
    """
    See Chapter §4.4.4 of "Trajectory planning for automatic machines and robots", L.Biagiotti, C. Melchiorri 
    """
    # Number of point extended to allow initial and final velociti and acceleration constraints
    N = len(q) + 1

    # Let's define the linear system Aw=c
    # Compute A
    A = np.zeros((N-1,N-1))
    for i in range(1,N-2):
        A[i,i-1] = T[i]
        A[i,i] = 2*(T[i] + T[i+1])
        A[i,i+1] = T[i+1]
    A[0,0] = 2*T[1] + T[0]*(3+T[0]/T[1])
    A[0,1] = T[1]
    A[1,0] -= T[0]*T[0]/T[1]
    A[-2,-1] -=  T[-1]*T[-1]/T[-2]
    A[-1,-2] = T[-2]
    A[-1,-1] = 2*T[-2]+T[-1]*(3+T[-1]/T[-2])

    # Compute c
    c = np.zeros(N-1)
    for i in range(1,N-2):
        c[i] = (q[i+1]-q[i])/T[i+1] -(q[i]-q[i-1])/T[i]
    c[0] = (q[1]-q[0])/T[1] - v[0]*(1+T[0]/T[1]) - a[0]*T[0]*(0.5+T[0]/T[1]/3)
    c[1] = (q[2]-q[1])/T[2] -(q[1]-q[0])/T[0] + v[0]*T[0]/T[1] + a[0]*T[0]*T[0]/T[1]/3
    c[-2] = (q[-1]-q[-2])/T[-2] -(q[-2]-q[-3])/T[-3] - v[1]*T[N-1]/T[N-2] + a[1]*T[N-1]*T[N-1]/T[N-2]/3
    c[-1] = (q[-2]-q[-1])/T[-2] + v[1]*(1+T[N-1]/T[N-2]) - a[1]*(0.5+T[N-1]/T[N-2]/3)*T[N-1]
    c *= 6

    # Solve the linear system finding the accellerations in the intermediate points
    w = (np.linalg.inv(A)@c).tolist()

    # Append initial and final accelerations
    w.insert(0,a[0])
    w.append(a[1])

    # Compute two extra point
    q_extra = [
                q[0]+T[0]*v[0]+T[0]*T[0]*a[0]/3+T[0]*T[0]/6*w[1],
                q[-1]-T[-1]*v[1]+T[-1]*T[-1]/3*a[1]+T[-1]*T[-1]/6*w[-2]
            ] 
    q.insert(1,q_extra[0])
    q.insert(N-1,q_extra[1])

    # Compute the spline coefficient
    a = np.zeros((N,4))
    for k in range(N):
        a[k,0] = q[k]
        a[k,1] = (q[k+1]-q[k])/T[k]-T[k]/6*(w[k+1]+2*w[k])
        a[k,2] = w[k]/2
        a[k,3] = (w[k+1]-w[k])/6/T[k]
        
    return a
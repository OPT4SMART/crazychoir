import numpy as np
from numpy import cos, sin
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as R
from scipy.constants import g
from .hierarchical_control import PositionCtrlStrategy

from choirbot.collision.safety_filters import SafetyFilterStrategy

class FlatnessPositionCtrl(PositionCtrlStrategy):

    def __init__(self, update_frequency, vicon= False):
        super().__init__(update_frequency)

        # Vicon Flag
        self.vicon = vicon
        
        # Control gains
        if self.vicon:
            # Vicon parameters
            self.K_p = np.array([720, 720, 1500])*1e-3
            self.K_v = np.array([120, 120, 400])*1e-3
            self.K_i = np.array([0, 0, 0])*1e-3
        else:
            # Webots parameters
            self.K_p = np.array([400, 400, 1250])*1e-3
            self.K_v = np.array([200, 200, 400])*1e-3
            self.K_i = np.array([0, 0, 0])*1e-3
        
        # Integral action settings
        self.e_i = 0
        self.delta_time = 0.01 #100Hz

        self.mass = 0.038
        

    def control(self, current_pose, desired_reference):
        # implement flatness based control scheme
        # compute desired thrust vector
        F_des = self.thrust_dir(current_pose, desired_reference)
        # compute thrust
        RR = R.from_quat(current_pose.orientation).as_matrix()
        e_3 = np.array([0,0,1])
        thrust = np.dot(F_des,np.dot(RR,e_3))

        # compute desired attitude
        if not "yaw" in desired_reference:
            desired_reference["yaw"] = 0.0


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
        self.e_i += e_p*self.delta_time
  
        F_des = self.mass*(g*e_3 + desired_reference["acceleration"]) - self.K_p*e_p - self.K_v*e_v - self.K_i*self.e_i
        return F_des


class FlatnessAccelerationCtrl(FlatnessPositionCtrl):

    def __init__(self, update_frequency, vicon= False):
        super().__init__(update_frequency)

    def thrust_dir(self, current_pose, desired_reference):
        
        # compute desired thrust vector
        e_3 = np.array([0,0,1])
        if not "acceleration" in desired_reference:
            desired_reference["acceleration"] = np.zeros(3)
            
        F_des = self.mass*(g*e_3 + desired_reference["acceleration"])

        return F_des


class SafeFlatnessPositionCtrl(FlatnessPositionCtrl):
    
    def __init__(self, agent_id:int, update_frequency: float, safety_filter: SafetyFilterStrategy, max_acceleration: float, vicon= False):
        super().__init__(update_frequency, vicon)

        self.agent_id = agent_id
        self.safety_filter = safety_filter
        self.max_acceleration = max_acceleration

    def thrust_dir(self, current_pose, desired_reference, obstacles):

        # compute desired thrust vector
        e_3 = np.array([0,0,1])
        e_p =  current_pose.position - desired_reference["position"]
        e_v =  current_pose.velocity - desired_reference["velocity"]
        self.e_i += e_p*self.delta_time
  
        F_des = self.mass*(g*e_3 + desired_reference["acceleration"]) - self.K_p*e_p - self.K_v*e_v - self.K_i*self.e_i
        

        # apply safety filter
        x = current_pose.position[:2].tolist()
        x += current_pose.velocity[:2].tolist()
        u = F_des[:2].tolist()

        safe_input = self.safety_filter.filter_desired_input(
            state = x,
            desired_input = u, 
            obstacles = obstacles
        )

        if safe_input is None:
            print("safe input is None")
            safe_input = F_des.tolist()

        # check if each element of the input is a float
        if not all(isinstance(u_i, (int, float)) for u_i in safe_input):
            raise ValueError(f"safe input {safe_input} must be a list of floats")
        
        safe_input = np.array([safe_input[0], safe_input[1], F_des[2]])


        return safe_input
    
    def control(self, current_pose, desired_reference, obstacles: list = []):
        # implement flatness based control scheme
        # compute desired thrust vector
        F_des = self.thrust_dir(current_pose, desired_reference, obstacles)
        # compute thrust
        RR = R.from_quat(current_pose.orientation).as_matrix()
        e_3 = np.array([0,0,1])
        thrust = np.dot(np.dot(RR,F_des),e_3)

        # compute desired attitude
        if not "yaw" in desired_reference:
            desired_reference["yaw"] = 0.0


        z_b_des = F_des/norm(F_des)
        x_c_des = np.array([cos(desired_reference["yaw"]),sin(desired_reference["yaw"]),0]).transpose()
        y_b_des = np.cross(z_b_des, x_c_des)/norm(np.cross(z_b_des, x_c_des))
        x_b_des = np.cross(y_b_des,z_b_des)
        R_des = np.array([x_b_des, y_b_des, z_b_des]).transpose()

        return thrust, R_des

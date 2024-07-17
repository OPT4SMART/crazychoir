"""
The .urdf file of the Crazyflie needs the Module 'MotorCtrl*' to link the 
motor controller plugin to the webots model. 
"""

from .motor_ctrl_fpqr import MotorCtrlFPQR
from .motor_ctrl_xyz import MotorCtrlXYZ
from .motor_ctrl_vel import MotorCtrlVel
from .real_pose_ctrl import RealPoseCtrl
from .real_pose_ctrl import RealPoseCtrlLighthouse
from .real_pose_ctrl import RealPoseCtrlVicon
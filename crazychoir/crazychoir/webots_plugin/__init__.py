"""
The .urdf file of the Crazyflie needs the Module 'MotorCtrl' to link the 
motor controller plugin to the webots model. 
Thus, import the the desired plugin as 'MotorCtrl'
"""

# TODO: Improve this mechanism

from .motor_ctrl_fpqr import MotorCtrlFPQR as MotorCtrl
# from .motor_ctrl_xyz import MotorCtrlXYZ as MotorCtrl
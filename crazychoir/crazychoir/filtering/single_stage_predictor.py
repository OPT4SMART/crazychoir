import numpy as np

class SingleStagePredictor:

  def __init__(self, dt = 0.01,dt_dyn = 1e-2,
                sigma_u = 10, sigma_m = 1e-4,  
                x0 = np.zeros((4,1)), PP0 = np.zeros((4,4))):
    """
      Double integrator dynamics
      Kalman-filter formalism
    """

    self.dt = dt 

    self.FF = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
    self.GG = np.array([[0, 0], [dt, 0], [0, 0], [0, dt]])
    self.HH = np.array([[1,0, 0, 0],[0,0,1,0]])   # only the two positions are measured

    self.nx = np.shape(self.FF)[0]
    self.nu = np.shape(self.GG)[1]
    self.ny = np.shape(self.HH)[0]

    self.sigma_u = sigma_u # std dev in input
    self.sigma_m = sigma_m # std dev in measurement

    self.QQ = self.GG@self.GG.T*self.sigma_u**2
    self.RR = np.diag(self.sigma_m**2*np.ones((self.ny,)))

    self.xx_predict = x0
    self.PP_predict = PP0

    self.KK = self.observer_gain(self.PP_predict, self.RR)

    self.AA = np.array([[1, dt_dyn, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt_dyn], [0, 0, 0, 1]])
    self.BB = np.array([[0, 0], [dt_dyn, 0], [0, 0], [0, dt_dyn]])

  def ss_predict(self, zz):
    """
      single stage Prediction
      - predicts next state, covariance
    """
    self.KK = self.observer_gain(self.PP_predict, self.RR)
    xx = self.prediction_state(self.xx_predict, zz)
    PP = self.prediction_covariance(self.PP_predict, self.RR)

    self.xx_predict = xx
    self.PP_predict = PP

    return xx, PP

  def measurement(self,xx):
    
    vv = np.random.multivariate_normal(np.zeros((self.ny, )), self.RR)
  
    zz = self.HH@xx + vv[:,None]

    return zz

  def prediction_state(self, xx, zz):
    """
    Single-stage predictor
    xx -> xx_plus
    """

    xxp = (self.FF -self.KK@self.HH)@xx + self.KK@zz

    return xxp

  def prediction_covariance(self, PP, RR):
    """
    Single-stage predictor
    PP -> PP_plus
    """

    PPp = (self.FF -self.KK@self.HH)@PP@(self.FF -self.KK@self.HH).T + \
              self.KK@RR@self.KK.T + self.QQ

    return PPp

  def observer_gain(self, PP, RR):
    """
    Optimal observer
      - covariance matrix (P)
      - R matrix (measurement)
    """

    HH = self.HH

    KK = self.FF@PP@HH.T@np.linalg.inv(HH@PP@HH.T + RR)

    return KK

  def correction_state(self, xx, zz):
    """
    xx_minus, zz -> xx
    """

    KK = self.KK
    HH = self.HH

    xx_hat = xx + KK@(zz - HH@xx)

    return xx_hat

  def correction_convariance(self, PP):
    """
    PP_minus -> PP
    """

    II = np.eye(self.nx)
    KK = self.KK
    HH = self.HH
    RR = self.RR

    PP_correct = (II - KK@HH)@PP

    return PP_correct

  def dynamics(self, xx, uu):

    xxp = self.AA@xx + self.BB@uu

    return xxp 









    


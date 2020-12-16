
import numpy as np
import vtolParamhw12 as P
import vtolParam as P1

class PDController:
    def __init__(self):
        
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0   # error signal delayed by 1 sample
        self.K_z = P.K_z  # state feedback gain
        self.K_h = P.K_h  # state feedback gain
        self.ki_z = P.ki_z  # integrator gain
        self.ki_h = P.ki_h  # integrator gain
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        
#------------------------------------------------------------------------------
    def update_lat(self, zref,state):
        # Compute the state feedback controller
        z = state
        
        error = zref - z
        self.integrateError(error)
        F_unsat = self.K_z @ z + self.ki_z *self.integrator
        
        tau = self.saturate(F_unsat[0])
        return tau.item(0)
    
#------------------------------------------------------------------------------
    def update_lon(self,ref,state):
        x = state

        #F = (P.mc+P.ml+P.mr)*P.g + self.kp * (ref - x) - self.kd * xdot
        
        x = state
        # integrate error
        error = ref - x
        self.integrateError(error)
        # Compute the state feedback controller
        F_unsat = (P1.mc+P1.ml+P1.mr)*P1.g  -self.K_h @ x + self.ki_h*self.integrator
        F = self.saturate(F_unsat[0])
        return F.item(0)
    
#------------------------------------------------------------------------------
    
    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
    
    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error
'''

class PDController:
    def __init__(self):
        
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0   # error signal delayed by 1 sample
        self.K_z = P.K_z  # state feedback gain
        self.K_h = P.K_h  # state feedback gain
        self.ki_z = P.ki_z  # integrator gain
        self.ki_h = P.ki_h  # integrator gain
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        
#------------------------------------------------------------------------------
    def update_lat(self,zref,state):

        #F = (P.mc+P.ml+P.mr)*P.g + self.kp * (ref - x) - self.kd * xdot
    
        z = state
        
        #F_unsat = -self.K_z @ z + self.kr_z * zref
        #tau = self.saturate(F_unsat)
        
        error = zref - z
        self.integrateError(error)
        tau = -self.K_z @ z - self.ki_z *self.integrator
        tau = self.saturate(tau)
        #tau = -self.kp_th * (theta_ref + th) - self.kd_th * thdot
        return tau.item(0)

#------------------------------------------------------------------------------
    def update_lon(self,ref,state):
        # Compute the state feedback controller
    
        x = state
        # integrate error
        error = ref - x
        self.integrateError(error)
        # Compute the state feedback controller
        F_unsat = (P1.mc+P1.ml+P1.mr)*P1.g  -self.K_h @ x - self.ki_h*self.integrator
        F = self.saturate(F_unsat)
        return F.item(0)
    
    
#------------------------------------------------------------------------------
    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
    
'''
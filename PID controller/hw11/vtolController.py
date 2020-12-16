'''import numpy as np
import vtolParam as P

class PDController:
    def __init__(self,kp,kd,kp2=0,kd2 = 0):
        self.kp = kp
        self.kd = kd
        self.kp_th = kp2
        self.kd_th = kd2

    def update_lon(self,ref,state,statedot):
        x = state
        xdot = statedot
        F = (P.mc+P.ml+P.mr)*P.g + self.kp * (ref - x) - self.kd * xdot
        return F
    
    def update_lat(self,zref,state):
        z = state.state.item(1)
        zdot = state.state.item(4)
        th = state.state.item(3)
        thdot = state.state.item(6)

        theta_ref = self.kp * (zref - z) - self.kd * zdot
        tau = -self.kp_th * (theta_ref + th) - self.kd_th * thdot
        return tau
'''



import numpy as np
import vtolParamhw11 as P
import vtolParam as P1
class PDController:
    def __init__(self):
        self.K_z = P.K_z  # state feedback gain
        self.K_h = P.K_h  # state feedback gain
        self.kr_z = P.kr_z  # Input gain
        self.kr_h = P.kr_h  # Input gain
        self.limit = P.F_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

#------------------------------------------------------------------------------
    def update_lat(self, zref,state):
        # Compute the state feedback controller
        z = state
        
        F_unsat = -self.K_z @ z + self.kr_z * zref
        tau = self.saturate(F_unsat)
        return tau
    
#------------------------------------------------------------------------------
    def update_lon(self,ref,state):
        x = state

        #F = (P.mc+P.ml+P.mr)*P.g + self.kp * (ref - x) - self.kd * xdot
        
        F_unsat = (P1.mc+P1.ml+P1.mr)*P1.g -self.K_h @ x + self.kr_h * ref
        F = self.saturate(F_unsat)
        return F
    
#------------------------------------------------------------------------------
    
    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

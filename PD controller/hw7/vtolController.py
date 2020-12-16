import numpy as np
import vtolParam as P
import sys
sys.path.append('/Users/pratik/Documents - pratik’s MacBook Pro/semester 5/Control Systems/VTOL/vtol')  # add parent directory
import vtolParamhw3 as P0

class vtolController:
    def __init__(self):

        # Instantiates the PD object
        self.kp = P.kp
        self.kd = P.kd
        self.limit = P0.F_max

    def update(self, fr x):
        fr = x.item(0)
        fl = x.item(1)
        frdot = x.item(2)
        fldot = x.item(3)

        x = state
        xdot = statedot
        F = (P.mc+P.ml+P.mr)*P.g + self.kp * (ref - x) - self.kd * xdot
    
        # feedback linearized force
        fr_fr = (2*(P0.ml+P0.mr) + P0.mc)*P0.g
        fl_fl = (2*(P0.ml+P0.mr) + P0.mc)*P0.g

        # equilibrium force around f_e = 0
        f_e= (2*(P0.ml+P0.mr) + P0.mc)*P0.g
        #f_e = P0.m * P0.g * P0.ell/2.0 * np.cos(theta_e)

        # compute the linearized torque using PD control
        fr_tilde = self.kp * (fr_r - fr) - self.kd * (frdot)
        
        fl_tilde = self.kp * (fl_l - fl) - self.kd * (fldot)
        
        # compute total force
        f1 = fr_fr + fr_tilde
        f2 = fl_fl + fl_tilde
        #tau = tau_e + tau_tilde
        # always saturate to protect hardware
        f1 = self.saturate(f1)
        f2 = self.saturate(f2)
        out = [f1,f2]
        return out

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u








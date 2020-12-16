import numpy as np
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
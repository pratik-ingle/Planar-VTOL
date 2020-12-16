import numpy as np
import vtolParam as P
import vtolParamhw10 as P10
from PIDControl import PIDControl

class vtolController:
    def __init__(self):
        # Instantiates the SS_ctrl object
        self.zCtrl = PIDControl(P10.kp_z, P10.ki_z, P10.kd_z,
                                P10.theta_max, P10.sigma, P10.Ts)
        self.hCtrl = PIDControl(P10.kp_th, 0.0, P10.kd_th,
                                    P10.F_max, P10.sigma, P10.Ts)
        self.thetaCtrl = PIDControl(P10.kp_th, 0.0, P10.kd_th,
                                    P10.F_max, P10.sigma, P10.Ts)
        
        
        self.filter = zeroCancelingFilter()

    def update(self, z_r,z_l, y):
        z = y.item(0)
        theta = y.item(1)
        
        # the reference angle for theta comes from
        # the outer loop PID control
        theta_r = self.zCtrl.PID(z_r, z, flag=False)

        # low pass filter the outer loop to cancel
        # left-half plane zero and DC-gain
        theta_r = self.filter.update(theta_r)

        # the force applied to
        # the inner loop PD control
        f1 = self.thetaCtrl.PD(theta_r, theta, flag=False)
        f2 = self.thetaCtrl.PD(theta_r, theta, flag=False)
        F = [f1,f2]
        return F

class zeroCancelingFilter:
    def __init__(self):
        self.a = -3.0/(2.0*P.d*P10.DC_gain)
        self.b = np.sqrt(3.0*P.g/(2.0*P.d))
        self.state = 0.0

    def update(self, input):
        # integrate using RK1
        self.state = self.state \
                     + P.Ts * (-self.b*self.state + self.a*input)
        return self.state







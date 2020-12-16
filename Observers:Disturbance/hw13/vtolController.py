
import numpy as np
import vtolParamhw13 as P
import vtolParam as P1

class PDController:
    def __init__(self):
        self.x1_hat = np.array([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0]])  # initial estimate for theta_hat_dot
        self.x2_hat =np.array([
            [0.0],  # initial estimate for h_hat
            [0.0]])  # initial estimate for h_hat_dot
        
        self.F_lat = 0.0  # Computed latral Force, delayed by one sample
        self.F_lon = 0.0  # Computed longitudinal Force, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K_z = P.K_z  # state feedback gain
        self.K_h = P.K_h  # state feedback gain
        self.ki_z = P.ki_z  # integrator gain
        self.ki_h = P.ki_h  # integrator gain
        self.L_z = P.L_z  # observer gain
        self.L_h = P.L_h  # observer gain
        self.A_z = P.A_z  # system model lat
        self.B_z = P.B_z
        self.C_z = P.C_z
        self.A_h = P.A_h  # system model long
        self.B_h = P.B_h
        self.C_h = P.C_h
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        
#------------------------------------------------------------------------------

#    def update_lat(self, zref,state):
#        # Compute the state feedback controller
#        z = state
        
#        error = zref - z
#        self.integrateError(error)
#        F_unsat = self.K_z @ z + self.ki_z *self.integrator
#        
#        tau = self.saturate(F_unsat[0])
#        return tau.item(0)
     
    
    def update_lat(self, z_r, y):
        # update the observer and extract z_hat
        x1_hat = self.update_observer_z(y)
        z_hat = x1_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrate_error(error)

        # Compute the state feedback controller
        F_unsat = -self.K_z*x1_hat - self.ki_z*self.integrator

        F_sat = self.saturate(F_unsat.item(0))
        self.F_lat= F_sat

        return F_sat, x1_hat
    
    def update_observer_z(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lat(self.x1_hat, y_m)
        F2 = self.observer_f_lat(self.x1_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lat(self.x1_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lat(self.x1_hat + self.Ts * F3, y_m)
        self.x1_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

        return self.x1_hat

    def observer_f_lat(self, x1_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        x1hat_dot = self.A_z @ x1_hat \
                   + self.B_z * self.F_lat \
                   + self.L_z @ (y_m-self.C_z @ x1_hat)

        return x1hat_dot
#------------------------------------------------------------------------------
#    def update_lon(self,ref,state):
#        x = state
#
#        #F = (P.mc+P.ml+P.mr)*P.g + self.kp * (ref - x) - self.kd * xdot
#        
#        x = state
#        # integrate error
#        error = ref - x
#        self.integrateError(error)
#        # Compute the state feedback controller
#        F_unsat = (P1.mc+P1.ml+P1.mr)*P1.g  -self.K_h @ x + self.ki_h*self.integrator
#        F = self.saturate(F_unsat[0])
#        return F.item(0)
    
    def update_lon(self, h_r, y):
        # update the observer and extract z_hat
        x2_hat = self.update_observer_h(y)
        h_hat = x2_hat.item(0)

        # integrate error
        error = h_r - h_hat
        self.integrate_error(error)

        # Compute the state feedback controller
        F_unsat = (P1.mc+P1.ml+P1.mr)*P1.g -self.K_h*x2_hat - self.ki_h*self.integrator

        F_sat = self.saturate(F_unsat.item(0))
        self.F_lon= F_sat

        return F_sat, x2_hat
    
    def update_observer_h(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lon(self.x2_hat, y_m)
        F2 = self.observer_f_lon(self.x2_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lon(self.x2_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lon(self.x2_hat + self.Ts * F3, y_m)
        self.x2_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        
        return self.x2_hat

    def observer_f_lon(self, x2_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        print(self.x2_hat,y_m)
        x2hat_dot = self.A_h @ x2_hat \
                   + self.B_h * self.F_lon \
                   + self.L_h @ (y_m-self.C_h @ x2_hat)

        return x2hat_dot
#------------------------------------------------------------------------------
    
    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
    
    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

#-------------------------------------------------------------------------------------
'''        
import numpy as np
import pendulumParamHW13 as P

class pendulumController:
    def __init__(self):
        self.x_hat = np.array([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0]])  # initial estimate for theta_hat_dot
        self.F_d1 = 0.0  # Computed Force, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki  # integrator gain
        self.L = P.L  # observer gain
        self.A = P.A  # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, y):
        # update the observer and extract z_hat
        x_hat = self.update_observer(y)
        z_hat = x_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrate_error(error)

        # Compute the state feedback controller
        F_unsat = -self.K*x_hat - self.ki*self.integrator

        F_sat = self.saturate(F_unsat.item(0))
        self.F_d1 = F_sat

        return F_sat, x_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y_m)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

        return self.x_hat

    def observer_f(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A @ x_hat \
                   + self.B * self.F_d1 \
                   + self.L @ (y_m-self.C @ x_hat)

        return xhat_dot

    def integrate_error(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u
'''

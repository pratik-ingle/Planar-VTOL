import numpy as np 
import random
import vtolParam as P

class vtolDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.zt0],   # target location
            [P.zv0],  # initial distance
            [P.h0],   # intial height
            [P.theta0],  # initial base angle
            [P.zvdot0],  # initial velocity horizontal
            [P.hdot0],    # initial velocity vetical
            [P.thetadot0]  # initial angular velocity 
        ])

        # simulation time step
        self.Ts = P.Ts
        
        # Mass of the base, kg
        self.mc = P.mc * (1.+alpha*(2.*np.random.rand()-1.))
        
        # Mass of the right arm, kg
        self.mr = P.ml * (1.+alpha*(2.*np.random.rand()-1.))

        # Mass of the left arm, kg
        self.ml = P.mr * (1.+alpha*(2.*np.random.rand()-1.))

        # Length of the rod, m
        self.d = P.d * (1.+alpha*(2.*np.random.rand()-1.))

        # Damping coefficient, Ns
        self.b = P.b * (1.+alpha*(2.*np.random.rand()-1.))

         # inertia of base
        self.Jc = P.Jc * (1.+alpha*(2.*np.random.rand()-1.))  
        
        self.fl = P.fl * (1.+alpha*(2.*np.random.rand()-1.))
        
        self.fr = P.fr * (1.+alpha*(2.*np.random.rand()-1.))
        

        # gravity constant is well known, don't change.
        self.g = P.g
        self.force_limit = P.F_max
        
        
    def update(self, u,v):
    
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input force
        u = self.saturate(u, self.force_limit)
        v = self.saturate(v, self.force_limit)

        self.rk4_step(u,v)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        #z = self.h()

        return y
    


    def f(self, state, u,v):
        # Return xdot = f(x,u)
        zt = state.item(0)
        zv = state.item(1)
        h = state.item(2)
        theta =2*np.pi/180 + state.item(3)
        zvdot = state.item(4)
        hdot = state.item(5)
        thetadot = state.item(6)
        fl = u
        fr = v
        # The equations of motion.
        '''M = np.array([[self.mc +self.ml +self.ml , 0, 0 ],
                      [0, self.mc +self.ml +self.ml, 0],
                      [0, 0, self.Jc]])
        C = np.array([[(fl+fr)*zv + self.d*(fr-fl)*np.cos(theta)-self.b*zvdot, 0 , 0],
                      [0, (fl+fr)*h + self.d*(fr-fl)*np.sin(theta)-((self.mc +self.ml +self.ml)*self.g) , 0],
                      [0, 0, -(self.mr*self.g*self.d*np.cos(theta)) + self.ml*self.g*self.d*np.cos(theta)]])'''
        
        '''M = np.array([[self.mc +self.ml +self.ml , 0, 0 ],
                      [0, self.mc +self.ml +self.ml, 0],
                      [0, 0, self.Jc + 2*(self.ml +self.ml)*(self.d**2)]])
        C = np.array([-(fr+fl)*np.sin(theta)-self.b*zvdot,
                      ((fr+fl)*np.cos(theta)+((self.mc +self.ml +self.ml)*self.g)),
                      (fr-fl)*self.d])'''
        
        M = np.array([[self.mc +self.ml +self.ml , 0, 0 ],
                      [0, self.mc +self.ml +self.ml, 0],
                      [0, 0, self.Jc + 2*(self.ml +self.ml)*(self.d**2)]])
        C = np.array([-(fr+fl)*np.sin(theta)-self.b*zvdot,
                      ((fr+fl)*np.cos(theta)-((self.mc +self.ml +self.ml)*self.g)),
                      (fr-fl)*self.d])


        '''M = np.array([[self.mc +self.ml +self.ml , 0, 0 ],
                      [0, self.mc +self.ml +self.ml, 0],
                      [0, 0, self.Jc + (self.ml +self.ml)*(self.d**2)]])
        C = np.array([(fr+fl)*np.sin(theta)-self.b*zvdot,
                      ((fr+fl)*np.cos(theta)-((self.mc +self.ml +self.ml)*self.g)),
                      (fr-fl)*self.g])'''
        
        tmp = np.linalg.inv(M) @ C
        ztdot = 0
        zvddot = tmp.item(0)
        hddot = tmp.item(1)
        thetaddot = tmp.item(2)
        
        # build xdot and return
        xdot = np.array([[ztdot], [zvdot],[hdot], [thetadot], [zvddot],[hddot], [thetaddot]])
        
        return xdot

    def h(self):
        # return y = h(x)
        zt = self.state.item(0)
        zv = self.state.item(1)
        h = self.state.item(2)
        theta = self.state.item(3)
        y = np.array([[zt],[zv],[h],[theta]])

        return y

    def rk4_step(self, u,v):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u,v)
        F2 = self.f(self.state + self.Ts / 2 * F1, u,v)
        F3 = self.f(self.state + self.Ts / 2 * F2, u,v)
        F4 = self.f(self.state + self.Ts * F3, u,v)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit*np.sign(u)
        return u

# VTOL Parameter File
import numpy as np
#import control as cnt

# Physical parameters of the VTOL known to the controller
mc = 2          # Mass of the , kg
mr  = 0.3   # Mass of the cart, kg
ml = 0.3
d = 0.35        # Length of the arm
g = 9.8         # Gravity, m/s**2
u = 0.13        # Damping coefficient, kg/s
jc = 0.0084     #kgm**2



# parameters for animation
w = 0.3       # Width of the , mc
h = 0.2      # Height of the cart, mc
#gap = 0.005   # Gap between the cart and x-axis
radius = 0.06 # Radius of circular trust
length = 1.0  # length of arm
width = 0.1   # width of arm
gap = 0.01


# Initial Conditions
zt0 = 0.0                # ,m
zv0 = 0.0                # ,m
h0 = 0.0                # ,m
theta0 = 0.0*np.pi/180  # ,rads
zdot0 = 0.0             # ,m/s
thetadot0 = 0.0         # ,rads/s



# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate



# saturation limits
F_max = 5.0                # Max Force, N


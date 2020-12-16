# VTOL Parameter File

import numpy as np

# Simulation Parameter

t_start = 0.0                       # Start time of simulation
t_end = 50.0                        # End time of simulation
Ts = 0.01                           # Sample time for simulation
t_plot = 0.1                        # Plotting and animation update rate
inner_loop_rate = 0.001


#  VTOL parameters DO NOT CHANGE  


mc = 2                          # Mass of the body, kg
mr  = 0.3                      # Mass of the motor, kg
ml = 0.3
d = 0.35                         # Length of the arm, m
g = 9.81                        # Acceleration due to the gravity, m/s**2
b = 0.13                         # Damping coefficient, kg/s
Jc = 0.0084                     # Moment of inertia, kgm**2


#  Parameters for animation    


w = 0.3                             # Width of the body , mc
h = 0.2                             # Height of the body, mc
radius = 0.06                       # Radius of propellers
length = 0.3                        # Length of plot screen


#   Initial Conditions    


zt0 = 0.0                           # ,m
zv0 = 1.0                           # ,m
h0 = 0.0                            # ,m
theta0 = 0.0                        # ,rads
zvdot0 = 0.0                        # ,m/s
hdot0 = 0.0                         # ,m/s
thetadot0 = 0.0*np.pi/180           # ,rads/s
fl = 1.5                            # ,N
fr = 1.5                            # ,N


# Saturation limits    


F_max = 30                          # Max Force, N
theta_max = np.pi/6                 # Max theta, rad
error_max = 1        		        # Max step size,m


#   PD Control: Time Design Strategy    



#  home work 7  


kp_lon7 = 0.06*(mc+ml+mr)
kd_lon7 = 0.5*(mc+ml+mr)

# home work 8

kp_lon8 = 0.075625*(mc+ml+mr)
kd_lon8 = 0.38885*(mc+ml+mr)
kp_lat_z8 = 0.075625/g
kd_lat_z8 = (0.38885*(mc+ml+mr) - b)/((mc+ml+mr)*g)
kp_lat_theta8 = 7.5625*(Jc+(ml+mr)*d**2)
kd_lat_theta8 = 3.885*(Jc+(ml+mr)*d**2)


#  Dirty derivative parameter  


Ts = 0.001
sigma = 0.05                                    # Cutoff freq for dirty derivative
beta = (2 * sigma - Ts) / (2 * sigma + Ts)      # Dirty derivative gain


print('kp_th: ', kp_lat_theta8)
print('kd_th: ', kd_lat_theta8)
print('kp_z: ', kp_lat_z8)
print('kd_z: ', kd_lat_z8)
print('kp_h: ', kp_lon8)
print('kd_h: ', kd_lon8)
import matplotlib.pyplot as plt
import sys
import vtolParam as P
from signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter
from vtolDynamics import vtolDynamics
import vtolController as PD
import numpy as np

# Instantiate VTOL modules
vtol = vtolDynamics()
# lon_controller_f7 = PD.PDController(P.kp_lon7,P.kd_lon7)
lon_controller_f8 = PD.PDController(P.kp_lon8,P.kd_lon8)
# theta_controller_f8 = PD.PDController(P.kp_lat_theta8,P.kd_lat_theta8)
# z_controller_f8 = PD.PDController(P.kp_lat_z8,P.kd_lat_z8)
lat_controller = PD.PDController(kp2=P.kp_lat_theta8,kd2 = P.kd_lat_theta8,kp = P.kp_lat_z8,kd = P.kd_lat_z8)

h_reference = signalGenerator(amplitude=1, frequency=0.05,y_offset=2)
theta_reference = signalGenerator(amplitude=0, frequency=0.05,y_offset=0)
z_reference = signalGenerator(amplitude=2.5, frequency=0.05,y_offset=3)

# disturbance = signalGenerator(amplitude=0.5, frequency=0.1)
# force_left = signalGenerator(amplitude=0.5, frequency=0.4,y_offset=55)
# force_right = signalGenerator(amplitude=0.5, frequency=0.4,y_offset=55)

# Instantiate the simulation plots and animation modules
dataPlot = dataPlotter()
animation = vtolAnimation()

t = P.t_start
y = vtol.h()

while t < P.t_end:  # main simulation loop

    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        href = h_reference.step(t)
        zref = z_reference.square(t)
        # thetaref = theta_reference.square(t)
        # d = disturbance.step(t)  # input disturbance
        # u = force_left.sin(t)
        # v = force_right.sin(t)
        # vtol.update(u,v) 
        vtol.F_lon = lon_controller_f8.update_lon(href,vtol.state.item(2),vtol.state.item(5))
        # tin = t
        # while tin<t+P.Ts:
        #     vtol.t_lat_theta = theta_controller_f8.update(thetaref,vtol.state.item(3),vtol.state.item(6))
        #     vtol.update()  # Propagate the dynamics
        #     tin = tin + P.inner_loop_rate
        vtol.t_lat_theta = lat_controller.update_lat(zref,vtol)
        vtol.update()  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(vtol.state)
    dataPlot.update(t, href,zref,vtol.state,vtol.F_lon,vtol.t_lat_theta)#vtol.F_lat_z,vtol.t_lat_theta)

    # the pause causes the figure to be displayed during the
    # simulation
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()

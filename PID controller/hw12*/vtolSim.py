import sys
import matplotlib.pyplot as plt
import vtolParam as P
from signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter
from vtolDynamics import vtolDynamics
import vtolController as PD
import numpy as np


# Instantiate VTOL modules
vtol = vtolDynamics()
#lon control input
###########lon_controller_f8 = PD.PDController(P.kp_lon8,P.kd_lon8)
lon_controller = PD.PDController()
#lat control input
###########lat_controller = PD.PDController(kp2=P.kp_lat_theta8,kd2 = P.kd_lat_theta8,kp = P.kp_lat_z8,kd = P.kd_lat_z8)
lat_controller = PD.PDController()

#reference 
h_reference = signalGenerator(amplitude=1, frequency=0.05,y_offset=2)
theta_reference = signalGenerator(amplitude=0, frequency=0.05,y_offset=0)
z_reference = signalGenerator(amplitude=2.5, frequency=0.05,y_offset=3)
disturbance = signalGenerator(amplitude=0.1)



dataPlot = dataPlotter()
animation = vtolAnimation()

t = P.t_start
y = vtol.h()

while t < P.t_end:  

    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

   #inner loop
    while t < t_next_plot:  
        href = h_reference.step(t)
        zref = z_reference.square(t)
        d =0.0 # disturbance.sin(t) # disturbance.step(t)  # input disturbance
        n = 0.0  #noise.random(t)  # simulate sensor noise
        x = vtol.state
        x_z = np.array([[vtol.state.item(1)],[vtol.state.item(3)],[vtol.state.item(4)],[vtol.state.item(6)]]) 
        x_h = np.array([[vtol.state.item(2)],[vtol.state.item(5)]])
        vtol.F_lon = lon_controller.update_lon(href,x_h + d) #vtol.state.item(2),vtol.state.item(5))
        vtol.t_lat_theta = lat_controller.update_lat(zref,x_z + d)
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

#######################

import sys
import matplotlib.pyplot as plt
import vtolParam as P
from signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter
from vtolDynamics import vtolDynamics
import vtolController as PD
import numpy as np
from dataPlotterObserver import dataPlotterObserver


# Instantiate VTOL modules
vtol = vtolDynamics(alpha=0.0)
#lon control input
###########lon_controller_f8 = PD.PDController(P.kp_lon8,P.kd_lon8)
lon_controller = PD.PDController()
#lat control input
###########lat_controller = PD.PDController(kp2=P.kp_lat_theta8,kd2 = P.kd_lat_theta8,kp = P.kp_lat_z8,kd = P.kd_lat_z8)
lat_controller = PD.PDController()

reference = signalGenerator(amplitude=0.5, frequency=0.05)
disturbance = signalGenerator(amplitude=0.5)
noise_z = signalGenerator(amplitude=0.01)
noise_th = signalGenerator(amplitude=0.01)
noise_h = signalGenerator(amplitude=0.01)

#reference 
h_reference = signalGenerator(amplitude=1, frequency=0.05,y_offset=2)
theta_reference = signalGenerator(amplitude=0, frequency=0.05,y_offset=0)
z_reference = signalGenerator(amplitude=2.5, frequency=0.05,y_offset=3)
disturbance = signalGenerator(amplitude=0.1)


# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = vtolAnimation()
dataPlotObserver = dataPlotterObserver()



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
        n = np.array([[0.0], [0.0]])  #noise.random(t)  # simulate sensor noise
        x_z = np.array([[vtol.state.item(1)],[vtol.state.item(3)],[vtol.state.item(4)],[vtol.state.item(6)]]) 
        x_h = np.array([[vtol.state.item(2)],[vtol.state.item(5)]])
        vtol.F_lon , x2hat = lon_controller.update_lon(href,x_h + n + d) #vtol.state.item(2),vtol.state.item(5))
        vtol.t_lat_theta , x1hat = lat_controller.update_lat(zref,x_z + n + d)
        vtol.update()  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(vtol.state)
    dataPlot.update(t, href,zref,vtol.state,vtol.F_lon,vtol.t_lat_theta)#vtol.F_lat_z,vtol.t_lat_theta)
    dataPlotObserver.update(t, vtol.state, x1hat,x2hat, d, 0.0)
    # the pause causes the figure to be displayed during the
    # simulation
    plt.pause(0.0001)  
    

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()

#######################
'''
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import pendulumParam as P
from hw3.pendulumDynamics import pendulumDynamics
from pendulumController import pendulumController
from hw2.signalGenerator import signalGenerator
from hw2.pendulumAnimation import pendulumAnimation
from hw2.dataPlotter import dataPlotter
from dataPlotterObserver import dataPlotterObserver

# instantiate pendulum, controller, and reference classes
pendulum = pendulumDynamics(alpha=0.0)
controller = pendulumController()
reference = signalGenerator(amplitude=0.5, frequency=0.05)
disturbance = signalGenerator(amplitude=0.5)
noise_z = signalGenerator(amplitude=0.01)
noise_th = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
y = pendulum.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)
        d = 0  #disturbance.step(t)
        n = np.array([[0.0], [0.0]])
        u, xhat = controller.update(r, y + n)
        y = pendulum.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(pendulum.state)
    dataPlot.update(t, r, pendulum.state, u)
    dataPlotObserver.update(t, pendulum.state, xhat, d, 0.0)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
'''
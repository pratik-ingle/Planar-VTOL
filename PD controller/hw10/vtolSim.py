import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import vtolParam as P
from vtolDynamics import vtolDynamics
from vtolController import vtolController
from hw2.signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter


# instantiate vtol, controller, and reference classes
vtol = vtolDynamics()
controller = vtolController()
reference = signalGenerator(amplitude=3, frequency=0.08)
disturbance = signalGenerator(amplitude=0.0)
noise = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = vtolAnimation()

t = P.t_start  # time starts at t_start
y = vtol.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot: 
        r = reference.square(t)
        l = reference.square(t)
        d = disturbance.step(t)  # input disturbance
        n = noise.random(t) #noise.random(t)  # simulate sensor noise
        x = vtol.state
        u = controller.update(r,l, x)  # update controller
        y = vtol.update(u[0] + d, u[1]+d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(vtol.state)
    dataPlot.update(t, r, vtol.state, u[0],u[1])

    # the pause causes the figure to be displayed for simulation
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()

import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import vtolParam as P
from hw2.signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter
from vtolDynamics import vtolDynamics

# instantiate satellite, controller, and reference classes
vtol = vtolDynamics()
reference = signalGenerator(amplitude=0.5, frequency=0.1)
force_left = signalGenerator(amplitude=1, frequency=0.8)
force_right = signalGenerator(amplitude=1, frequency=0.2)
#theta = signalGenerator(amplitude=0.1, frequency=0.1)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = vtolAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        r = reference.sin(t)
        u =13+ force_left.sin(t)
        v =13+ force_right.square(t)
        y = vtol.update(u,v)  # Propagate the dynamics
        #z = vtol.update2(v)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(vtol.state)
    dataPlot.update(t, r, vtol.state, u,v)

    # the pause causes the figure to be displayed during the
    # simulation
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()

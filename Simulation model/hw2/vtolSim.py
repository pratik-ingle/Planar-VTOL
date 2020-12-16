import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import vtolParam as P
from signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter


# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
zvRef = signalGenerator(amplitude=0.5, frequency=0.1)
ztRef = signalGenerator(amplitude=0.5, frequency=0.1)
hRef = signalGenerator(amplitude=0.5, frequency=0.1)
thetaRef = signalGenerator(amplitude=.5*np.pi, frequency=0.5)
fRef = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = vtolAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # set variables
    r = reference.square(t)
    zt = ztRef.sin(t)
    zv = zvRef.sin(t)
    h = 1 + 2*hRef.sin(t)**2
    theta = thetaRef.square(t)
    f = fRef.sin(t)

    # update animation
    state = np.array([[zt], [zv], [h], [theta], [0.0], [0.0], [0.0], [0.0]])
    animation.update(state)
    dataPlot.update(t, r, state, f)

    # advance time by t_plot
    t = t + P.t_plot  
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()

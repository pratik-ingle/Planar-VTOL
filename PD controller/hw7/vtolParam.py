# Single link arm Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('/Users/pratik/Documents - pratik’s MacBook Pro/semester 5/Control Systems/VTOL/vtol')  # add parent directory
import hw3.vtolParam as P

Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
F_max = P.F_max  # limit on control signal

# PD gains
kp = 1.6
kd = 1.92

print('kp: ', kp)
print('kd: ', kd)




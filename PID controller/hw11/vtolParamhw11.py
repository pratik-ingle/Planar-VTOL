# vtol Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import vtolParam as P

# sample rate of the controller
Ts = P.Ts

# saturation limits
F_max = 30.0                # Max Force, N
theta_max = np.pi/6                 # Max theta, rad
error_max = 1        	

####################################################
#                 State Space
####################################################
# tuning parameters
tr_z = 8        # rise time for latral
tr_theta = 0.8    # rise time for angle
tr_h     = 8      # rise time for longitudinal
zeta_z   = 0.707  # damping ratio latral
zeta_th  = 0.707  # damping ratio angle
zeta_h   = 0.707  # damping ratio longitudinal


# State Space Equations   
# xdot = A*x + B*u
# y = C*x

#----------------------------- latral ---------------------------------------
A_z = np.array([[0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, -P.g, -(P.b/(P.mc+ 2*P.mr)), 0.0],
               [0.0, 0.0, 0.0, 0.0]])
B_z = np.array([[0.0],
               [0.0],
               [0.0],
               [1/(P.mc+ 2*P.mr)]])

C_z = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])


#----------------------------- longitudinal ---------------------------------------
A_h = np.array([[0.0, 1.0],
               [0.0, 0.0]])
B_h = np.array([[0.0],
               [1/(P.mc+ 2*P.mr)]])

C_h = np.array([1.0, 0.0])



# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for latral
wn_h = 2.2/tr_h  # natural frequency for longitudinal

#-----------------------------latral-----------------------------------------

des_char_poly_z = np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                            [1, 2*zeta_th*wn_th, wn_th**2])
des_poles_z = np.roots(des_char_poly_z)

# Compute the gains if the system is controllable for latral
if np.linalg.matrix_rank(cnt.ctrb(A_z, B_z)) != 4:
    print("The system is not controllable")
else:
    K_z = cnt.acker(A_z, B_z, des_poles_z)
    Cr_z = np.array([[1.0, 0.0, 0.0, 0.0]])
    kr_z = -1.0/(Cr_z*np.linalg.inv(A_z-B_z@K_z)@B_z)

print('K_z: ', K_z)
print('kr_z: ', kr_z)

#----------------------------longi----------------------------------------
des_char_poly_h = [1, 2*zeta_h*wn_h, wn_h**2]                         
des_poles_h = np.roots(des_char_poly_h)


# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A_h, B_h)) != 2:
    print("The system is not controllable")
else:
    K_h = cnt.acker(A_h, B_h, des_poles_h)
    Cr_h = np.array([[1.0, 0.0]])
    kr_h = -1.0/(Cr_h*np.linalg.inv(A_h-B_h@K_h)@B_h)

print('K_h: ', K_h)
print('kr_h: ', kr_h)

#----------------------------------------------



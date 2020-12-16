# Planar-VTOL
control moduls for vertical take-off and landing 

1. Simulation module 
2. PD/PID Controller
3. Full State Feedback 
4. Observers / Disturbunce

Problem: Planar VTOL



In this design study we will explore the control design for a simplified planar version of a quadrotor following a ground target. In particular, we will constrain the dynamics to be in two dimension plane comprising vertical and one dimension of horizontal, as shown in Fig. The planar vertical take-off and landing (VTOL) system is comprised of a center pod of mass mc and inertia Jc, a right motor/rotor that is modeled as a point mass mr that exerts a force fr at a distance d from motor/rotor that is modeled as a point mass ml that exerts a force fl at a distance −d from the center of mass. The position of the center of mass of the planar VTOL system is given by horizon- tal position zv and altitude h. The airflow through the rotor creates a change in the direction of flow of air and causes what is called “momentum drag.” Momentum drag can be modeled as a viscous drag force that is proportional to the horizontal velocity z ̇v. In other words, the drag force is Fdrag = −μz ̇v. The target on the ground will be modeled as an object with position zt and altitude h = 0. We will not explicitly model the dynamics of the target.

Use the following physical parameters: mc = 1 kg, Jc = 0.0042 kg m2, mr =0.25kg,ml =0.25kg,d=0.3m,μ=0.1kg/s,g=9.81m/s2.


Ref: http://controlbook.byu.edu/doku.php 

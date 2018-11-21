#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Quadrotor control simulation

#====================================================================================================
#System simulation with graphic results
#====================================================================================================

import dynamics
import control
import math
import matplotlib.pyplot as plt
import numpy as np

#====================================================================================================
#System definition
#====================================================================================================

#System conditions
simulationTime = 50
t = 0.01

#Control constants
#Altitude
#P
kp1=3
#PID
kp2=6
ki2=0
kd2=2.5
#PID
kp3=0.75
ki3=1.5
kd3=0

#Pitch
#P
kp1_theta=4.5
#PID
kp2_theta=0.15
ki2_theta=0.1
kd2_theta=0.004

#Roll
#P
kp1_phi=4.5
#PID
kp2_phi=0.15
ki2_phi=0.1
kd2_phi=0.004

#Yaw
#P
kp1_psi=10
#PID
kp2_psi=0.2
ki2_psi=0.02
kd2_psi=0

#Control
imax1=10
t_max1=200
imax2=10
t_max2=200

#Constrains
rate_imax = 50
ac_imax = 40
t_min = 800
t_max = 1500

#Goals
target_z=1
target_theta=0
target_phi=0
target_psi=0

#====================================================================================================
#Initial Conditions
#====================================================================================================

#Throttles
w=[1000, 1000, 1000, 1000]
#Physics
#ev = [0, 0, 1, 0, 0, 0, 0, 0, -9.81, 0.4, -0.2, 0.5, 0.1, -0.1, 0.2, 0.01, -0.01, 0.02]
ev = [0, 0, 0, 0, 0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0]

#Variables to graph
#Linear
pos= []
vel = []
ac = []
z_target = []
#Angular
angT = []
angPh = []
angPs = []
theta_target = []
phi_target = []
psi_target = []
#Motors
throttle1 = []
throttle2 = []
throttle3 = []
throttle4 = []

#Create controllers
altitudeControler = control.controlAlt (t, kp1, kp2, ki2, kd2, kp3, ki3, kd3, rate_imax, ac_imax,t_min,t_max)
thetaControler = control.controlAngle (t, kp1_theta, kp2_theta, ki2_theta, kd2_theta, imax1, t_max1)
phiControler = control.controlAngle (t, kp1_phi, kp2_phi, ki2_phi, kd2_phi, imax1, t_max1)
psiControler = control.controlAngle (t, kp1_psi, kp2_psi, ki2_psi, kd2_psi, imax2, t_max2)

#====================================================================================================
#Main loop
#====================================================================================================

for i in range(int(simulationTime/t)):
	#System iteration
	ev = dynamics.sDynamics(w, ev, t)
	x = ev[0]
	y = ev[1]
	z = ev[2]
	xD = ev[3]
	yD = ev[4]
	zD = ev[5]
	xDD = ev[6]
	yDD = ev[7]
	zDD = ev[8]
	theta = ev[9]
	phi = ev[10]
	psi = ev[11]
	thetaD = ev[12]
	phiD = ev[13]
	psiD = ev[14]
	thetaDD = ev[15]
	phiDD = ev[16]
	psiDD = ev[17]
	
	#Apply controllers
	control1 = altitudeControler.iterate(target_z, z, zD, zDD)
	control2 = phiControler.iterate(target_phi, phi, phiD)
	control3 = thetaControler.iterate(target_theta, theta, thetaD)
	control4 = psiControler.iterate(target_psi, psi, psiD)

	thr1 = control1
	thr2 = control2
	thr3 = control3
	thr4 = control4
	
	#Compute the throttle for each motor
	w1=thr1-thr3-thr4
	w2=thr1-thr2+thr4
	w3=thr1+thr3-thr4
	w4=thr1+thr2+thr4
	
	#Redefine throttle and state vectors
	w=[w1,w2,w3,w4]
	ev = [x, y, z, xD, yD, zD, xDD, yDD, zDD, theta, phi ,psi, thetaD, phiD, psiD, thetaDD, phiDD, psiDD]
	
	#Store the values
	pos.append(z)
	vel.append(zD)
	ac.append(zDD)
	z_target.append(target_z)
	angT.append(theta)
	angPh.append(phi)
	angPs.append(psi)
	theta_target.append(target_theta)
	phi_target.append(target_phi)
	psi_target.append(target_psi)
	throttle1.append(thr1)
	throttle2.append(thr2)
	throttle3.append(thr3)
	throttle4.append(thr4)

#====================================================================================================
#Graphics
#====================================================================================================

#Create time array
sTime = np.arange(0, simulationTime, t)	
#Linear - z axis
plt.plot(sTime, ac, label="ac")
plt.plot(sTime, pos, label="pos")
plt.plot(sTime, vel, label="vel")
plt.xlabel('Time(s)')
plt.ylabel('altitude')
plt.title('Quadrotor altitude control')
plt.grid(True)
leg = plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
leg.get_frame().set_alpha(0.5)
plt.show()
#z position and target
plt.plot(sTime, pos, label="pos")
plt.plot(sTime, z_target, label="target")
plt.xlabel('Time(s)')
plt.ylabel('altitude')
plt.title('Quadrotor altitude path')
plt.grid(True)
leg = plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
leg.get_frame().set_alpha(0.5)
plt.show()
#Angular - 3 axis
plt.plot(sTime, theta_target, label="target")
plt.plot(sTime, angT, label="theta")
plt.plot(sTime, angPh, label="phi")
plt.plot(sTime, angPs, label="psi")
plt.xlabel('Time(s)')
plt.ylabel('Radians')
plt.title('Angles')
plt.grid(True)
leg = plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
leg.get_frame().set_alpha(0.5)
plt.show()
#Throttles
plt.plot(sTime, throttle1, label="thr1")
plt.plot(sTime, throttle2, label="thr2")
plt.plot(sTime, throttle3, label="thr3")
plt.plot(sTime, throttle4, label="thr4")
plt.xlabel('Time(s)')
plt.ylabel('RPM')
plt.title('Throttles')
plt.grid(True)
leg = plt.legend(loc='best', ncol=2, mode="expand", shadow=True, fancybox=True)
leg.get_frame().set_alpha(0.5)
plt.show()
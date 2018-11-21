#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Quadrotor control simulation

#====================================================================================================
#System dynamics model
#====================================================================================================

import math

#Parameters that describe the drone
g=9.81
m=0.5
Ix=0.943*math.pow(10,-3)
Iy=Ix
Iz=0.546*math.pow(10,-3)
rho=1.275
Ct=1.752*math.pow(10,-6)
Cd=Ct/10
d=0.125
Umax=30000
l=0.15

#Air conditions
kt=rho*Ct/2
kd=rho*Cd/2

#Function to compute the next pose
def sDynamics(w, ev, t):
	"""
	Construct for a new altitude controller

	:param w: vector with the speed of the four motors
	:param ev: state vector
	:param t: time between two samples
	:return: returns the next state vector
	"""

	#Extract all the system information
	w1=w[0]
	w2=w[1]
	w3=w[2]
	w4=w[3]

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

	#Control variables
	U1=kt*(w1*w1+w2*w2+w3*w3+w4*w4)
	U2=kt*(w4*w4-w2*w2)
	U3=kt*(w3*w3-w1*w1)
	U4=kd*(-w1*w1+w2*w2-w3*w3+w4*w4)

	#Linear movement equations
	xDD=(math.cos(psi)*math.sin(theta)*math.cos(theta)+math.sin(psi)*math.sin(theta))*U1/m
	yDD=(math.sin(psi)*math.sin(phi)*math.cos(phi)-math.cos(psi)*math.sin(phi))*U1/m
	zDD=(math.cos(phi)*math.cos(theta))*U1/m-g

	#Angular movement equations
	phiDD=(l*U2+thetaD*psiD*(Iy-Iz))/Ix
	thetaDD=(l*U3+phiD*psiD*(Iz-Ix))/Iy
	psiDD=(U4+phiD*thetaD*(Ix-Iy))/Iz

	#Calculate actual position
	xD = xD + xDD * t
	yD = yD + yDD * t
	zD = zD + zDD * t 
	x = x + xD * t
	y = y + yD * t
	z = z + zD * t
	thetaD = thetaD + thetaDD *t
	phiD = phiD + phiDD * t
	psiD = psiD + psiDD *t
	theta = theta + thetaD *t
	phi = phi + phiD * t
	psi = psi + psiD *t

	#Ground constraint
	if (z < 0 and zD < 0):
		z=0
		zD=0

	ev2 = [x, y, z, xD, yD, zD, xDD, yDD, zDD, theta, phi ,psi, thetaD, phiD, psiD, thetaDD, phiDD, psiDD]
	
	return ev2

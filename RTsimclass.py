#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Quadrotor control simulation

#====================================================================================================
#System simulation joining the system with the controllers
#====================================================================================================

import dynamics
import control
import math
import numpy as np

#====================================================================================================
#Class to define system and controllers
#====================================================================================================

class QuadSim:
	def __init__(self, t=0.01):
		
		"""
		Construct for a new simulation

		:param t: time between two samples
		:return: returns nothing
		"""

		self.t = float(t)
		
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
		t_max2=120
		
		#Constrains
		rate_imax = 50
		ac_imax = 40
		t_min = 800
		t_max = 1500
		
		#Goals
		self.target_z=3
		self.target_theta=0
		self.target_phi=0
		self.target_psi=0

		#Initial Conditions
		self.w=[1000, 1000, 1000, 1000]
		#self.ev = [0, 0, 1, 0, 0, 0, 0, 0, -9.81, 0.4, -0.2, 1, 0.1, -0.1, 0.2, 0.01, -0.01, 0.02]
		self.ev = [0, 0, 0, 0, 0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		#Create controllers
		self.altitudeController = control.controlAlt (self.t, kp1, kp2, ki2, kd2, kp3, ki3, kd3, rate_imax, ac_imax,t_min,t_max)
		self.thetaController = control.controlAngle (self.t, kp1_theta, kp2_theta, ki2_theta, kd2_theta, imax1, t_max1)
		self.phiController = control.controlAngle (self.t, kp1_phi, kp2_phi, ki2_phi, kd2_phi, imax1, t_max1)
		self.psiController = control.controlAngle (self.t, kp1_psi, kp2_psi, ki2_psi, kd2_psi, imax2, t_max2)

	def run(self, dist_theta, dist_phi, dist_psi):

		"""
		Function to run the controllers once. It has to be called inside a loop

		:param dist_theta: disturbance in theta as angular velocity 
		:param dist_phi: disturbance in phi as angular velocity
		:param dist_psi: disturbance in psi as angular velocity
		:return: returns pose
		"""
		#Extract all the system information
		ev = dynamics.sDynamics(self.w, self.ev, self.t)
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
		thetaD = ev[12] + dist_theta
		phiD = ev[13] + dist_phi
		psiD = ev[14] + dist_psi
		thetaDD = ev[15]
		phiDD =ev[16]
		psiDD = ev[17]
		
		#Apply controllers
		control1 = self.altitudeController.iterate(self.target_z, z, zD, zDD)
		control2 = self.phiController.iterate(self.target_phi, phi, phiD)
		control3 = self.thetaController.iterate(self.target_theta, theta, thetaD)
		control4 = self.psiController.iterate(self.target_psi, psi, psiD)

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
		self.w = [w1,w2,w3,w4]
		self.ev = [x, y, z, xD, yD, zD, xDD, yDD, zDD, theta, phi, psi, thetaD, phiD, psiD, thetaDD, phiDD, psiDD]

		return [x, y, z, theta, phi, psi]
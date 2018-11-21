#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Quadrotor control simulation
#Script with 2 controler classes

#====================================================================================================
#Class to control the altitude
#====================================================================================================

class controlAlt:
	
	#Get all the necesary system information
	#Be sure to keep the same paramater structure when calling the function
	def __init__(self, t, kp1, kp2, ki2, kd2, kp3, ki3, kd3, rate_imax, ac_imax,t_min,t_max):
		"""
		Construct for a new altitude controller

		:param t: time between two samples
		:param kp1: proportional constant for linear P
		:param kp2: proportional constant for velocity PID
		:param ki2: integral constant for velocity PID
		:param kd2: derivative constant for velocity PID
		:param kp3: proportional constant for acceleration PID
		:param ki3: integral constant for acceleration PID
		:param kd3: derivative constant for acceleration PID
		:param rate_imax: velocity constraint
		:param ac_imax: acceleration constraint
		:param t_min: minimum throttle
		:param t_max: maximum throttle
		:return: returns nothing
		"""

		#System conditions
		self.t = t
		
		#Control constants
		self.kp1=kp1

		self.kp2=kp2
		self.ki2=ki2
		self.kd2=kd2

		self.kp3=kp3
		self.ki3=ki3
		self.kd3=kd3

		#Controler constants
		self.error_int_zD = 0
		self.error_int_zDD = 0
		self.error_past_zD = 0
		self.error_past_zDD = 0
		self.flag=1
		self.c2=0

		#Constrains
		self.rate_imax = rate_imax
		self.ac_imax = ac_imax
		self.t_min = t_min
		self.t_max = t_max

	def iterate(self, target_z, z, zD, zDD):
		"""
		Function to run the controller once. It has to be called inside a loop

		:param target_z: target altitude
		:param z: position
		:param zD: velocity
		:param zDD: acceleration
		:return: returns the throttle to reach the target
		"""

		#Gravity
		g = 9.81

		#This part of the code has to be runned once every 2 times
		if (self.flag == 1):
			#Compute error
			error_z = target_z - z

			#Proportional altitude control
			c1=self.kp1 * error_z

			if c1 > self.rate_imax:
				c1 = self.rate_imax
			else:
				if c1 < -self.rate_imax:
					c1 = -self.rate_imax
			
			#PID rate control
			error_zD = c1 - zD
			cp2 = self.kp2 *error_zD
			self.error_int_zD = self.error_int_zD + error_zD*2*self.t
			ci2 = self.ki2 * self.error_int_zD
			cd2 = self.kd2 *(error_zD - self.error_past_zD)/(2*self.t)

			#Limit the max rate
			if (ci2 > self.rate_imax):
				ci2=self.rate_imax
			else:
				if (ci2 < -self.rate_imax):
					ci2=-self.rate_imax

			#Sum the three parts of the PID
			self.c2 = cp2 + ci2 + cd2
			#Store the last error for the next derivative
			self.error_past_zD = error_zD

			#Limit from -1g to 1.5g
			if self.c2 > 1.5*g:
				self.c2=1.5*g
			else:
				if self.c2 < -1.0*g:
					self.c2=-1.0*g
			self.flag=0
		else:
			self.flag=1

		#PID acceleration control
		error_zDD = self.c2 - zDD
		cp3 = self.kp3 *error_zDD
		self.error_int_zDD = self.error_int_zDD + error_zDD*self.t
		ci3 = self.ki3 * self.error_int_zDD
		cd3 = self.kd3 *(error_zDD - self.error_past_zDD)/self.t

		#Limit the acceleration
		if (ci3 > self.ac_imax):
			ci3=self.ac_imax
		else:
			if (ci3 < -self.ac_imax):
				ci3=0
		#Sum the three parts of the PID
		c3 = cp3 + ci3 + cd3
		#Store the last error for the next derivative
		self.error_past_zDD = error_zDD

		#Limit throttle
		if (c3>self.ac_imax):
			thr=self.t_max
		else:
			if (c3<0):
				thr=self.t_min
			else:
				thr=self.t_min+c3*(self.t_max-self.t_min)/self.ac_imax

		return thr

#====================================================================================================
#Class to control the pose
#====================================================================================================

class controlAngle:
	
	#Get all the necesary system information
	#Be sure to keep the same paramater structure when calling the function
	def __init__(self, t, kp1, kp2, ki2, kd2, imax,t_max):
		"""
		Construct for a new altitude controller

		:param t: time between two samples
		:param kp1: proportional constant for linear P
		:param kp2: proportional constant for velocity PID
		:param ki2: integral constant for velocity PID
		:param kd2: derivative constant for velocity PID
		:param imax: velocity constraint
		:param t_max: maximum throttle
		:return: returns nothing
		"""

		#System conditions
		self.t = t
		
		#Control constants
		self.kp1=kp1

		self.kp2=kp2
		self.ki2=ki2
		self.kd2=kd2

		#Controler constants
		self.error_int=0
		self.error_past = 0

		#Constraints
		self.imax = imax
		self.t_max = t_max

	def iterate(self, target, angle, angleD):
		"""
		Function to run the controller once. It has to be called inside a loop

		:param target: target angle
		:param angle: position
		:param angleD: velocity
		:return: returns the throttle to reach the target
		"""

		#Compute error
		error = target - angle
		error = error % (2*3.1416)
		if error > 3.1416:
			error = -3.1416+(error-3.1416)
		else:
			if (error < -3.1416):
				error = 3.1416+(error+3.1416) 

		#Proportional angle control
		c1 = self.kp1 * error

		if c1 > self.imax:
			c1 = self.imax
		else:
			if c1 < -self.imax:
				c1 = -self.imax

		#PID control
		error_D = c1 - angleD

		cp = self.kp2 *error_D
		self.error_int = self.error_int + error_D*self.t
		ci = self.ki2 * self.error_int
		cd = self.kd2 *(error_D - self.error_past)/self.t

		#Limit the rate
		if (ci > self.imax):
			ci = self.imax
		else:
			if (ci < -self.imax):
				ci= -self.imax

		#Sum the three parts of the PID
		c = cp + ci + cd
		#Store the last error for the next derivative
		self.error_past = error_D

		#Limit throttle
		if (c > self.imax):
			thr=self.t_max
		else:
			if (c < -self.imax):
				thr=-self.t_max
			else:
				thr=c*self.t_max/self.imax

		return thr
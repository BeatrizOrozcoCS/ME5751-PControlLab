import numpy as np
import math

class E160_state:
	def __init__(self):
		self.x = .0
		self.y = .0
		self.theta = .0
		self.vix = .0 #velocity global frame x direction
		self.viy = .0 #velocity global frame y direction
		self.wi = .0 #angular velocity, global frame
		self.v = .0 #velocity, robot frame x direction
		self.w = .0 #angular velocity, robot frame
		self.phi_l = .0 #wheel speed, left
		self.phi_r = .0 #wheel speed, right

	def set_pos_state(self,x,y,theta):
		self.x = x
		self.y = y
		self.theta = theta

	def get_pos_state(self):
		return self.x, self.y, self.theta


	def get_global_vel_state(self):
		return self.vix, self.viy, self.wi


	def get_local_vel_state(self):
		return self.v, self.w

	# set the velocity in local frame, we have code to convert it to global frame, not sure the mode for
	def set_vel_state(self, v, w, deltaT = 0.1, mode = "S"):
		self.v = v
		self.w = w
		if (mode == 'S'):
			self._get_global_velocity()
			self.update_pos_state(deltaT)


	# set the velocity in 
	def set_wheel_speed(self, phi_l, phi_r):
		self.phi_l = phi_l
		self.phi_r = phi_r


	# transfer local velocity to global velocity
	def _get_global_velocity(self):
		# velocity transform matrix T_v
		T_v = np.array([[math.cos(self.theta),-math.sin(self.theta),0],[math.sin(self.theta),math.cos(self.theta),0],[0,0,1]])

		#local velocity vector
		v_v = np.array([[self.v],[0],[self.w]])

		veVec=np.dot(T_v,v_v)
		self.vix = veVec[0][0]
		self.viy = veVec[1][0]
		self.wi = self.w

	def update_pos_state(self, deltaT):
		self.x = self.x + deltaT * self.vix
		self.y = self.y + deltaT * self.viy
		self.theta = self.theta + deltaT * self.wi
		if(self.theta>math.pi):
			self.theta -= math.pi*2
		elif(self.theta<-math.pi):
			self.theta += math.pi*2

# Destination is stored in a list
class E160_des_state:
	def __init__(self):
		self.x = []
		self.y = []
		self.theta = []
		self.p = 0 #current goal in the list
		

	def reset_destination(self,x,y,theta):
		self.x = []
		self.y = []
		self.theta = []
		self.p = 0
		print("destinations has been reset")
		self.add_destination(x,y,theta)



	def add_destination(self,x,y,theta):
		self.x.append(x)
		self.y.append(y)
		self.theta.append(theta)
		print(str(x)+" "+str(y)+" "+str(theta)+"  is added to destination list")


	def get_des_size(self):
		return len(self.x)


	def get_des_state(self):
		return self.x[self.p], self.y[self.p], self.theta[self.p]

	def reach_destination(self):
		self.p += 1
		if(self.p < len(self.x)):
			return False #reached a midway destination
		else:
			return True #reached to the final destination
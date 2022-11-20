import numpy as np

class Path:
	def __init__(self):
		self.poses=[]

	#add a pose at the end of poses
	def add_pose(self,pose):
		self.poses.append(pose)

	#insert a pose in front of poses
	def insert_pose(self, pose):
		self.poses.insert(0,pose)

	def get_pose(self, pose_id):
		return self.poses[pose_id]

	def get_path_len(self):
		return len(self.poses)

	def print_path(self):
		for v in self.poses:
			print (v.map_i, v.map_j)

	def save_path(self,file_name="Log\path.txt"):
		f = open(file_name, 'a+')
		for v in self.poses:
		# edit this line to have data logging of the data you care about
			data = [str(v.map_i), str(v.map_j), str(v.theta)]
			f.write('\t'.join(data) + '\n')
		f.close()

	def clear_path(self):
		self.poses.clear()
	
#all pose are in map coordinate
class Pose:
	def __init__(self,map_i=0,map_j=0,theta=0):
		self.map_i = map_i
		self.map_j = map_j
		self.theta = theta

	def set_pose(self,map_i,map_j,theta):
		self.map_i = map_i
		self.map_j = map_j
		self.theta = theta

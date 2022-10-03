import cv2
import numpy as np
import math
from PIL import Image, ImageTk
from queue import Queue

class cost_map:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 # This value should be same as the pixel value of the image
		self.inflation_radius = 18 # radius of our robot is 18 pixel or cm
		self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2
		self.map_width = int(self.graphics.environment.width*self.graphics.scale)
		self.map_height = int(self.graphics.environment.height*self.graphics.scale)
		try:
			self.load_map(map = "maps/testmap2.png") #load map
		except:
			self.graphics.show_map_button.configure(state="disabled")
			print ("no map loaded") #if fail to find the map png
			return
		self.show_map()
		self.compute_costmap()

		# self.show_costmap()
		self.save_vis_map(map = "maps/testcostmap2.png")

	#load occupancy grid into self.map
	#self.map is a numpy 2d array
	#initialize self.costmap, a numpy 2d array, same as self.map
	def load_map(self,map="maps/testmap2.png"):
		self.map_img = Image.open(map).convert('L')
		self.map_img = self.map_img.resize((int(self.map_width),int(self.map_height)),Image.ANTIALIAS)
		# self.graphics.draw_map(map_img=self.map_img)
		self.map = cv2.imread(map,cv2.IMREAD_GRAYSCALE)
		print (self.map.dtype)
		print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
		self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
		self.vis_map=np.copy(self.map) #map for visualization
		self.distmap=np.copy(self.map).astype(np.float)
		self.costmap=np.copy(self.map).astype(np.float)

	#save your costmap into a grayscale image
	def save_vis_map(self,map="maps/costmap.png"):
		save_img = Image.fromarray(self.vis_map)
		save_img.save(map)

	def show_vis_map(self):
		self.get_vis_map()
		self.vis_map_img=Image.frombytes('L', (self.vis_map.shape[1],self.vis_map.shape[0]), self.vis_map.astype('b').tostring())
		self.graphics.draw_map(map_img=self.vis_map_img)

	#display costmap on the dialogue window
	def show_costmap(self):
		self.costmap_img=Image.frombytes('L', (self.costmap.shape[1],self.costmap.shape[0]), self.costmap.astype('b').tostring())
		self.graphics.draw_map(map_img=self.costmap_img)

	#display binary occupancy grid on the dialogue window 
	def show_map(self):
		self.graphics.draw_map(map_img=self.map_img)


	#This is the function you really should update!

	def compute_costmap(self):
		#The following is an example how you manipulate the value in self.costmap
		#It has nothing to do with brushfire, replace it
		#A whilte pixel has value of 255, a black one is 0
		#However, some block value may be dark gray
		self.costmap[200:400][0:-1]=128
		pass

	#scale costmap to 0 - 255 for visualization
	def get_vis_map(self):
		self.vis_map = np.uint8(255-self.costmap/4.0)
		np.savetxt("Log/vismap.txt",self.vis_map)

import cv2
import numpy as np
import math
from PIL import Image, ImageTk
from queue import Queue

class cost_map:
    def __init__(self,graphics):
        self.graphics = graphics
        #self.graphics.scale = 500 # This value should be same as the pixel value of the image
        self.inflation_radius = 18 # radius of our robot is 18 pixel or cm
        self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
        #self.graphics.environment.width/height = 2
        self.map_width = int(self.graphics.environment.width*self.graphics.scale)
        self.map_height = int(self.graphics.environment.height*self.graphics.scale)
        try:
            # self.load_map(map = "maps/ultimate test.png") #load map
            self.load_map(map = "maps/map_1.png") #load map
            # self.load_map(map = "maps/map_2.png") #load map
            # self.load_map(map = "maps/map_3.png") #load map
            # self.load_map(map = "maps/map_4.png") #load map
            #4 maps
            # office
            #roadmap
            #objectsintheway
            #tests
            # ultimate test


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
    def load_map(self,map="maps/testmap.png"):
        self.map_img = Image.open(map).convert('L')
        self.map_img = self.map_img.resize((int(self.map_width),int(self.map_height)),Image.ANTIALIAS)
        # self.graphics.draw_map(map_img=self.map_img)
        self.map = cv2.imread(map,cv2.IMREAD_GRAYSCALE)
        # print(self.map)
        # print (self.map.dtype)
        # print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
        self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
        self.vis_map=np.copy(self.map) #map for visualization
        self.distmap=np.copy(self.map).astype(np.float)
        self.costmap=np.copy(self.map).astype(np.float)

    #save your costmap into a grayscale image
    def save_vis_map(self,map="maps/costmap.png"):
        save_img = Image.fromarray(np.uint8(self.costmap))
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
        
        white = 255
        black = 0
        #However, some block value may be dark gray
        # print(type(self.costmap))
        grid = self.costmap
        numRows = np.size(grid,0)
        numCols = np.size(grid,1)
        
        visitedList = []
        copyList = []
        distanceList = []
        potentialList = []

        #print("numrows: ", numRows)
        #print("numCols: ", numCols)

        # create a visited?list, init it to false to confirm if a node has been vistied before
        # make a copy of the inputed list
        for i in range(0, numRows):
            currentItem_visitlist = []
            currentItem_list = []
            currentItem_distancelist = []
            for j in range(0, numCols):
                currentItem_visitlist.append(False)
                currentItem_list.append(grid[i][j])
                currentItem_distancelist.append(black)

            visitedList.append(currentItem_visitlist)
            copyList.append(currentItem_list)
            distanceList.append(currentItem_distancelist)
            potentialList.append(currentItem_distancelist)
        #create a list that will act as your queue
        queuelist = []
        #find all occupied spaces (1'sr) and append it to my queue
        for i in range(0,numRows):
            for j in range(0,numCols):
                position = []
                if copyList[i][j] == 0:
                    position.append(i)
                    position.append(j)
                    queuelist.append(position)
                    visitedList[i][j] = True
        
        numChildren = len(queuelist)
        distance = 1 
        #start breadth-first-search
        while (len(queuelist)>0):
            # asumptions seach one distance away (child) from beginning land points
            # after searching all the beining occupied spaces  distance grows by 1 once every level is finished and keep searching one distance away and continue until the end of queue
            # possible neighbors - one distance away
            # {
            #  (n,(n-1)) #left
            #  (n,(n+1)) #right
            #  (n-1),n)  #top
            #  (n+1),n)  #bottom
            # }
            # some connections may not be valid when n = 0 or if n = last column
            # ignore previosly visited cells
            # mark vistied cells
            # pop the queue
            # appnd the queue
            # repeat until done
            if numChildren == 0: #increase the distance when all parents are popped out
                numChildren = len(queuelist)
                distance +=1 #cost function
            #get the position value
            posx = queuelist[0][0]
            posy = queuelist[0][1]

            #pop the queue
            queuelist.pop(0)
            numChildren = numChildren -1 #subtract parent count
           
            if (posx-1) >= 0: #Left
                if visitedList[posx-1][posy] == False:
                    visitedList[posx-1][posy] = True 
                    queuelist.append([posx-1,posy])
                    distanceList[posx-1][posy] = distance
                    potentialList[posx-1][posy] = self.calculate_costmap(distance)
               

            if posx+1 < numRows: #right
                if visitedList[posx+1][posy] == False:
                    visitedList[posx+1][posy] = True
                    queuelist.append([posx+1,posy])
                    distanceList[posx+1][posy] = distance
                    potentialList[posx+1][posy] = self.calculate_costmap(distance)


            if posy-1 >= 0: #bottom
                if visitedList[posx][posy-1] == False:
                    visitedList[posx][posy-1] = True
                    queuelist.append([posx,posy-1])
                    distanceList[posx][posy-1] = distance
                    potentialList[posx][posy-1] = self.calculate_costmap(distance)


            if posy+1 < numCols: #top
                if visitedList[posx][posy+1] == False:
                    visitedList[posx][posy+1] = True
                    queuelist.append([posx,posy+1])
                    distanceList[posx][posy+1] = distance
                    potentialList[posx][posy+1] = self.calculate_costmap(distance)
   
        #np.savetxt("Log/distancemap.txt",np.array(distanceList)) 
        self.distmap = distanceList
        np.savetxt("Log/distancemap.txt",distanceList) #potential map for debugging
        self.costmap = potentialList
        np.savetxt("Log/potentialmap.txt",self.costmap) #potential map for debugging
        #print(self.costmap)#debugging
        
        pass    
    
 
    def calculate_costmap(self,distance):
        potential_cost = 0 #declare potential_cost varible
        bufferzone = 80 # 50 good for close quarters. 50 is risky as quack. Makes it go through small cracks.
        # 200 good for open spaces. 80 is a good meduium. Still kinda risky. Orienting car correctly at start is import to avoid over shoots.
        if distance > bufferzone:#onces its plenty away for desired bufferzone
            pEq2 = 255
            pEq1 = 255
        elif distance <= bufferzone/4.0:
            pEq1 = 0
        
        else:
            pEq2 = -math.exp(-distance/2+bufferzone)+200
            pEq1 = (-bufferzone)/math.pow(distance/4,2)+200
        
            
        potential_cost = pEq1 #change this to selected pEq1 or pEq2

        return potential_cost

    #scale costmap to 0 - 255 for visualization
    def get_vis_map(self):
        self.vis_map = np.uint8(self.costmap) #self.costmap#np.uint8(self.costmap)
        #print(self.vis_map)
        np.savetxt("Log/vismap.txt",self.vis_map)


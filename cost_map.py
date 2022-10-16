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
            self.load_map(map = "maps/objectsintheway.png") #load map
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
        print(self.map)
        print (self.map.dtype)
        print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
        self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
        self.vis_map=np.copy(self.map) #map for visualization
        self.distmap=np.copy(self.map).astype(np.float)
        self.costmap=np.copy(self.map).astype(np.float)

    #save your costmap into a grayscale image
    def save_vis_map(self,map="maps/testcostmap2.png"):
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
        
        white = 255
        black = 0
        #However, some block value may be dark gray
        print(type(self.costmap))
        grid = self.costmap
        numRows = np.size(grid,0)
        numCols = np.size(grid,1)
        
        visitedList = []
        copyList = []
        distanceList = []
        potentialList = []

        print("numrows: ", numRows)
        print("numCols: ", numCols)

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
        #find all land (1'sr) and append it to my queue
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
            # after searching all the beining landpoints  distance grows by 1 once every level is finished and keep searching one distance away and continue until the end of queue
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
            bufferzone = 0 #adds a bufferzone around the dark pixels
            
            if (posx-1) >= 0: #Left
                if visitedList[posx-1][posy] == False:
                    visitedList[posx-1][posy] = True 
                    queuelist.append([posx-1,posy])
                    distanceList[posx-1][posy] = distance
                    #potentialList[posx-1][posy] = math.exp(-distance+15)
                    #potentialList[posx-1][posy] = distance * 0.85

                if (posy+1) < numCols: 
                    if visitedList[posx-1][posy+1] == False: #top left
                        visitedList[posx-1][posy+1] = True 
                        queuelist.append([posx-1,posy+1])
                        distanceList[posx-1][posy+1] = distance+1 #diagonals needs the distnace increased by one
                        #potentialList[posx-1][posy+1] = math.exp(-distance+15)
                        #potentialList[posx-1][posy] = distance * 0.85
                if (posy-1) >= 0:
                    if visitedList[posx-1][posy-1] == False: # bottom left
                        visitedList[posx-1][posy-1] = True 
                        queuelist.append([posx-1,posy-1])
                        distanceList[posx-1][posy-1] = distance+1 #diagonals needs the distnace increased by one
                        #potentialList[posx-1][posy-1] = math.exp(-distance+15)
                        #potentialList[posx-1][posy] = distance * 0.85
               

            if posx+1 < numRows: #right
                if visitedList[posx+1][posy] == False:
                    visitedList[posx+1][posy] = True
                    queuelist.append([posx+1,posy])
                    distanceList[posx+1][posy] = distance
                    #potentialList[posx+1][posy] = math.exp(-distance+15)
                    #potentialList[posx+1][posy] = distance * 0.85
                if (posy+1) < numCols: 
                    if visitedList[posx+1][posy+1] == False: #top right
                        visitedList[posx+1][posy+1] = True 
                        queuelist.append([posx+1,posy+1])
                        distanceList[posx+1][posy+1] = distance+1 #diagonals needs the distnace increased by one
                        #potentialList[posx+1][posy+1] = math.exp(-distance+15)
                        #potentialList[posx-1][posy] = distance * 0.85
                if (posy-1) >= 0:
                    if visitedList[posx+1][posy-1] == False: # bottom right
                        visitedList[posx+1][posy-1] = True 
                        queuelist.append([posx+1,posy-1])
                        distanceList[posx+1][posy-1] = distance+1 #diagonals needs the distnace increased by one
                        #potentialList[posx-1][posy] = math.exp(-distance+15)
                        #potentialList[posx-1][posy] = distance * 0.85



            if posy-1 >= 0: #bottom
                if visitedList[posx][posy-1] == False:
                    visitedList[posx][posy-1] = True
                    queuelist.append([posx,posy-1])
                    distanceList[posx][posy-1] = distance
                    #potentialList[posx][posy-1] = math.exp(-distance+15)
                    #potentialList[posx][posy-1] = distance * 0.85

            if posy+1 < numCols: #top
                if visitedList[posx][posy+1] == False:
                    visitedList[posx][posy+1] = True
                    queuelist.append([posx,posy+1])
                    distanceList[posx][posy+1] = distance
                    #potentialList[posx][posy+1] = math.exp(-distance+15)
                    #potentialList[posx][posy+1] = distance * 0.85

        self.costmap = np.array(distanceList)
        np.savetxt("Log/distancemap.txt",self.costmap) #pure distance map to check
        print(self.costmap)
        pass


    #scale costmap to 0 - 255 for visualization
    def get_vis_map(self):
        self.vis_map = np.uint8(self.costmap)#np.uint8(255-self.costmap/4.0) 
        np.savetxt("Log/vismap.txt",self.vis_map)

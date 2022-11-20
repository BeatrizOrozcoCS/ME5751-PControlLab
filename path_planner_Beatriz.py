import queue
import time
import cv2
import numpy as np
import math
import warnings
from PIL import Image, ImageTk
from queue import PriorityQueue
from bsTree import *
from Path import *
import inspect
from PIL import Image, ImageTk
# from Queue import Queue


class path_planner:
    def __init__(self, graphics):
        self.graphics = graphics
        # self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
        # self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
        # self.graphics.environment.width/height = 2

        self.costmap = self.graphics.map
        self.map_width = self.costmap.map_width
        self.map_height = self.costmap.map_height

        self._init_path_img()
        self.path = Path()

        self.set_start(world_x=0, world_y=-100)
        self.set_goal(world_x=220.0, world_y=220.0, world_theta=.0)

        self.plan_path()
        self._show_path()

    def set_start(self, world_x=0, world_y=0, world_theta=0):
        self.start_state_map = Pose()
        map_i, map_j = self.world2map(world_x, world_y)
        print("Start with %d, %d on map" % (map_i, map_j))
        self.start_state_map.set_pose(map_i, map_j, world_theta)

    def set_goal(self, world_x, world_y, world_theta=0):
        self.goal_state_map = Pose()
        map_i, map_j = self.world2map(world_x, world_y)
        print("our new goal is %d, %d on map" % (map_i, map_j))
        self.goal_state_map.set_pose(map_i, map_j, world_theta)

    # convert a point a map to the actual world position
    def map2world(self, map_i, map_j):
        world_x = -self.graphics.environment.width / 2 * self.graphics.scale + map_j
        world_y = self.graphics.environment.height / 2 * self.graphics.scale - map_i
        return world_x, world_y

    # convert a point in world coordinate to map pixel
    def world2map(self, world_x, world_y):
        map_i = int(self.graphics.environment.width / 2 * self.graphics.scale - world_y)
        map_j = int(self.graphics.environment.height / 2 * self.graphics.scale + world_x)
        if (map_i < 0 or map_i >= self.map_width or map_j < 0 or map_j >= self.map_height):
            warnings.warn("Pose %f, %f outside the current map limit" % (world_x, world_y))

        if (map_i < 0):
            map_i = int(0)
        elif (map_i >= self.map_width):
            map_i = self.map_width - int(1)

        if (map_j < 0):
            map_j = int(0)
        elif (map_j >= self.map_height):
            map_j = self.map_height - int(1)

        return map_i, map_j

    def _init_path_img(self):
        self.map_img_np = 255 * np.ones((int(self.map_width), int(self.map_height), 4), dtype=np.int16)
        # self.map_img_np[0:-1][0:-1][3] = 0
        self.map_img_np[:, :, 3] = 0

    def _show_path(self):
        for pose in self.path.poses:
            map_i = pose.map_i
            map_j = pose.map_j
            self.map_img_np[map_i][map_j][1] = 0
            self.map_img_np[map_i][map_j][2] = 0
            self.map_img_np[map_i][map_j][3] = 255

        self.path_img = Image.frombytes('RGBA', (self.map_img_np.shape[1], self.map_img_np.shape[0]),
                                        self.map_img_np.astype('b').tostring())
        self.graphics.draw_path(self.path_img)

    # If you want to save the path as an image, un-comment the following line:
    # self.path_img.save('Log\path_img.png')

    # If you want to output an image of map and path, un-comment the following two lines
    # self.path_img = toimage(self.map_img_np)
    # self.path_img.show()

    def plan_path(self):
        # The major program you are going to write!
        # The following simply demo that how you can add pose to path
        self.path.clear_path()
        grid = np.copy(self.costmap.costmap)
        start_point = (self.start_state_map.map_i, self.start_state_map.map_j)
        end_point = (self.goal_state_map.map_i, self.goal_state_map.map_j)

        # points = bresenham(self.start_state_map.map_i, self.start_state_map.map_j, self.goal_state_map.map_i,
        #                    self.goal_state_map.map_j)
        bfsdistance = self.sp_to_gp_bfs(self.start_state_map.map_i,self.start_state_map.map_j,self.goal_state_map.map_i,self.goal_state_map.map_j)
        #print(bfsdistance)
        eucdialiandistance = self.gp_to_sp_bfs(self.start_state_map.map_i,self.start_state_map.map_j,self.goal_state_map.map_i,self.goal_state_map.map_j)


        points = self.aStar(start_point, end_point,bfsdistance,eucdialiandistance)

        for p in points:
            self.path.add_pose(Pose(map_i=p[0][0], map_j=p[0][1], theta=0))  # theta is wrong

        self.path.save_path(file_name="Log\path.csv")

    def sp_to_gp_bfs(self,x1,y1,x2,y2): #manhantann distance from start point to goal point
        #init varibles
        copylist = []
        queuelist = []
        distanceList = []
        visitedList = [] 

        grid = self.costmap.costmap
        #print("entering bfs")
        numRows = np.size(grid,0)
        #print(numRows)
        numCols = np.size(grid,1)
        #preload lists with default values
        for i in range(0, numRows):
            currentItem_visitlist = []
            currentItem_list = []
            currentItem_distancelist = []
            for j in range(0, numCols):
                currentItem_visitlist.append(False)
                currentItem_list.append(grid[i][j])
                currentItem_distancelist.append(0)
        

            visitedList.append(currentItem_visitlist)
            copylist.append(currentItem_list)
            distanceList.append(currentItem_distancelist)

        for i in range(0, numRows):
            for j in range(0, numCols):
                if grid[i][j] == 0:
                    visitedList[i][j] = True
 
        #append queue from starting position find the distance 
        queuelist.append([x1,y1])
        visitedList[x1][y1] = True
        visitedList[x2][y2] = True 

        #find all occupied pixels and set them to true as well

        numChildren = len(queuelist)
        distance = 1 
        #start breadth-first-search
        while (len(queuelist)>0):
            if numChildren == 0: #increase the distance when all parents are popped out
                numChildren = len(queuelist)
                distance +=1
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

            if posx+1 < numRows: #right
                if visitedList[posx+1][posy] == False:
                    visitedList[posx+1][posy] = True
                    queuelist.append([posx+1,posy])
                    distanceList[posx+1][posy] = distance

            if posy-1 >= 0:
                if visitedList[posx][posy-1] == False:
                    visitedList[posx][posy-1] = True
                    queuelist.append([posx,posy-1])
                    distanceList[posx][posy-1] = distance

            if posy+1 < numCols:
                if visitedList[posx][posy+1] == False:
                    visitedList[posx][posy+1] = True
                    queuelist.append([posx,posy+1])
                    distanceList[posx][posy+1] = distance

        np.savetxt("Log/distancemapfrompoint.txt",np.array(distanceList))     
        return distanceList

    def gp_to_sp_bfs(self,x1,y1,x2,y2): #educlain distance distance from goal point to start point
        #init varibles
        copylist = []
        queuelist = []
        distanceList = []
        visitedList = [] 

        grid = self.costmap.costmap
        #print("entering bfs")
        numRows = np.size(grid,0)
        #print(numRows)
        numCols = np.size(grid,1)
        #preload lists with default values
        for i in range(0, numRows):
            currentItem_visitlist = []
            currentItem_list = []
            currentItem_distancelist = []
            for j in range(0, numCols):
                currentItem_visitlist.append(False)
                currentItem_list.append(grid[i][j])
                currentItem_distancelist.append(0)
        

            visitedList.append(currentItem_visitlist)
            copylist.append(currentItem_list)
            distanceList.append(currentItem_distancelist)

        for i in range(0, numRows):
            for j in range(0, numCols):
                if grid[i][j] == 0:
                    visitedList[i][j] = True
 
        #append queue from starting position find the distance 
        queuelist.append([x2,y2])
        visitedList[x1][y1] = True
        visitedList[x2][y2] = True 

        #find all occupied pixels and set them to true as well

        numChildren = len(queuelist)
        distance = 1 
        #start breadth-first-search
        while (len(queuelist)>0):
            if numChildren == 0: #increase the distance when all parents are popped out
                numChildren = len(queuelist)
                distance +=1
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
                    distanceList[posx-1][posy] = distance*1.4

                if (posy+1) < numCols: 
                    if visitedList[posx-1][posy+1] == False: #top left
                        visitedList[posx-1][posy+1] = True 
                        queuelist.append([posx-1,posy+1])
                        distanceList[posx-1][posy+1] = distance*1.4 #diagonals needs the distnace increased by one
    
                if (posy-1) >= 0:
                    if visitedList[posx-1][posy-1] == False: # bottom left
                        visitedList[posx-1][posy-1] = True 
                        queuelist.append([posx-1,posy-1])
                        distanceList[posx-1][posy-1] = distance*1.4 #diagonals needs the distnace increased by one
                        
            if posx+1 < numRows: #right
                if visitedList[posx+1][posy] == False:
                    visitedList[posx+1][posy] = True
                    queuelist.append([posx+1,posy])
                    distanceList[posx+1][posy] = distance*1.4
                   

                if (posy+1) < numCols: 
                    if visitedList[posx+1][posy+1] == False: #top right
                        visitedList[posx+1][posy+1] = True 
                        queuelist.append([posx+1,posy+1])
                        distanceList[posx+1][posy+1] = distance*1.4 #diagonals needs the distnace increased by one

                if (posy-1) >= 0:
                    if visitedList[posx+1][posy-1] == False: # bottom right
                        visitedList[posx+1][posy-1] = True 
                        queuelist.append([posx+1,posy-1])
                        distanceList[posx+1][posy-1] = distance*1.4 #diagonals needs the distnace increased by one
                        
            if posy-1 >= 0: #bottom
                if visitedList[posx][posy-1] == False:
                    visitedList[posx][posy-1] = True
                    queuelist.append([posx,posy-1])
                    distanceList[posx][posy-1] = distance*1.4

            if posy+1 < numCols: #top
                if visitedList[posx][posy+1] == False:
                    visitedList[posx][posy+1] = True
                    queuelist.append([posx,posy+1])
                    distanceList[posx][posy+1] = distance*1.4


        np.savetxt("Log/distancemapgptosp.txt",np.array(distanceList))     
        return distanceList


    def aStar(self,start_point,end_point, distance_man_s2g, distance_euc_g2s):

        #init varibles
        points = [] #path to the goal
        pointsset = [] #all paths 

        openlist = [start_point] #unvisted points
        closedlist = [] #vistied points

        pathlen = {}
        pathlen[start_point] = 0

        parentnode = {} 
        parentnode[start_point] = start_point


        #varibles to start aStar
        grid = self.costmap.costmap
        overallcostmap = np.array(distance_man_s2g)  + np.array(distance_euc_g2s) * (4-(np.array(grid)/255)/3)

        while len(openlist)>0:
            node = None
            for n in openlist:
                if node == None or overallcostmap[n[0]][n[1]] < overallcostmap[node[0]][node[1]]:
                    node = n

            if node == node: #no other path exist
                break
            if node in end_point:
                finalvalue =  overallcostmap[node[0]][node[1]]
                recreate_path = []

                next = node
                while parentnode[next] != next:
                    recreate_path.append(next)
                    next = parentnode[next]
                recreate_path.append(start_point)
                recreate_path.reverse()

                pointsset.append(recreate_path,finalvalue)
                openlist.remove(node)
                closedlist.append(node)
                continue
            path_cost = overallcostmap[node]
            #for m in range()

        #init varibles to return
        counter = 0
        points =[]
        st = time.time() #gett

        
        
        #                                                                                                                                   
        queuelist = [] #
        visitedList = [] 
        childrenList=[]
        grid = self.costmap.costmap
        numRows = np.size(grid,0)
        numCols = np.size(grid,1)

        #preload lists with default values
        for i in range(0, numRows):
            currentItem_visitlist = []
            for j in range(0, numCols):
                currentItem_visitlist.append(False)
 
            visitedList.append(currentItem_visitlist)

        for i in range(0, numRows): #this is so it does not queue pixels that are obstacles
            for j in range(0, numCols):
                if grid[i][j] == 0:
                    visitedList[i][j] = True 
 
        #append queue from starting position find the distance 
        queuelist.append(start_point)
        visitedList[start_point[0]][start_point[1]] = True 

        #check if given point is inside at an obstacle
        if visitedList[end_point[0]][end_point[1]] == True:
            print("unable to go to point, point is at an obstacle")
            return points

        #find all occupied pixels and set them to true as well
        #numChildren = len(queuelist)
        #start astar
        print("starting aStar")

        neighbors = [(-1,0),(-1,1),(-1,-1),(1,0), (1,1), (1,-1), (0,1),(0,-1)]

        while (len(queuelist)>0):
            #if numChildren == 0: #increase the distance when all parents are popped out
            #    numChildren = len(queuelist)
            #    distance +=1
            localchildrenList = []
            costList = []
            #get the position value
            posx = queuelist[0][0]
            posy = queuelist[0][1]
            print(posx,posy)
            #pop the queue
            queuelist.pop(0)
            #numChildren = numChildren -1 #subtract parent count


            # search neighbors
            localcostList = [pow(10,9),pow(10,9),pow(10,9),pow(10,9),pow(10,9),pow(10,9),pow(10,9),pow(10,9)]

            if (posx-1) >= 0: #Left
                if visitedList[posx-1][posy] == False:
                    localchildrenList.append([posx-1,posy])
                    localcostList[0] = overallcostmap[posx-1][posy]

            if (posx-1) >= 0 and (posy+1) < numCols: 
                if visitedList[posx-1][posy+1] == False: #top left
                    localchildrenList.append([posx-1,posy+1])
                    localcostList[1] = overallcostmap[posx-1][posy+1]

            if (posx-1) >= 0 and (posy-1) >= 0:
                if visitedList[posx-1][posy-1] == False: # bottom left
                    visitedList[posx-1][posy-1] == True
                    localchildrenList.append([posx-1,posy-1])
                    localcostList[2] = overallcostmap[posx-1][posy-1]
            
            if posx+1 < numRows: #right
                if visitedList[posx+1][posy] == False:
                    visitedList[posx+1][posy] == True
                    localchildrenList.append([posx+1,posy])
                    localcostList[3] = overallcostmap[posx+1][posy]

            if posx+1 < numRows and (posy+1) < numCols: 
                if visitedList[posx+1][posy+1] == False: #top right
                    visitedList[posx+1][posy+1] == True
                    localchildrenList.append([posx+1,posy+1])
                    localcostList[4] = overallcostmap[posx+1][posy+1]
                    
            if posx+1 < numRows and (posy-1) >= 0:
                if visitedList[posx+1][posy-1] == False: # bottom right
                    visitedList[posx+1][posy-1] == True
                    localchildrenList.append([posx+1,posy-1])
                    localcostList[5] = overallcostmap[posx+1][posy-1] 

            if posy-1 >= 0: #bottom
                if visitedList[posx][posy-1] == False:
                    visitedList[posx][posy-1] == True
                    localchildrenList.append([posx,posy-1])
                    localcostList[6] = overallcostmap[posx][posy-1]

            if posy+1 < numCols: #top
                if visitedList[posx][posy+1] == False:
                    visitedList[posx][posy+1] = True
                    localchildrenList.append([posx,posy+1])
                    localcostList[7] = overallcostmap[posx][posy+1]

            #check if endpoint is within the neightbors
            if end_point in localchildrenList:
                et = time.time()
                print("Time Elasped: ",et-st)
                print("Counter: ", counter) 
                return points

            #after searching all the nieghbors append the queue 
            minvalueIndex = localcostList.index(min(localcostList))
            print(minvalueIndex)
            print(localcostList)
            print(localchildrenList[minvalueIndex])
            queuelist.append(localchildrenList[minvalueIndex])


            visitedList[localchildrenList[minvalueIndex][0]][localchildrenList[minvalueIndex][1]]== True
            #print([localchildrenList[minvalueIndex][0]][localchildrenList[minvalueIndex][1]])
            points.append(queuelist[0])
            counter = counter +1 
            #print(counter)




def bresenham(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end

    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions

    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    # print points
    print(points)
    return points
import os
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
import cubic_spline_planner


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

        # robot stuff
        self.controller = self.graphics.environment.robots[0].controller
        self.robot = self.graphics.environment.robots[0]

        # Path Planner - Robot Orientation is changed in E160_robot line 11
        self.set_start(world_x=.0, world_y=.0)
        # Map 1 Goals
        # self.set_goal(world_x = 200.0, world_y = .0, world_theta = .0) # Map 1 Des 1
        self.set_goal(world_x=-200.0, world_y=-142.0, world_theta=.0)  # Map 1 Des 2

        # Map 2 Goal
        # self.set_goal(world_x = -210.0, world_y = 205.0, world_theta = .0) # Map 1 Des 1
        # self.set_goal(world_x=222.0, world_y=-165.0, world_theta=.0)  # Map 1 Des 2

        # Map 3 Goals
        # self.set_goal(world_x = -124.0, world_y = -89.0, world_theta = .0) # Map 1 Des 1
        # self.set_goal(world_x=199.0, world_y=-180.0, world_theta=.0)  # Map 1 Des 2

        self.plan_path()
        self._show_path()
        # self.spline()

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
    #     self.path_img.save('Log\path_img.png')

    # If you want to output an image of map and path, un-comment the following two lines
    # self.path_img = Image(self.map_img_np)
    # self.path_img.show()

    def plan_path(self):
        # The major program you are going to write!
        # The following simply demo that how you can add pose to path
        self.path.clear_path()
        grid = np.copy(self.costmap.costmap)
        start_point = (self.start_state_map.map_i, self.start_state_map.map_j)
        end_point = (self.goal_state_map.map_i, self.goal_state_map.map_j)

        bfsdistance = self.sp_to_gp_bfs(self.start_state_map.map_i, self.start_state_map.map_j,
                                        self.goal_state_map.map_i,
                                        self.goal_state_map.map_j)  # (g) manhantann distance from start
        eucdialiandistance = self.gp_to_sp_bfs(self.start_state_map.map_i, self.start_state_map.map_j,
                                               self.goal_state_map.map_i,
                                               self.goal_state_map.map_j)  # (h) educdialin distance form end to start

        u = (4 - (np.array(grid) / 255) / 3)  # u depends on the bufferzone varible on path_planner.py

        overallcostmap = np.array(bfsdistance) + np.array(eucdialiandistance) * u * 5  # overall node cost map

        np.savetxt("Log/overallcostmap.txt", np.array(overallcostmap))

        points = astar(overallcostmap, start_point, end_point)

        if (points) == None:
            print("path not possible")
        else:
            for p in points:
                self.path.add_pose(Pose(map_i=p[0][0], map_j=p[0][1], theta=0))
            self.ref_path(points)
            # self._show_path()
            # self.path.save_path(file_name="Log\path.csv")

    def ref_path(self, points):
        x = []
        y = []
        # print(points)
        # print(points[-1][0][0])
        n = 5 # Discretization variable - lower => spline is exactly like path planner -
        # Higher => spline is smoother deviates more from planner
        # 5 is pretty close to planner, 50 still pretty close edges get smoothened gives more feasible path
        # 100 path deviates greatly from planner. Path is getting smoother. Verry feasible trajectory - less deviation with planner
        # 200 path deviates greatly from planner. Smoothest path so far. Verrry feasible trajector - easy for controller to track - not good for close quarters
        # 300 cuts edges way too much. No GO
        # 500 path basically goes straight to goal- NO GO
        for p in points[0:-1:n]:
            # print(p)
            xx, yy = self.map2world(p[0][0], p[0][1])
            x.append(xx)
            y.append(yy)

        # Add Goal point to list - May not be added during path discretization
        xx, yy = self.map2world(points[-1][0][0],points[-1][0][1])
        x.append(xx)
        y.append(yy)
        # print(x[-1],y[-1])

        # xx, yy = self.map2world(p[0][0], p[0][1])
        # x.append(xx)
        # y.append(yy)
        # Add final location - travel is dictated by spline
        self.robot.state_des.add_destination(x[-1], y[-1], 0)
        # print("path length: ", len(points))

        # Create spline to send to robot for MPC controller
        self.controller.create_spline(x, y)

        for p in range(0,len(self.controller.cx)):
            map_i, map_j = self.world2map(int(self.controller.cx[p]), int(self.controller.cy[p]))
            self.map_img_np[map_i][map_j][1] = 0
            self.map_img_np[map_i][map_j][2] = 255
            self.map_img_np[map_i][map_j][3] = 255

        self.path_img = Image.frombytes('RGBA', (self.map_img_np.shape[1], self.map_img_np.shape[0]),
                                        self.map_img_np.astype('b').tostring())
        self.graphics.draw_path(self.path_img)

        # print(self.controller.cy)
        # print(self.controller.cx)
        # print(self.controller.cyaw)

    def sp_to_gp_bfs(self, x1, y1, x2, y2):  # manhantann distance from start point to goal point
        # init varibles
        copylist = []
        queuelist = []
        distanceList = []
        visitedList = []

        grid = self.costmap.costmap
        # print("entering bfs")
        numRows = np.size(grid, 0)
        # print(numRows)
        numCols = np.size(grid, 1)
        # preload lists with default values
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

        # append queue from starting position find the distance
        queuelist.append([x1, y1])
        visitedList[x1][y1] = True
        visitedList[x2][y2] = True

        # find all occupied pixels and set them to true as well

        numChildren = len(queuelist)
        distance = 1
        # start breadth-first-search
        while (len(queuelist) > 0):
            if numChildren == 0:  # increase the distance when all parents are popped out
                numChildren = len(queuelist)
                distance += 1
            # get the position value
            posx = queuelist[0][0]
            posy = queuelist[0][1]

            # pop the queue
            queuelist.pop(0)
            numChildren = numChildren - 1  # subtract parent count
            if (posx - 1) >= 0:  # Left
                if visitedList[posx - 1][posy] == False:
                    visitedList[posx - 1][posy] = True
                    queuelist.append([posx - 1, posy])
                    distanceList[posx - 1][posy] = distance

            if posx + 1 < numRows:  # right
                if visitedList[posx + 1][posy] == False:
                    visitedList[posx + 1][posy] = True
                    queuelist.append([posx + 1, posy])
                    distanceList[posx + 1][posy] = distance

            if posy - 1 >= 0:
                if visitedList[posx][posy - 1] == False:
                    visitedList[posx][posy - 1] = True
                    queuelist.append([posx, posy - 1])
                    distanceList[posx][posy - 1] = distance

            if posy + 1 < numCols:
                if visitedList[posx][posy + 1] == False:
                    visitedList[posx][posy + 1] = True
                    queuelist.append([posx, posy + 1])
                    distanceList[posx][posy + 1] = distance

        np.savetxt("Log/distancemapfrompoint.txt", np.array(distanceList))
        return distanceList

    def gp_to_sp_bfs(self, x1, y1, x2, y2):  # educlain distance distance from goal point to start point
        # init varibles
        copylist = []
        queuelist = []
        distanceList = []
        visitedList = []

        grid = self.costmap.costmap
        # print("entering bfs")
        numRows = np.size(grid, 0)
        # print(numRows)
        numCols = np.size(grid, 1)
        # preload lists with default values
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

        # append queue from starting position find the distance
        queuelist.append([x2, y2])
        visitedList[x1][y1] = True
        visitedList[x2][y2] = True

        # find all occupied pixels and set them to true as well

        numChildren = len(queuelist)
        distance = 1
        # start breadth-first-search
        while (len(queuelist) > 0):
            if numChildren == 0:  # increase the distance when all parents are popped out
                numChildren = len(queuelist)
                distance += 1
            # get the position value
            posx = queuelist[0][0]
            posy = queuelist[0][1]

            # pop the queue
            queuelist.pop(0)
            numChildren = numChildren - 1  # subtract parent count

            if (posx - 1) >= 0:  # Left
                if visitedList[posx - 1][posy] == False:
                    visitedList[posx - 1][posy] = True
                    queuelist.append([posx - 1, posy])
                    distanceList[posx - 1][posy] = distance * 1.4

                if (posy + 1) < numCols:
                    if visitedList[posx - 1][posy + 1] == False:  # top left
                        visitedList[posx - 1][posy + 1] = True
                        queuelist.append([posx - 1, posy + 1])
                        distanceList[posx - 1][
                            posy + 1] = distance * 1.4  # diagonals needs the distnace increased by one

                if (posy - 1) >= 0:
                    if visitedList[posx - 1][posy - 1] == False:  # bottom left
                        visitedList[posx - 1][posy - 1] = True
                        queuelist.append([posx - 1, posy - 1])
                        distanceList[posx - 1][
                            posy - 1] = distance * 1.4  # diagonals needs the distnace increased by one

            if posx + 1 < numRows:  # right
                if visitedList[posx + 1][posy] == False:
                    visitedList[posx + 1][posy] = True
                    queuelist.append([posx + 1, posy])
                    distanceList[posx + 1][posy] = distance * 1.4

                if (posy + 1) < numCols:
                    if visitedList[posx + 1][posy + 1] == False:  # top right
                        visitedList[posx + 1][posy + 1] = True
                        queuelist.append([posx + 1, posy + 1])
                        distanceList[posx + 1][
                            posy + 1] = distance * 1.4  # diagonals needs the distnace increased by one

                if (posy - 1) >= 0:
                    if visitedList[posx + 1][posy - 1] == False:  # bottom right
                        visitedList[posx + 1][posy - 1] = True
                        queuelist.append([posx + 1, posy - 1])
                        distanceList[posx + 1][
                            posy - 1] = distance * 1.4  # diagonals needs the distnace increased by one

            if posy - 1 >= 0:  # bottom
                if visitedList[posx][posy - 1] == False:
                    visitedList[posx][posy - 1] = True
                    queuelist.append([posx, posy - 1])
                    distanceList[posx][posy - 1] = distance * 1.4

            if posy + 1 < numCols:  # top
                if visitedList[posx][posy + 1] == False:
                    visitedList[posx][posy + 1] = True
                    queuelist.append([posx, posy + 1])
                    distanceList[posx][posy + 1] = distance * 1.4

        np.savetxt("Log/distancemapgptosp.txt", np.array(distanceList))
        return distanceList


class Node:
    def __init__(self, parent=None, position=(None, None), ):
        # Initialize Relationships
        self.parent = parent
        # Initialize Positions
        self.position = position
        self.f = 0
        self.h = 0
        self.g = 0

    def f_cost(self):
        self.f = self.g + self.h

    def h_cost(self, end_node, grid):
        self.h = grid[end_node.position[0]][end_node.position[1]]

    def g_cost(self, start_node):
        if self.h == 0:  # if the h_node = 0 its an obstacle, make sure the cost is evalated
            self.g = 1000000
        else:
            self.g = 0


def astar(grid, start, end):  # Inputs are the NP array, start index, and end index
    st = time.time()
    # Initialize start and end nodes
    end_node = Node(None, end)
    start_node = Node(None, start)
    start_node.g = 0
    start_node.h_cost(end_node, grid)
    start_node.f_cost()

    # Initialize the Queues
    openQ = queue.PriorityQueue()
    openL = []
    closedL = []

    # input obstacles into closed L
    numRows = np.size(grid, 0)
    # print(numRows)
    numCols = np.size(grid, 1)
    for i in range(0, numRows):
        for j in range(0, numCols):
            if grid[i][j] == 0:
                closedL.append(Node(None, (i, j)))

    # Input starting node into open queue
    counter = 0
    openQ.put((start_node.f, counter, start_node))
    openL.append(start_node)

    # Initialize Neighbor search indexes
    neighbors = [(0, 1), (1, 0), (-1, 0), (0, -1)]

    try:
        while openQ.empty() is not True:
            # Pop Current Node with lowest F value
            currentN = openQ.get()[2]
            openL.remove(currentN)
            closedL.append(currentN)

            # Verify if we have reached our goal
            if currentN.position == end_node.position:
                plan = []
                while currentN is not None:
                    plan.append([currentN.position])
                    currentN = currentN.parent
                et = time.time()
                print("counter: ", counter)
                print("time elasped: ", et - st)
                planF = plan[::-1]

                return planF

            # Find neighbors
            children = []  # Initialize children list
            for neighbor in neighbors:
                search_loc = (currentN.position[0] + neighbor[0], currentN.position[1] + neighbor[1])

                # Ensure index is within bounds
                if (np.shape(grid)[0] - 1) < search_loc[0] < 0 or (np.shape(grid)[1] - 1) < search_loc[1] < 0:
                    # If out of bounds continue to nexrt neighbor
                    continue

                # Do not search walls
                # print(grid[search_loc])
                if abs(grid[search_loc]) < 5:  # seach if its neighboring tot he end point
                    if end_node.position[0] - search_loc[0] == 0 and end_node.position[1] - search_loc[1] == 0:
                        plan = []
                        while currentN is not None:
                            plan.append([currentN.position])
                            # print(currentN.position)
                            currentN = currentN.parent
                        et = time.time()
                        print("counter: ", counter)
                        print("time elasped: ", et - st)

                        planF = plan[::-1]

                        return planF
                    continue

                # Enlist Children

                childN = Node(currentN, search_loc)
                children.append(childN)

            # Search through children
            for child in children:
                # Check if child has been searched previously
                if len([searchedN for searchedN in closedL if searchedN == child]) > 0:
                    continue
                # If it hasnt been searched calculate costs

                child.g = currentN.g + 1
                child.h_cost(currentN, grid)
                child.f_cost()

                # Check if child is in open list
                if len([node for node in openL if
                        child.position == node.position and node.f >= child.f]) > 0:  # If node is in list and has more steps from start do not add

                    continue
                counter += 1
                # print(counter)
                # print(child.position)
                openQ.put((child.f, counter, child))
                openL.append(child)

    except KeyboardInterrupt:
        plan = []
        while currentN is not None:
            plan.append([currentN.position])
            currentN = currentN.parent
        et = time.time()
        print("counter: ", counter)
        print("time elasped: ", et - st)
        planF = plan[::-1]
        return planF


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
    return points

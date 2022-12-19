
import random
import time
from E160_environment import *
from E160_graphics import *

# MPC and Vehicle Variables
# NX = 4  # x = x, y, v, yaw
# NU = 2  # a = [accel, steer]
# T = 6  # horizon length
#
# # mpc parameters
# R = np.diag([0.01, 0.01])  # input cost matrix
# Rd = np.diag([0.01, 1.0])  # input difference cost matrix
# Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
# Qf = Q  # state final matrix
# GOAL_DIS = 1.5  # goal distance
# STOP_SPEED = 0.5 / 3.6  # stop speed
# MAX_TIME = 500.0  # max simulation time
#
# # iterative paramter
# MAX_ITER = 4  # Max iteration
# DU_TH = 0.1  # iteration finish param
#
# N_IND_SEARCH = 10  # Search index number
#
# DT = 0.2  # [s] time tick
#
# # Vehicle parameters
# LENGTH = 4.5  # [m]
# WIDTH = 2.0  # [m]
# BACKTOWHEEL = 1.0  # [m]
# WHEEL_LEN = 0.3  # [m]
# WHEEL_WIDTH = 0.2  # [m]
# TREAD = 0.7  # [m]
# WB = 2.5  # [m]
#
# MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
# MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
# MAX_SPEED = 20.0   # maximum speed [m/s]
# MIN_SPEED = -20.0   # minimum speed [m/s]
# MAX_ACCEL = 5.0  # maximum accel [m/ss]
# TARGET_SPEED = 15  # [m/s] target speed

def main():  
    
    # set time step size in seconds
    deltaT = 0.1

    # instantiate robot navigation classes
    environment = E160_environment(deltaT)
    graphics = E160_graphics(environment)
    
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break
        
        # update robots
        environment.update_robots(deltaT)
        
        # log all the robot data
        environment.log_data()
    
        # maintain timing
        time.sleep(deltaT)
            
main()

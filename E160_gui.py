
import random
import time
from E160_environment import *
from E160_graphics import *

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

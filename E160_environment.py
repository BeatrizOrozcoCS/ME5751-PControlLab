from E160_robot import *
from E160_state import *
# from E160_wall import *
from P_controller import *
# import serial
import time
# from xbee import XBee

class E160_environment:
    
    def __init__(self, deltaT = 0.1):
        self.width = 2.0
        self.height = 2.0

        # set up walls, putting top left point first
        # self.walls = []
        # self.walls.append(E160_wall([-0.5, 0.5, -0.5, -0.5],"vertical"))
        # self.walls.append(E160_wall([0.5, 0.5, 0.5, -0.5],"vertical"))
        # self.walls.append(E160_wall([-0.5, 0.5, 0.5, 0.5],"horizontal"))
        # self.walls.append(E160_wall([0.5, -0.5, 1.0, -0.5],"horizontal"))
        # self.walls.append(E160_wall([0.0, -0.5, 0.0, -1.0],"vertical"))

        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE" 
        #self.robot_mode = "HARDWARE MODE"
        self.control_mode = "MANUAL CONTROL MODE"
        
        # # setup xbee communication
        # if (self.robot_mode == "HARDWARE MODE"):
        #     self.serial_port = serial.Serial('/dev/tty.usbserial-DN01IWND', 9600)
        #     print" Setting up serial port"
        #     try:
        #         self.xbee = XBee(self.serial_port)
        #     except:
        #         print("Couldn't find the serial port")

        # Setup the robots
        self.num_robots = 1
        self.robots = []
        for i in range (0,self.num_robots):
            
            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i, deltaT)
            self.robots.append(r)

        self.robots[0].set_controller(P_controller(self.robots[0]))# set P_controller for controller 0


    def update_robots(self, deltaT):
        
        # loop over all robots and update their state
        for r in self.robots:
            if(self.control_mode == "AUTONOMOUS CONTROL MODE"):
                if(r.controller.track_point()==True):
                    self.control_mode = "MANUAL CONTROL MODE"
            # set the control actuation
            # r.update(deltaT)

    def log_data(self):
        pass
        
        # # loop over all robots and update their state
        # for r in self.robots:
        #     r.log_data()
            
    def quit(self):
        exit()
        # self.xbee.halt()
        # self.serial.close()
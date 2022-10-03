
from E160_state import *
import math
import datetime

class E160_robot:

    def __init__(self, environment, address, robot_id, deltaT = 0.1):
        self.environment = environment
        self.state = E160_state()
        self.state.set_pos_state(0,0,0)
        self.state_des = E160_des_state()
        # self.state_des.set_des_state(0,0,0)
        self.v = 0
        self.w = 0
        self.set_bot_size()
        # self.L_pixel = 120
        # self.r_pixel = 3
        # # self.L = 0.12 #robot body radius
        # # self.r = 0.03 #robot wheel radius
        # self.L = float(self.L_pixel / 100.0)
        # self.r = float(self.r_pixel / 100.0)

        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + (datetime.datetime.now().replace(microsecond=0).__str__()).replace(":","_") + '.csv'
        # self.make_headers()
        self.deltaT = deltaT

    #this function is called in E160_graphics
    def set_controller(self, controller):
        self.controller = controller

    def set_bot_size(self, body_cm = 12, wheel_cm = 3):
        self.L_pixel = body_cm
        self.r_pixel = wheel_cm
        self.L = float(self.L_pixel / 100.0)
        self.r = float(self.r_pixel / 100.0)        

    def update(self, deltaT, mode = "S"):
        pass
        # # get sensor measurements
        # encoder_measurements, range_measurements = self.update_sensor_measurements()

        # # localize
        # self.localize(encoder_measurements, range_measurements, deltaT)
        
        # # call motion planner
        # #self.motion_planner.update_plan()

        # # determine new control signals
        # R, L = self.update_control(range_measurements)
        
        # send the control measurements to the robot

    
    def update_sensor_measurements(self):
        pass 
        # # send to actual robot !!!!!!!!! John
        # if self.environment.robot_mode == "HARDWARE MODE":
        #     command = '$S @'
        #     self.environment.xbee.tx(dest_addr = self.address, data = command)
            
        #     update = self.environment.xbee.wait_read_frame()
            
        #     data = update['rf_data'].decode().split(' ')[:-1]
        #     data = [int(x) for x in data]
        #     encoder_measurements = data[-2:]
        #     range_measurements = data[:-2]
            
        #     print "update sensors measurements ",encoder_measurements, range_measurements
        #     return encoder_measurements, range_measurements

        
        # # obtain sensor measurements !!!!!! Chris
        # elif self.environment.robot_mode == "SIMULATION MODE":
        #     return 1,1
        
        # return 1,1
        
    def localize(self, encoder_measurements, range_measurements, deltaT):
        pass
    

    def update_control(self, range_measurements):
        pass
        # if self.environment.control_mode == "MANUAL CONTROL MODE":
        #     R = self.manual_control_right_motor
        #     L = self.manual_control_left_motor
            
        # elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":        
            
        #     # Put student code for lab 1 control here!!!!!!
        #     # Don't keep the R=0, L=0 code
        #     R=0
        #     L=0
        
        # return R, L
            
    def send_control(self, R, L, deltaT):
        pass
        
        # # send to actual robot !!!!!!!!, will NOT be executed in CPP ME5751
        # if self.environment.robot_mode == "HARDWARE MODE":
        #     if (L < 0):
        #         LDIR = 0
        #     else:
        #         LDIR = 1

        #     if (R < 0):
        #         RDIR = 0
        #     else:
        #         RDIR = 1
        #     RPWM = int(abs(R))
        #     LPWM = int(abs(L))

        #     command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
        #     self.environment.xbee.tx(dest_addr = self.address, data = command)
            

        # # simulate kinematics
        # elif self.environment.robot_mode == "SIMULATION MODE":
        #     self.state = self.update_state(deltaT, self.state, R, L)

        
    # def update_state(self, deltaT, state, R, L):
    #     # Robot Kinematics !
    #     self.state.x = R * deltaT + self.state.x
    #     return state
        
        
    def make_headers(self,variables):
        f = open(self.file_name, 'a+')
        # edit this line to have data logging of the data you care about
        data = [str(x) for x in variables]
        
        f.write('\t'.join(data) + '\n')
        f.close()


    def log_data(self, variables):
        f = open(self.file_name, 'a+')
        
        # edit this line to have data logging of the data you care about
        data = [str(x) for x in variables]
        
        f.write('\t'.join(data) + '\n')
        f.close()

        
    def set_motor_control(self, v, w):
        if self.environment.robot_mode == "HARDWARE MODE":
            pass
            # self.manual_control_right_motor = int(R*256/100)
            # self.manual_control_left_motor = int(L*256/100)
        else:
            self.state.set_vel_state(v = v, w = w, deltaT = self.deltaT)

    def send_wheel_speed(self,phi_l,phi_r):
        if self.environment.robot_mode == "HARDWARE MODE":
            pass
            # self.manual_control_right_motor = int(R*256/100)
            # self.manual_control_left_motor = int(L*256/100)
        else:
            self.state.set_wheel_speed(phi_l = phi_l, phi_r = phi_r)
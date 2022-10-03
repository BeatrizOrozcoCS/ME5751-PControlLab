#!/usr/bin/python
# -*- coding: utf-8 -*-
from E160_state import *
from E160_robot import *
import math
import time


class P_controller:

    def __init__(self, robot, logging = True):
        self.robot = robot  # do not delete this line
        #k constants - for 3 iterations
        #        KP  KB  KA
        #    1  5   -2    10 (done)
        #    2  3   -2    6
        #    3  0.7 -2    10
         #   4  0.7 -2    5
        
        self.ka = 6 # k_alpha
        self.kb = -2 # k_beta   
        self.kp = 3 # k_ho

        self.logging = logging

        if(logging == True):
            #all the values
            self.robot.make_headers(['TurnsLeft','kp','kb','ka','dpos_X','dpos_Y','dpos_theta', 'cpos_X','cpos_Y','cpos_theta','Deltax', 'deltay','deltatheta', 'rho', 'alpha', 'beta', 'vix','viy','wi','vr','wr','phi_l','phi_r'])

        self.set_goal_points()

    #Edit goal point list below, if you click a point using mouse, the psoints programmed
    #will be washed outs
    def set_goal_points(self):
        # here is the example of destination code      

       self.robot.state_des.add_destination(x=0,y=200,theta=math.pi/8)    #goal point 1
       self.robot.state_des.add_destination(x=0,y=-200,theta=-math.pi/4) #goal point 2
       self.robot.state_des.add_destination(x=-100,y=0,theta=-math.pi/8) #goal point 3
       self.robot.state_des.add_destination(x=-250,y=200,theta=math.pi/4) #goal point 4
       self.robot.state_des.add_destination(x=0,y=200,theta=0.5)    #goal point 5 - back to begginning
        
        
    def track_point(self):
        # All d_ means destination
        (d_posX, d_posY, d_theta) = self.robot.state_des.get_des_state()  # get next destination configuration

        # All c_ means current_
        (c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
        (c_vix, c_viy, c_wi) = self.robot.state.get_global_vel_state() #get current velocity configuration, in the global frame
        (c_v, c_w) = self.robot.state.get_local_vel_state() #get current local velocity configuration


        # Most of your program should be here, compute rho, alpha and beta using d_pos and c_pos
        deltaX = (d_posX-c_posX)
        deltaY = (d_posY-c_posY)
        lamda = math.atan2(deltaY,deltaX)
        theta = c_theta
        rho = math.sqrt(math.pow(deltaX,2)+math.pow(deltaY,2))
        
        alpha = -theta + lamda
        alpha= normalizeAngle(alpha) #%normalizeAngle   set angle to the range [-pi,pi)
        
        print("og alpha: ", alpha)

        if (alpha <= math.pi/2 or alpha > -math.pi/2 ): # go forward
            beta = -d_theta - alpha
            #beta= normalizeAngle(beta)    #%normalizeAngle   set angle to the range [-pi,pi)
            
            #calcualte wheelspeed
            c_v = self.kp*rho
            c_w = self.ka*alpha + self.kb*beta

        else: #go backward
            alpha = math.atan2(-deltaY,-deltaX)-theta- math.pi 
            alpha= normalizeAngle(alpha) #%normalizeAngle   set angle to the range [-pi,pi)


            beta = -d_theta- alpha
           # beta= normalizeAngle(beta)    #%normalizeAngle   set angle to the range [-pi,pi)
  

            #calcualte wheelspeed
            c_v = -self.kp*rho
            c_w = self.ka*alpha + self.kb*beta

        turnsLeft = str(self.robot.state_des.get_des_size() - self.robot.state_des.p)
        print("turns left#: "+ turnsLeft)
        print("c_theta: ", c_theta)
        print("d_theta: ", d_theta)
        print("theta: ", theta)
        print("deltaX: ", deltaX)
        print("deltaY: ", deltaY)
        print("new alpha: ", alpha)
        print("beta: ", beta)
        
        # self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
        self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
        
        #Calculate wheel rotation speed
        r= 3
        L = 24
        phi_r = c_v / r + (c_w*L)/r
        phi_l = (c_v - c_w*L)/r
        
        self.robot.send_wheel_speed(phi_l,phi_r ) #unit rad/s
        # use the following to log the variables, use [] to bracket all variables you want to store

        # stored values are in log folder
        if self.logging == True:
            self.robot.log_data([turnsLeft, self.kp,self.kb,self.ka,d_posX,d_posY,d_theta, c_posX,c_posY,c_theta, deltaX, deltaY,abs(d_theta - c_theta),  rho,  alpha, beta, c_vix,c_viy,c_wi,c_v,c_w,phi_l,phi_r])

        threshold = 3 # threshold constant for directoin
        radsthreshold = 0.14 # threshold constant for oreitnation (0.14 rads = 8 degrees)
        if abs(deltaX) <threshold and  abs(deltaY) <threshold and abs(d_theta - c_theta)<radsthreshold : #you need to modify the reach way point criteria
            if(self.robot.state_des.reach_destination()): 
                print("final goal reached")
                self.robot.set_motor_control(.0, .0)  # stop the motor
                return True
            else:
                print("one goal point reached, continute to next goal point")
                

        return False

def normalizeAngle(angle):
    if(angle>math.pi):   #%normalizeAngle   set angle to the range [-pi,pi)
        angle -= math.pi*2
    elif(angle<-math.pi):
        angle += math.pi*2

    return angle

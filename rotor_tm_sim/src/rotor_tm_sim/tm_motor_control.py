#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 22:40:24 2022

@author: p00972zl
"""
import numpy as np
from scipy import linalg
import rospy

#kf: 4.179e-09 # unit: N/rpm^2
# km: 3.97005e-11 # unit: Nm/rpm^2

#mass: 0.25
#inertia: {Ixx: 0.000601, Ixy: 0.0, Ixz: 0.0, Iyx: 0.0, Iyy: 0.000589, Iyz: 0.0, Izx: 0.0, Izy: 0.0, Izz: 0.001076} arm_length: 0.1075 

#max_angle: 80 #degree

#num_props: 4
#max_rpm: 16400
#min_rpm: 5500
# motor_coefficients: 4.0172e-07


class motorControl:
    # init function
    def __init__(self, c_step):
        # simulation step
        self.step = c_step
        # inner loop frequency
        self.inner_fre = 100

        # motor speed limit
        self.MAX_motorSpeed = 16400

        self.Min_motorSpeed = 5500  

        self.Euili_motorSpeed = 12106;

        # motor speed
        self.v_ref_motorSpeed= np.zeros([4, 1], dtype=np.float64)
        # mapping matrix
        kf =  4.179e-09 
        km= 3.97005e-11
        d = 0.1075
        # wrench = [d*kf^2 d*kf^2 d*kf^2 d*kf^2] * [omega_1^2, omega_2^2, omega_3^2, omega_4^2]^T
        #          [d*kf^2 0 -d*kf^2 0]
        #          [0 d*kf^2 0 -d*kf^2]
        #          [km^2 km^2 km^2 km^2]
        #self.m_mapMotor2Wrench = np.eye(4,dtype=np.float64)
        self.m_mapMotor2Wrench = np.array([[kf, kf, kf, kf], [np.dot(kf,d), 0, -np.dot(kf,d), 0], [0, np.dot(kf,d), 0, -np.dot(kf,d)], [-km, km, -km, km] ],dtype=np.float64)
        # motor speed in last step
        # intilise with min speed
        # self.v_motorSpeed_old = np.zeros((4,1))
        self.v_motorSpeed_old = np.array([self.Euili_motorSpeed, self.Euili_motorSpeed, self.Euili_motorSpeed, self.Euili_motorSpeed],dtype=np.float64).reshape(4,1)
        # motor speed in current step
        self.v_motorSpeed = np.zeros((4,1))      


        # gin
        self.PGain = 5
        #
        self.outputWrench = np.zeros([4, 1], dtype=np.float64)
        #
        self.inputWrench = np.zeros([4, 1], dtype=np.float64)
        # 
        self.v_dmotorSpeed = np.zeros([4, 1], dtype=np.float64)
        #
        self.output= []
        #
        self.controlInput = []
   #
    def quadrotor_wrench2motor(self):
        # get the norm of thrust force magnitude
        #thrust = np.linalg.norm(self.v_thrust)
    
        # build wrench vector with a dimention of 4 X 1
        # v_wrench =  [thrust, torque in body frame]
        #v_wrench = np.zeros([4, 1], dtype=np.float64)
        
        # get thrust as reference by using view
        #v_wrench[0] = thrust.view()
        
        # get torque with 3 elements as reference by using view
        #v_wrench[1:4] = self.v_torque.reshape(3,1).view()
        
        # self.m_mapMotor2Wrench is mapping matrix that maps motor speed square to drone wrench
        # v_wrench = m_mapMotor2Wrench * v_motorSpeed2
        
        # motor speed square
        # v_motorSpeed2 = inv(m_mapMotor2Wrench) * v_wrench
        v_motorSpeed2 = np.zeros([4, 1], dtype=np.float64)
        
        v_motorSpeed2[0:4] = np.dot(linalg.inv(self.m_mapMotor2Wrench), self.inputWrench)
        
        #print("motor square is", v_motorSpeed2.transpose())
        
        # v_motorSpeed is the motor speed 
        self.v_ref_motorSpeed[0:4] = np.sqrt(v_motorSpeed2.clip(np.power(self.Min_motorSpeed, 2), np.power(self.MAX_motorSpeed, 2)))
        
        #print("refe motor speed is", self.v_ref_motorSpeed.transpose())
         

    def motorController(self):
        #
        self.v_dmotorSpeed[0:4] = self.PGain * (self.v_ref_motorSpeed - self.v_motorSpeed_old)
        #print("motor control input", self.v_dmotorSpeed.transpose())
        self.controlInput.append(np.copy(self.v_dmotorSpeed.transpose()))
    
    def motorDynamicSimulator(self):
        ## TODO debug
        #print("motor control input", self.v_dmotorSpeed.transpose())  
        #print("motor control step", self.step)  
        #print("motor control step * input", self.v_dmotorSpeed.transpose()*self.step)  

        # without saturate
        # self.v_motorSpeed[0:4] = self.v_dmotorSpeed*self.step + self.v_motorSpeed_old
             
        # saturate 
        v_motorSpeed_raw = self.v_dmotorSpeed*self.step + self.v_motorSpeed_old
        #print("motor speed raw", v_motorSpeed_raw.transpose())  
        self.v_motorSpeed[0:4] = np.clip(v_motorSpeed_raw, self.Min_motorSpeed, self.MAX_motorSpeed)
        #print("motor speed sat", self.v_motorSpeed.transpose())  
    
    def quadrotor_motor2wrench(self):
        # mapping matrix that maps motor speed square to drone wrench
        # v_wrench = self.m_mapMotor2Wrench * v_motorSpeed2
    
        # motor speed square
        # v_motorSpeed2 = inv(m_mapMotor2Wrench) * v_wrench
    
        self.outputWrench[0:4] = np.dot(self.m_mapMotor2Wrench, np.square(self.v_motorSpeed))
        #print("output wrench", self.outputWrench.transpose())  
          
    def inputRefWrench(self, ref_v_thrust, ref_v_torque):
        # get the norm of thrust force magnitude
        thrust = np.linalg.norm(ref_v_thrust)
    
        
        # get thrust as reference by using view
        self.inputWrench[0] = thrust.view()
        
        # get torque with 3 elements as reference by using view
        self.inputWrench[1:4] = ref_v_torque.reshape(3,1).view()       
    
    
    def runController(self):
        # 1. compute reference motor speed
        self.quadrotor_wrench2motor()

        time = 100*self.step
        ##---------------------------- debug-------------------------------------
        #output= np.empty([time/self.step,4])
        #print(time/self.step)
        
     
        # self.step =0.001
        for t in np.arange(0, time, self.step): 
            # 2. do motor speed control
            #print("ref motor speed ", self.v_ref_motorSpeed.transpose())
            self.motorController()
            #print("motor commands ", self.v_dmotorSpeed.transpose())
            #print("old motor speed ", self.v_motorSpeed_old.transpose())

            # 3. calculate motor speed base don dynmaic of motor
            self.motorDynamicSimulator()
            #print("motor speed ", self.v_motorSpeed.transpose())

            # 4. compute output wrench        
            self.quadrotor_motor2wrench()
            print("new wrench at", rospy.Time.now().to_sec(), " is ", self.outputWrench.transpose())

            # 5. update motor speed
            self.v_motorSpeed_old = self.v_motorSpeed
            
            # print("current element",self.outputWrench.transpose())      
            #
            self.output.append(np.copy(self.outputWrench.transpose()))
            #self.output.append(self.outputWrench.transpose())

            # print("first element", output[0])
            #self.outputWrench = self.inputWrench   



       # return self.output
            
    def getOutputWrench(self):
        return self.outputWrench.transpose()
    








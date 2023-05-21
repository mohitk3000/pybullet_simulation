#!/usr/bin/env python
# coding: utf-8


import numpy as np
import pybullet as p
import time
import pybullet_data
import inspect
#from sensor_msgs.msg import JointState
import threading
import datetime
import csv
import pandas as pd


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	physicsClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally


planeId = p.loadURDF("plane.urdf")

#spawn cube
cube_big = p.loadURDF("cube_big.urdf",[-1.03,-0.03, 0.1], globalScaling=0.1)
p.changeDynamics(cube_big,-1, lateralFriction=0.5)




kuka_allegro_hand_biotac = p.loadURDF("kuka-allegro-biotac.urdf",[0,0,0],useFixedBase=1)
for j in range(53):
    p.enableJointForceTorqueSensor(kuka_allegro_hand_biotac, j, enableSensor=1)


numJoints = p.getNumJoints(kuka_allegro_hand_biotac)

p.setRealTimeSimulation(1)


#RUN AGAIN FROM HERE

from datetime import datetime
p.setGravity(0,0,-9.8)


#reset
p.changeDynamics(kuka_allegro_hand_biotac,-1, lateralFriction=0.5)
p.changeDynamics(kuka_allegro_hand_biotac,-1, rollingFriction=0.5)
joint_cmd = [0 for _ in range(53)]
p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

#RECORD 1ST FROM HERE
time.sleep(5)


time.sleep(5)


#Start Reaching to the cube

#REACH (in steps)



for i in range(15+1):
    joint_cmd[5] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)

for i in range(12+1):
    joint_cmd[7] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

for i in range(90+1):
    joint_cmd[3] = i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)


time.sleep(5)
    


#THUMB
joint_cmd[43]= 90 * np.pi/180
joint_cmd[45]= 5 * np.pi/180
joint_cmd[46]= 10 * np.pi/180

#INDEX_UNGERS
joint_cmd[17]= 40 * np.pi/180    
joint_cmd[18]= 40 * np.pi/180    
joint_cmd[19]= 20 * np.pi/180 
#joint_cmd[19]= 5 * np.pi/180 


# MIDDLE_FINGER
joint_cmd[26]= 40 * np.pi/180
joint_cmd[27]= 40 * np.pi/180
joint_cmd[28]= 20 * np.pi/180 
#joint_cmd[28]= 5 * np.pi/180



# PINKY FINGER
joint_cmd[35]= 40 * np.pi/180
joint_cmd[36]= 40 * np.pi/180
joint_cmd[37]= 20 * np.pi/180

p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
time.sleep(1/20.)

time.sleep(5)


T = 250
delta = 0.01
t=0


#to get the data from the csv file
i=0

data=pd.read_csv('test6.csv')
data=data.drop(['Timestamp', 'Size','Mass','Slip','Crumple'], axis=1)

p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)



# PICKUP
#PCIKUP (IN STEPS)
while t<T:
    cubePos, cubeOrn= p.getBasePositionAndOrientation(cube_big)
    #print(cubePos, cubeOrn)
    x_time = datetime.now().time()
    joint_cmd[3] = joint_cmd[3] - delta
    joint_cmd[5] = joint_cmd[5] - delta
    a=data.iloc[i].values
    ## joint based
    #joint_cmd[7] = joint_cmd[7] + delta
    #joint_cmd[46] = joint_cmd [46] - 0.1* delta
    joint_cmd[19] = joint_cmd[19] - 10*delta
    #joint_cmd[28] = joint_cmd[19] - 0.099*delta
    
    

    #{plug threshold based}
    #THUMB
    joint_cmd[43]= a[24]   
    joint_cmd[44]= a[26]    
    joint_cmd[45]= a[28] 
    joint_cmd[46]= a[30] 
    joint_cmd[48]= a[25]   
    joint_cmd[49]= a[27]    
    joint_cmd[50]= a[29] 
    joint_cmd[51]= a[31] 

    #INDEX_UNGERS
    joint_cmd[16]= a[0]   
    joint_cmd[17]= a[2]    
    joint_cmd[18]= a[4] 
    joint_cmd[19]= a[6] 
    joint_cmd[21]= a[1]   
    joint_cmd[22]= a[3]    
    joint_cmd[23]= a[5] 
    joint_cmd[24]= a[7]  


    # MIDDLE_FINGER
    joint_cmd[25]= a[8]   
    joint_cmd[26]= a[10]    
    joint_cmd[27]= a[12] 
    joint_cmd[28]= a[14] 
    joint_cmd[30]= a[9]   
    joint_cmd[31]= a[11]    
    joint_cmd[32]= a[13] 
    joint_cmd[33]= a[15] 



     # PINKY FINGER
    joint_cmd[34]= a[16]   
    joint_cmd[35]= a[18]    
    joint_cmd[36]= a[20] 
    joint_cmd[37]= a[22] 
    joint_cmd[39]= a[17]   
    joint_cmd[40]= a[19]    
    joint_cmd[41]= a[21] 
    joint_cmd[42]= a[23] 
    

    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/50.)
    
    t += 1
    i+=1









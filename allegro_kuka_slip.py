#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import pybullet as p
import time
import pybullet_data
import inspect
#from sensor_msgs.msg import JointState
import threading
import datetime
import csv


# In[336]:


def save_to_csv(FILENAME, TRACE_CSV, type_open='a'):    
    with open(FILENAME,type_open,newline="") as trace_file:
        writer = csv.writer(trace_file, )
        writer.writerow(TRACE_CSV)



# In[ ]:





# In[3]:


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	physicsClient = p.connect(p.GUI)


# In[4]:


p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally


# In[5]:


def get_joint_states(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple == 2:
                for l in k:
                    DATUM.append(l)
            else:
                DATUM.append(k)
    return DATUM

def get_joint_angles(robot, numJoints):
    DATUM=[]
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple ==1 : #Just the joint angle
                DATUM.append(k)
    return DATUM

def get_joint_states_hand_only(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for jno, j in enumerate(joint_states):
        if jno in [16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47]:
            #print(jno)
            for quadruple, k in enumerate(j):
                if quadruple == 2:
                    for l in k:
                        DATUM.append(l)
                else:
                    DATUM.append(k)
    return DATUM


# In[6]:


planeId = p.loadURDF("plane.urdf")


# In[7]:
#spawn cube
cube_small = p.loadURDF("cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)
p.changeDynamics(cube_small,-1, lateralFriction=0.5)


# Loading KUKA 1 
#robot = p.loadURDF("ll4ma_robots_description/urdf/allegro_right/allegro_hand_description_right.urdf")
kuka_allegro_hand_biotac = p.loadURDF("kuka-allegro-biotac.urdf",[0, 0, 0], useFixedBase=1)
for j in range(53):
    p.enableJointForceTorqueSensor(kuka_allegro_hand_biotac, j, enableSensor=1)


# In[8]:





# In[10]:


#cube_big = p.loadURDF("./Haptics/haptics_examples/objects/cube_small.urdf",[-1.03,-0.03, 0.1], globalScaling=2.5)


# In[9]:


numJoints = p.getNumJoints(kuka_allegro_hand_biotac)


# In[10]:


p.setRealTimeSimulation(1)


# # RUN AGAIN FROM HERE

# In[325]:


from datetime import datetime
p.setGravity(0,0,-9.8)

						
						
					
# In[326]:


#save_to_csv(FILENAME,['START',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')


# In[327]:


#for _ in range(200):
#    save_to_csv(FILENAME,['STEADY',datetime.now().time() ]+get_joint_states_hand_only( kuka_allegro_hand_biotac, numJoints),'a')


# In[ ]:





# In[ ]:





# In[328]:


#reset
p.changeDynamics(kuka_allegro_hand_biotac,-1, lateralFriction=0.5)
p.changeDynamics(kuka_allegro_hand_biotac,-1, rollingFriction=0.5)
joint_cmd = [0 for _ in range(53)]
p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
#RECORD 1ST FROM HERE




# In[329]:
time.sleep(10)


# In[330]:


#spawn cube



# ### Start Reaching to the cube

# In[331]:


#PCIKUP (IN STEPS)
#REACH (in steps)
time.sleep(10)




for i in range(15+1):
    joint_cmd[5] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)
    

for i in range(12+1):
    joint_cmd[7] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)
    

for i in range(90+1):
    joint_cmd[3] = i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)
 
    
    


time.sleep(10)
    

# In[ ]:





# In[332]:


#joint_cmd = [0 for j in range(53)]



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


# In[333]:


#print(get_joint_angles(kuka_allegro_hand_biotac,53))
time.sleep(10)


# In[334]:


# PICKUP
angles = get_joint_angles(kuka_allegro_hand_biotac,53)

T = 100
delta = 0.01
t=0

grav = 0
while t < T:
    joint_cmd[3] = joint_cmd[3] - delta
    joint_cmd[5] = joint_cmd[5] - delta
    #joint_cmd[7] = joint_cmd[7] + delta
    #joint_cmd[46] = joint_cmd [46] - 0.1* delta
    #joint_cmd[19] = joint_cmd[19] - 0.099*delta
    #joint_cmd[28] = joint_cmd[19] - 0.099*delta
    if t >40 and t<42:
        
        grav = grav - 9.8 - (t*0.2)
        print('Gravity='+str(grav))
        p.setGravity(0,0,grav)
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    time.sleep(1/20.)
  
    t += 1
    time.sleep(1/100.)



    
"""    
for i in range(15+1):
    joint_cmd[7] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)
    
for i in range(15+1):
    joint_cmd[5] = -i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

for i in range(90+1):
    joint_cmd[3] = i *np.pi/180
    p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(53), p.POSITION_CONTROL,  targetPositions=joint_cmd)

"""


# In[267]:


#REFERENCE TO JOINTS INFORMATION
#2 is base of kuka
#8 is wrist (end of kuka)
#9 is nothing upto 15

#16- index
#17- index
#18- index
#19- index
#20- index
#21 to 24 are biotacs for index finger


#25 - middle 0
#26 - middle 1
#27 - middle 2
#28 - middle 3
#29 - middle 4
#30 to 33 are biotacs for middle finger

#34 - pinky 0
#35 - pinky 1
#36 - middle 2
#37 - middle 3
#38 - middle 4
#39 to 42 are biotacs for middle finger


#43 - thumb 0
#44 - thumb 1
#45 - thumb 2
#46 - thumb 3
#47 - thumb 4
#48 - 51 are biotacs for middle finger


#52 and 53 are useless

#for j in range(17,19):
#    joint_cmd [j] = 145*np.pi/180


# In[233]:


p.setGravity(0,0,-9.8-10)


# In[335]:


print(grav)


# In[ ]:





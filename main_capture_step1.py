# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 22:38:50 2021

@author: Dylan Tan
"""

import numpy as np
import pandas as pd
import math
from robot import robot
from camera import mykinectazure
from geometry import unit, getRmatrix, rotateonaxis, rmat2rot,rmat2rpy
from plot import extractlog, plotxy, plotxz, plotyz, plot3d
from matplotlib import pyplot as plt
import time
import pickle

# from matplotlib import image

#Options: depth_mode=pyk4a.DepthMode.NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_2X2BINNED, 
# if not specificed, color is BGRA 720p

#%%
#Tasks and movement sequences

def calibrate_home_position(robot):
    """# calibrate function. Use it to place the cornerblock and cone in the exact home position
    
    # 9cm on length +5cm for safety, = 14cm
    # 8.9cm for width +5cm for safety = 13.9cm
    
    
    """
    
    robot.wtrJ()
    robot.rotateanchor(45,0)
    robot.moveanchor(0.3,0.3,0,10)
    img_color,a,b = camera.cap()
    plt.imshow(img_color) # BGRA to RGB , cmap="hot"
    plt.show()
    # df = capturerecord(robot,camera,conetype,df)
    # wtr(robot,anchor)
    calibrate = robot.anchor.copy()
    calibrate[0] = robot.home[0]-0.106066
    calibrate[1] = robot.home[1]+0.106066
    # robot.moveanchor(0.3,0.3,0,10)
    robot.wtr(calibrate,0.3,0.3,0,10)

    # time.sleep(5)

    robot.wtrJ()
    # print("first",self.anchor)
    robot.rotateanchor(-90,0)
    robot.moveanchor()
    img_color,a,b = camera.cap()
    plt.imshow(img_color) # BGRA to RGB , cmap="hot"
    plt.show()
    # df = capturerec
    calibrate = robot.anchor.copy()
    calibrate[0] = robot.home[0]-0.106066
    calibrate[1] = robot.home[1]-0.106066
    robot.wtr(calibrate,0.3,0.3,0,10)
    return

def task(robot,camera,df,direction=1,originkeep = True):
    
    """
    Direction parameter refers to either left of center or right of center. 
    1 means it wil sweep left "timestep" number of times. while -1 means it will sweep right
    """
    
    timestep = 6
    totaldeg = 60
    interval = totaldeg / timestep
    for j in range(timestep):
        for i in range(timestep):
            
            # if(i<2):
            #     df = task2(robot,camera,df)
            
            robot.rotateanchor(direction*interval,0)
            robot.moveanchor(t=3)
            robot.printanchor()
            
            # if(i>=2):
            #     df = capturerecord(robot,camera,conetype,df)
            df = capturerecord(robot,camera,conetype,df)
            
        if(j==timestep-1):
            robot.wtrJ()
            robot.rotateanchor(-interval*timestep*direction,-interval*(timestep-1))
            robot.moveanchor()
            robot.printanchor()
            # df = capturerecord(robot,camera,conetype,df)
        else:
            robot.wtrJ()
            robot.rotateanchor(-direction*totaldeg,interval)
            robot.moveanchor()
            robot.printanchor()
            if(originkeep):
                df = capturerecord(robot,camera,conetype,df)
        
    log = robot.log
    log = extractlog(log)
    plot3d(log)
    return df

def task2(robot,camera,df,interval=10,dim=0.5):
    
    """
    A movement sequence that translates the flyer around in a box sequence where the 2D box is on the vertical and horizontal axis
    
    """
    
    robot.getfixedtranslatebox(interval=interval, dim=dim)
    
    for i in range(len(robot.translatebox)):
        robot.translateflyer(t=0.5, t2=3, interval=8)
        robot.printflyer()
        df = capturerecord(robot,camera,conetype,df)
        if i==35: return df


    return df

def rotateMove(robot,df, h, v,t=6):
    robot.rotateanchor(h,v)
    robot.moveanchor(t=t)
    robot.printanchor()
    df = capturerecord(robot,camera,conetype,df)
    return df

def task3(robot,camera,df):
    """
    A movement sequence that rotates about a fixed home position. 
    This is similar to task 1, but it sweeps from left to right in 12 different postions (task 1 is only 6 positions from the center)
    Make sure that home is in robot.py is correctly specified and that the cone is calibrated corrected to the home position
    """
    timestep = 12
    totaldeg = 120
    interval = totaldeg / timestep
    df = rotateMove(robot,df,60,0)
    for i in range(6):
        for j in range(timestep):
            df = rotateMove(robot,df,-interval,0,t=3)
            if(j == (timestep/2)-2):
                robot.wtrJ()
        robot.wtrJ()
        if (i==5):
            robot.wtrJ()
            return df
        else:
            df = rotateMove(robot,df,120,10)
                    
    robot.wtrJ()
    return df

def task_collectDatabase():
    robot.wtrJ()

    robot.rotateonspot(thetaV=30)
    robot.moveanchor()
    robot.translateglobcm(x=20)
    robot.moveanchor()
    quickCapture(camera)
    task2(robot,camera,df,interval=8 , dim=0.8)
    print("First layer completed")

    robot.translateglobcm(x=-10)
    robot.moveanchor()
    task2(robot,camera,df,interval=8 , dim=0.8)
    print("Second layer completed")

    robot.translateglobcm(x=-10)
    robot.moveanchor()
    task2(robot,camera,df,interval=8 , dim=0.8)
    print("Task completed")

    return    
    

#%%    

def capturerecord(robot,camera,conetype,df):
    
    camera.cap()
    flyer = robot.flyer
    anchor = robot.anchor
    home = robot.home
    
    homex = unit(np.array([-1,1,0]))
    homey = unit(np.array([-1,-1,0]))
    homez = unit(np.array([0,0,1]))
    eye = np.eye(3)

    homerefmat = np.array([[np.dot(homex,eye[i]) for i in range(3)],
                           [np.dot(homey,eye[i]) for i in range(3)],
                           [np.dot(homez,eye[i]) for i in range(3)]])
    
    
    POSE = [home[0]-flyer[0],home[1]-flyer[1],home[2]-flyer[2]] #change these parts accordingly
    negativepose = [flyer[0]-home[0],flyer[1]-home[1],flyer[2]-home[2]]
    
    # theta = [flyer[6],flyer[7]-1.57,(flyer[8]+0.7853981)]
    drefframeT = change_ref_frame(home,anchor)                     #relative reference frame Transformation matrix
    
    #these angle conventions are for rightcorner block using right hand rule thumb pointed into the block
    # netpose = POSE
    xyplane = np.array([POSE[0],POSE[1],0])
    yaxis = np.array([0,-1,0])
        
    dy = np.dot(unit(np.array(xyplane)),unit(yaxis))
    Yangle = math.acos(dy) 
    Pangle = math.acos(round(np.dot(unit(np.array(POSE)),unit(xyplane)),6)) ##round is to make sure dot product doesnt have 1.00001 which screwsup the acos()
        
    
    if(POSE[0]<0):
        Yangle = -Yangle
    if(POSE[0]<0 and POSE[1]>0):
        Yangle += 2*math.pi  
    theta = [flyer[6],-1*Pangle,(Yangle-0.7853981)]
        
    RelPose = np.matmul(drefframeT,POSE)
    RelPose = np.round(RelPose,5) 
    homeorigin = np.matmul(homerefmat,negativepose) # this is the global coordinate syste changed to home cone as the new axis and origin
    homeorigin = np.round(homeorigin,5)  
    
    df = df.append({'Cycle':camera.namecounter,'Cone':conetype, 'RGBname':camera.RGBname,'DEPTHname':camera.DEPTHname, 
                    'RGBD':camera.RGBDname,'Pose':homeorigin, 'RelPose':RelPose, 'Theta':theta} , ignore_index=True)
    
    camera.saveupdate()

    return df

def quickCapture(camera):
    """
    quick capture, save and update counters    

    """
    camera.cap()
    camera.saveupdate()
    return


#%% high level functions that transform camera axis to robot axis

def change_ref_frame(home,anchor): # change reference frame creff for relative 
    normal = np.array([home[0]-anchor[0],home[1]-anchor[1],home[2]-anchor[2]])
    unorm = unit(normal)
    znorm = np.array(anchor[9:])*-1
    ynorm = np.cross(znorm,unorm)
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(unorm,eye[i]) for i in range(3)],
                        [np.dot(ynorm,eye[i]) for i in range(3)],
                        [np.dot(znorm,eye[i]) for i in range(3)]])
    return refmat



def change_ref_frame_trans(anchor):
    """"
    Change reference frame for translation of camera axis to robot axis
    This does a 2 simple rotations of camera axis to fit the robot axis exactly.
    """
    
    #this method use 
    
    
    #Rightsideup
    # t1 = rotateonspot(np.eye(3),np.array([0,0,1]),90)
# # print(t1)
    # t2 = rotateonspot(np.eye(3),np.array([0,1,0]),60)
# print(t2)

    #Upsidedown
    t1 = rotateonaxis(np.eye(3),np.array([0,0,1]),270) #270
# print(t1)
    t2 = rotateonaxis(np.eye(3),np.array([0,1,0]),120)
    
    #newest
    # t1 = rotateonspot(np.eye(3),np.array([0,0,1]),90) #270
# print(t1)
    # t2 = rotateonspot(np.eye(3),np.array([0,1,0]),120)
    
    
    t = np.matmul(t1,t2) 

    

    
    return t


    
def rotateoncamera(axischoice, theta):
    """
    

    Parameters
    ----------
    axischoice : int
        Axis of rotation: [xaxis,yaxis,zaxis,vertical]
        
    theta : int
        angle of rotation in degree. 

    Returns
    -------
    None.

    """
    
    identity = np.eye(3)
    # camera xaxis in the robot's definition: 
    
    #Original rightside up X and Y axis
    # xaxis = [0,-1,0]
    # yaxis =  [0.5,0,0.866]   #robot.anchor[9:]
    
    # Upsidedown
    xaxis = [0,1,0]
    yaxis =  [0.5,0,-0.866]   #robot.anchor[9:]  
    
    #newest
    # xaxis = [0,-1,0]
    # yaxis =  [-0.5,0,0.866]   #robot.anchor[9:]  
    
    zaxis = np.cross(xaxis,yaxis)
    vertical = np.array([0,0,1])
    chosenaxis = [xaxis,yaxis,zaxis,vertical]
    
    currentrotatin = getRmatrix(robot.anchor[6],robot.anchor[7],robot.anchor[8])
    
    
    # normal = [self.home[0]-self.anchor[0],self.home[1]-self.anchor[1],self.home[2]-self.anchor[2]]
    # unitnormal = unit(np.array(normal))
    # vaxis = self.anchor[9:]
    # zaxis = np.array([0,0,1]) #using z axis
    # haxis = np.cross(zaxis,unitnormal)
    
    
    
    Hrad = (3.142*theta)/180
    
    axis = chosenaxis[axischoice]
    
    W = np.array([[0, -axis[2], axis[1]],
               [axis[2], 0, -axis[0]],
               [-axis[1], axis[0], 0]])

    rod = identity + np.sin(Hrad)*W + (1-np.cos(Hrad)) * np.matmul(W,W)
    newv = np.matmul(rod,currentrotatin)
    robot.anchor[3:6] = rmat2rot(newv)
    robot.anchor[6:9] = rmat2rpy(newv)

    return

#%%

"""
1. Check robot connection T/F
2. Check home distance if you need to perform fixed radial tasks and make sure that the 
       home distance is correct to the physical location of the cone the camera will be rotating around.
3. Check camera connection T/F
4. Check name counter that it is 0 if you want the saving png to be 0000.png. set to 1 if you want 0001.png or
        if you dont want to overwrite the original image.

"""


path = 'temp\\'
robot = robot(connection=True,homedistance = 1000)
camera = mykinectazure(connection=False,namecounter=0)
camera.path = path

df = pd.DataFrame( columns=['Cycle','Cone','RGBname', 'DEPTHname', 'RGBD', 'Pose', 'RelPose', 'Theta'])
conetype = "C12"


#%% this is moving part




with open('final_tmat.p', 'rb') as f: 
    dict_tmat_pred = pickle.load(f)
    

tmat_1 = dict_tmat_pred

# flip = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# tmat_1 = np.matmul(tmat_1,flip)

Translation = tmat_1[0:3,3]
Rotation = tmat_1[0:3,0:3]


# robot.wtrJ()
print("Camera trans: ",Translation)

##this is for the original orientation
# robot.rotateonspot(thetaV=30)
# robot.translateglobcm(x=20)
# robot.moveanchor()

##this is for upside down orientation
# robot.rotateonspot(thetaV=-30)
# robot.translateglobcm(x=20)
# robot.translateglobcm(z=-7)

# robot.rotateonspot(thetaV=-30) #try rotate on camera instead
# robot.translateglobcm(x=3.4) #originially 3.9
# robot.translateglobcm(z=-26.2)
# robot.moveanchor()

#right side up mount
# robot.rotateonspot(thetaV=-30) #try rotate on camera instead
# robot.translateglobcm(x=4.5) #originially 3.9
# robot.translateglobcm(z=-26.7)
# # robot.moveanchor()

drefframeT=change_ref_frame_trans(robot.anchor)
newt = np.matmul(Translation,drefframeT)
print("Robot trans: ",newt)


#########3 xsafety = 5
########## angle = 3.142*30/180

# move translate, -2 to get accurate results

#rightsideup
# robot.translateglobcm(y=newt[1]*100-2) #for rightupsideup is -2
# robot.moveanchor()
# robot.translateglobcm(x=(newt[0]*100)-2) # forrightsideup is-2
# robot.moveanchor()
# robot.translateglobcm(z=(newt[2]*100)+15)#+11   put positive 15 for safety buffer  in rightside up, -15 for upside down
# robot.moveanchor()

# # upsidedown
# robot.translateglobcm(y=newt[1]*100+4) #for rightupsideup is -2
# robot.moveanchor()
# robot.translateglobcm(x=(newt[0]*100)) # forrightsideup is-2
# robot.moveanchor()
# robot.translateglobcm(z=(newt[2]*100)-15)#+11   put positive 15 for safety buffer  in rightside up, -15 for upside down
# robot.moveanchor()

#newest
# robot.translateglobcm(y=newt[1]*100-4) #for rightupsideup is -2
# robot.moveanchor()
# robot.translateglobcm(x=(newt[0]*100)) # forrightsideup is-2
# robot.moveanchor()
# robot.translateglobcm(z=(newt[2]*100)-15)#+11   put positive 15 for safety buffer  in rightside up, -15 for upside down
# robot.moveanchor()




#%% rotation part

############## rout = np.matmul(drefframeT.transpose(),Rotation)
############## Rotation = np.matmul(getRmatrix(0,0,1.57),Rotation)

########################################################################

# rpyorientation = rmat2rpy(Rotation)/3.142*180
# print("RPY for final: ", rpyorientation)

# xaxis = [0,-1,0]
# yaxis =  robot.anchor[9:]
# zaxis = np.cross(xaxis,yaxis)

# rotateoncamera(0,rpyorientation[0])
# rotateoncamera(1,rpyorientation[1])
# rotateoncamera(2,rpyorientation[2])

# # # """"Rightside up/ upside down read below properly"""
# rotateoncamera(3,-90) 
# robot.moveanchor()
# ###############################################################################
# # """ For original left images, 90 is correct, for flipped, take note it will likely be -90
# """ For upside down: -90 for L and 90 for flipped


# # #take note that the original is 90, but after nipuni, it changed to -90 as it 
# # seems axis definition has kinda changed"""

#for newest use 90 for regular and -90 for flipped

"""
To flip rightside up/ upside down
0. Go into the UR10 and remember to change the fixed axis from 60/-60mm appropriately
1. after print("Camera trans: ",Translation) select correct orientation
2. In the translation area a few lines down from that select appropriately as well
3. In the rotation area a few lines down from that read the 90/-90 part properly
4. Go to rotateoncamera function and change the axis appropriately
5. Go to crefftrans function and change the rotations inside appropriately as well

"""

#######################################################################
# df = capturerecord(robot,camera,conetype,df)
# 


# %%
"""
Collect for database
make sure that if cone is placed rightside up, that camera is also rightside up.
"""



# df.to_csv(path+'output.csv', index = None)
# # # 



print("End")






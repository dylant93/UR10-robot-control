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
    conetype = 'C12'
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

def task2(robot,camera,interval=10,dim=0.5):
    
    """
    A movement sequence that translates the flyer around in a box sequence where the 2D box is on the vertical and horizontal axis
    
    """
    
    robot.getfixedtranslatebox(interval=interval, dim=dim)
    
    for i in range(len(robot.translatebox)):
        robot.translateflyer(t=0.5, t2=3, interval=8)
        robot.printflyer()
        quickCapture(camera)
        if i==35: return


    return

def rotateMove(robot,df,conetype, h, v,t=6):
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
    
    conetype = "C12"
    timestep = 12
    totaldeg = 120
    interval = totaldeg / timestep
    df = rotateMove(robot,df,conetype,60,0)
    for i in range(6):
        for j in range(timestep):
            df = rotateMove(robot,df,conetype,-interval,0,t=3)
            if(j == (timestep/2)-2):
                robot.wtrJ()
        robot.wtrJ()
        if (i==5):
            robot.wtrJ()
            return df
        else:
            df = rotateMove(robot,df,conetype,120,10)
                    
    robot.wtrJ()
    return df


def task_collectDatabase():
    """
    Collect for database
    make sure that if cone is placed rightside up, that camera is also rightside up.
    This Task requires you to mount the camera on the arm. If not. Whats the point?
    """
    df = pd.DataFrame( columns=['Cycle','Cone','RGBname', 'DEPTHname', 'RGBD', 'Pose', 'RelPose', 'Theta'])
    
    robot.wtrJ()

    robot.rotateonspot(thetaV=30)
    robot.moveanchor()
    robot.translateglobcm(x=20)
    robot.moveanchor()
    quickCapture(camera)
    task2(robot,camera,interval=8 , dim=0.8)
    print("First layer completed")

    robot.translateglobcm(x=-10)
    robot.moveanchor()
    task2(robot,camera,df,interval=8 , dim=0.8)
    print("Second layer completed")

    robot.translateglobcm(x=-10)
    robot.moveanchor()
    task2(robot,camera,interval=8 , dim=0.8)
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



def change_ref_frame_trans(camera_orientation):
    """"
    Change reference frame for translation of camera axis to robot axis
    This does a 2 simple rotations of camera axis to fit the robot axis exactly.
    This is hardcoded as the axis of the robot and camera should not be dynamically changing
    """
    
    #this method use 
    
    if camera_orientation == 0:
    #Rightsideup arm
        t1 = rotateonaxis(np.eye(3),np.array([0,0,1]),90)
        t2 = rotateonaxis(np.eye(3),np.array([0,1,0]),60)


    elif camera_orientation == 1:
    #Upsidedown profile
        t1 = rotateonaxis(np.eye(3),np.array([0,0,1]),270) 
        t2 = rotateonaxis(np.eye(3),np.array([0,1,0]),120)

    elif camera_orientation == 2:    
    #rightsideup profile
        t1 = rotateonaxis(np.eye(3),np.array([0,0,1]),90) 
        t2 = rotateonaxis(np.eye(3),np.array([0,1,0]),120)
        
    return np.matmul(t1,t2) 



    
def rotateoncamera(camera_orientation, axischoice, theta):
    """
    

    Parameters
    ----------
    axischoice : int
        Axis of rotation: [xaxis,yaxis,zaxis,vertical]
        
    theta : int
        angle of rotation in degree. 

    This function does not update the anchor as it is meant to be a last movement

    """
    
    # camera xaxis in the robot's definition: 
    if camera_orientation == 0:
    #Original rightside up X and Y axis arm mounted
        xaxis = [0,-1,0]
        yaxis =  [0.5,0,0.866]   
    
    if camera_orientation == 1:
    # Upsidedown metal profile mounted
        xaxis = [0,1,0]
        yaxis =  [0.5,0,-0.866]  
    
    if camera_orientation == 2:
    # rightside up metal profile mounted
        xaxis = [0,-1,0]
        yaxis =  [-0.5,0,0.866]  
    
    zaxis = np.cross(xaxis,yaxis)
    vertical = np.array([0,0,1])
    
    #This part is broken like this to dissect the rotation into 3 steps for easier troubleshooting
    chosenaxis = [xaxis,yaxis,zaxis,vertical]
    
    currentrotatin = getRmatrix(robot.anchor[6],robot.anchor[7],robot.anchor[8])
    
    axis = chosenaxis[axischoice]
    newv = rotateonaxis(currentrotatin,axis,theta)
    
    robot.anchor[3:6] = rmat2rot(newv)
    robot.anchor[6:9] = rmat2rpy(newv)

    return



#%%

def initialSetup(camera_orientation):
    """
    Take note only camera orientation 1 is set up. These distances will change when you alter the mount setup.
    """
##this is for the original orientation on arm
    if camera_orientation == 0:
        robot.rotateonspot(thetaV=30)
        robot.translateglobcm(x=20)
        # robot.moveanchor()
        return

    #upside down profile mounted
    elif camera_orientation == 1:
        robot.rotateonspot_local(thetaV=-30) #try rotate on camera instead
        robot.translateglobcm(x=3.4) #originially 3.9
        robot.translateglobcm(z=-26.2)
        # robot.moveanchor()
        return

    #right side up mount profile
    elif camera_orientation == 2:
        robot.rotateonspot(thetaV=-30) #try rotate on camera instead
        robot.translateglobcm(x=4.5) #originially 3.9
        robot.translateglobcm(z=-26.7)
        # robot.moveanchor()
        return

def translate_arm_task(camera_orientation,newt):
    
    safety = 15
    #rightsideup arm mounted
    if camera_orientation == 0:
        robot.translateglobcm(y=newt[1]*100-2) #for rightupsideup is -2
        robot.moveanchor()
        robot.translateglobcm(x=(newt[0]*100)-2) # forrightsideup is-2
        robot.moveanchor()
        robot.translateglobcm(z=(newt[2]*100)+safety)#+11   put positive 15 for safety buffer  in rightside up, -15 for upside down
        robot.moveanchor()

# # upsidedown mounted on profile
    elif camera_orientation == 1:
        robot.translateglobcm(y=newt[1]*100) #for rightupsideup is -2
        robot.moveanchor()
        robot.translateglobcm(x=(newt[0]*100)) # forrightsideup is-2
        robot.moveanchor()
        robot.translateglobcm(z=(newt[2]*100)-safety)#+11   put positive 15 for safety buffer  in rightside up, -15 for upside down
        robot.moveanchor()

#rightsideup mounted profile
    elif camera_orientation == 2:
        robot.translateglobcm(y=newt[1]*100-4) #for rightupsideup is -2
        robot.moveanchor()
        robot.translateglobcm(x=(newt[0]*100)) # forrightsideup is-2
        robot.moveanchor()
        robot.translateglobcm(z=(newt[2]*100)-safety)#+11   put positive 15 for safety buffer  in rightside up, -15 for upside down
        robot.moveanchor()

    return

def orientate_arm_task(Rotation,camera_orientation, left):
    rpyorientation = rmat2rpy(Rotation)/3.142*180
    print("RPY for final: ", rpyorientation)

    rotateoncamera(camera_orientation,0,rpyorientation[0])
    rotateoncamera(camera_orientation,1,rpyorientation[1])
    rotateoncamera(camera_orientation,2,rpyorientation[2])

# # """"Rightside up/ upside down read below properly"""
    if  camera_orientation == 0 or camera_orientation == 2:
        if left:
            rotateoncamera(3,90)
        else:
            rotateoncamera(3,-90)
            
    elif  camera_orientation == 1:
        if left:
            rotateoncamera(3,-90)
        else:
            rotateoncamera(3,90)
   
    robot.moveanchor()
    return





#%%
"""
1. Check robot connection T/F
2. Check home distance if you need to perform fixed radial tasks and make sure that the 
       home distance is correct to the physical location of the cone the camera will be rotating around.
3. Check camera connection T/F
4. Check name counter that it is 0 if you want the saving png to be 0000.png. set to 1 if you want 0001.png or
        if you dont want to overwrite the original image.
5. Make sure that the camera orientation AND arm mount is selected correctly

If you are mounting the camera, remember to appropriately offset the origin on the UR10 side
Go into the UR10 and remember to change the fixed axis from 60/-60mm appropriately (with camera)



"""


path = 'temp\\'
robot = robot(connection=True,homedistance = 1000, arm_mounted_camera = False, cameraFlip = True )
camera = mykinectazure(connection=False,namecounter=0)
camera.path = path


camera_orientation_list = {'rightsideup_arm':0,'upsidedown_mounted':1, 'rightsideup_mounted':2}
# rightside up and mounted onto the arm, upside down and mounted onto the profile, ridesideup and mounted onto the profile
camera_orientation = camera_orientation_list['upsidedown_mounted']



#%% Execution

if __name__ == '__main__':
    
    quickCapture(camera)
    
    with open('final_tmat.p', 'rb') as f: 
        dict_tmat_pred = pickle.load(f)
        
    
    tmat_1 = dict_tmat_pred
    
    #FYI: i left this here just in case
    # flip = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # tmat_1 = np.matmul(tmat_1,flip)
    
    Translation = tmat_1[0:3,3]
    Rotation = tmat_1[0:3,0:3]
    print("Camera trans: ",Translation)
    
    robot.wtrJ()
    initialSetup(camera_orientation)
    
    drefframeT=change_ref_frame_trans(robot.anchor)
    newt = np.matmul(Translation,drefframeT)
    print("Robot trans: ",newt)
    
    translate_arm_task(camera_orientation,newt)
    
    orientate_arm_task(Rotation,camera_orientation,left=True)
    
    
    # df.to_csv(path+'output.csv', index = None)
    # # # 
    
    
    
    print("End")






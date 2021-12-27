# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 22:38:50 2021

@author: Dylan Tan
"""
# from PIL import Image
import numpy as np
# import pyk4a
# from pyk4a import Config, PyK4A
import pandas as pd
import math
from robot import robot
from camera import mykinectazure
from geometry import unit, getRmatrix, rotateonspot, rmat2rot,rmat2rpy
from plot import extractlog, plotxy, plotxz, plotyz, plot3d
from matplotlib import pyplot as plt
import time

# from matplotlib import image

#Options: depth_mode=pyk4a.DepthMode.NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_2X2BINNED, 
# if not specificed, color is BGRA 720p

def calibrate(robot):
    # calibrate
    # 9cm on length +5cm for safety, = 14cm
    # 8.9cm for width +5cm for safety = 13.9cm
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


def move(robot,df, h, v,t=6):
    robot.rotateanchor(h,v)
    robot.moveanchor(t=t)
    robot.printanchor()
    df = capturerecord(robot,camera,conetype,df)
    return df

def task(robot,camera,df,direction=1,originkeep = True):

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
    # robot.moveanchor()
    # robot.printanchor()
    # df = capturerecord(robot,camera,conetype,df)
    
    robot.getfixedtranslatebox(interval=interval, dim=dim)
    for i in range(len(robot.translatebox)):
        robot.translateflyer(t=0.5, t2=3, interval=8)
        robot.printflyer()
        df = capturerecord(robot,camera,conetype,df)
        if i==35: return df

    # log = robot.log
    # log = extractlog(log)
    # plot3d(log)
    return df

def task3(robot,camera,df):
    timestep = 12
    totaldeg = 120
    interval = totaldeg / timestep
    df = move(robot,df,60,0)
    for i in range(6):
        for j in range(timestep):
            df = move(robot,df,-interval,0,t=3)
            if(j == (timestep/2)-2):
                robot.wtrJ()
        robot.wtrJ()
        if (i==5):
            robot.wtrJ()
            return df
        else:
            df = move(robot,df,120,10)
                    
    robot.wtrJ()
    return df

def testboundaries(robot,camera,df):
    # robot.rotateanchor(40,0)
    # robot.moveanchor(t=6)
    # box = robot.gettranslatebox()

    # robot.wtr(box[10])
    # df = capturerecord(robot,camera,conetype,df)
    # robot.wtr(box[54])
    # df = capturerecord(robot,camera,conetype,df)
    
    robot.rotateanchor(60,60)
    robot.printanchor()
    robot.moveanchor(t=6)
    box = robot.gettranslatebox()

    robot.wtr(box[10],0.3,0.3,t=9)
    # df = capturerecord(robot,camera,conetype,df)
    robot.wtr(box[54],0.3,0.3,t=9)
    # df = capturerecord(robot,camera,conetype,df)
    print(box[54])
    # wtr(robot,li[0])
# wtr(robot,li[10])
# wtr(robot,li[44])
# wtr(robot,li[54])
    
    
    return df
    

    

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
    drefframeT = creff(home,anchor)                     #relative reference frame Transformation matrix
    
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
    # POSE = np.round(POSE,5)  
    homeorigin = np.matmul(homerefmat,negativepose) # this is the global coordinate syste changed to home cone as the new axis and origin
    homeorigin = np.round(homeorigin,5)  
    
    df = df.append({'Cycle':camera.namecounter,'Cone':conetype, 'RGBname':camera.RGBname,'DEPTHname':camera.DEPTHname, 
                    'RGBD':camera.RGBDname,'Pose':homeorigin, 'RelPose':RelPose, 'Theta':theta} , ignore_index=True)
    
    camera.saveupdate()

    return df


def creff(home,anchor): # change reference frame creff for relative 
    normal = np.array([home[0]-anchor[0],home[1]-anchor[1],home[2]-anchor[2]])
    unorm = unit(normal)
    znorm = np.array(anchor[9:])*-1
    ynorm = np.cross(znorm,unorm)
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(unorm,eye[i]) for i in range(3)],
                        [np.dot(ynorm,eye[i]) for i in range(3)],
                        [np.dot(znorm,eye[i]) for i in range(3)]])
    return refmat



def crefftrans(anchor):
    #this part is fixed. This is arm wrt the camera.  
    #we get instructions from camera, hence need to convert to global arm instruction
    # x = np.array([0,-1,0]) 
    # y = np.array([-0.5,0,-0.8660254037]) 
    # z = np.array([0.8660254037,0,-0.5]) #np.array([1,0,0]) #i know that its the reverse 30 degree, hence i take the last 3 and flip only the x to -ve

    #this method use 
    
    
    #Rightsideup
#     t1 = rotateonspot(np.eye(3),np.array([0,0,1]),90)
# # print(t1)
#     t2 = rotateonspot(np.eye(3),np.array([0,1,0]),60)
# print(t2)

    #Upsidedown
    t1 = rotateonspot(np.eye(3),np.array([0,0,1]),270)
# print(t1)
    t2 = rotateonspot(np.eye(3),np.array([0,1,0]),120)
    
    
    t = np.matmul(t1,t2) 

    
    # eye = np.eye(3)
    
    # refmat = np.array([[np.dot(x,eye[i]) for i in range(3)],
    #                     [np.dot(y,eye[i]) for i in range(3)],
    #                     [np.dot(z,eye[i]) for i in range(3)]])
    # print(refmat)
    
    # return refmat
    
    return t

def creffrot(anchor):
    #this part is fixed. This is arm wrt the camera.  
    #we get instructions from camera, hence need to convert to global arm instruction
    x = np.array([0,-1,0])  *-1
    y = np.array([-0.5,0,-0.8660254037]) *-1
    z = np.array([0.8660254037,0,-0.5]) #np.array([1,0,0]) #i know that its the reverse 30 degree, hence i take the last 3 and flip only the x to -ve

    # x = np.array([0,-1,0])  *-1
    # y = np.array([0.5,0,0.8660254037]) *-1
    # z = np.array([-0.8660254037,0,0.5]) *1

    # x = np.array([0,-0.5,0.866])
    # y = np.array([-1,0,0]) 
    # z = np.array([0.5,-0.866,0])
    
    eye = np.eye(3)
    
    refmat = np.array([[np.dot(x,eye[i]) for i in range(3)],
                        [np.dot(y,eye[i]) for i in range(3)],
                        [np.dot(z,eye[i]) for i in range(3)]])
    return refmat


def rotateonaxis( mat , axis, theta):
    identity = np.eye(3)
    
    
    # zaxis = np.array([0,0,1]) #using z axis

    
    rad = (3.142*theta)/180
    
    W = np.array([[0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]])

    rod = identity + np.sin(rad)*W + (1-np.cos(rad)) * np.matmul(W,W)
    newv = np.matmul(rod,mat)
    return newv    
    
def rotateoncamera(axischoice, theta):
    identity = np.eye(3)
    # camera xaxis in the robot's definition: 
    
    #Original rightside up X and Y axis
    # xaxis = [0,-1,0]
    # yaxis =  [0.5,0,0.866]   #robot.anchor[9:]
    
    #Upsidedown
    xaxis = [0,1,0]
    yaxis =  [0.5,0,-0.866]   #robot.anchor[9:]  
    
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
    # print()
    # self.anchor[9:] = newv
    # self.anchor[8] +=Hrad
    # self.anchor[3:6] = rpy2rot(self.anchor)
    return



path = 'temp\\'
#try 1536p
robot = robot(connection=True,homedistance = 1000)
camera = mykinectazure(connection=True,namecounter=1)
camera.path = path
# camera2 = mykinectazure(conn)

df = pd.DataFrame( columns=['Cycle','Cone','RGBname', 'DEPTHname', 'RGBD', 'Pose', 'RelPose', 'Theta'])
conetype = "C12"


#%% this is moving part

import pickle


with open('final_tmat.p', 'rb') as f: 
    dict_tmat_pred = pickle.load(f)



# tmat_1 = dict_tmat_pred
tmat_1 =np.array([[-0.2727546 ,  0.09546881, -0.95733517, -0.17310054],
                  [-0.55009624,  0.80088506,  0.23659511,  0.17356807],
                  [ 0.78930289,  0.59115888, -0.16592805, -0.66807057],
                  [ 0.        ,  0.        ,  0.        ,  1.        ]])


tmat1trans = tmat_1[0:3,3]
tmat_1rot = tmat_1[0:3,0:3]

Translation = tmat1trans
Rotation=  tmat_1rot

robot.wtrJ()
print("Camera trans: ",Translation)

##this is for the original orientation
# robot.rotateonspot(thetaV=30)
# robot.translateglobcm(x=20)
# robot.moveanchor()

##this is for upside down orientation
# robot.rotateonspot(thetaV=-30)
# robot.translateglobcm(x=20)
# robot.translateglobcm(z=-7)

robot.rotateonspot(thetaV=-30)
robot.translateglobcm(x=3) #originially 3.9
robot.translateglobcm(z=-26.8)
# robot.moveanchor()

drefframeT=crefftrans(robot.anchor)
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
# # robot.moveanchor()
# robot.translateglobcm(x=(newt[0]*100)) # forrightsideup is-2
# # robot.moveanchor()
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

# # """"Rightside up/ upside down read below properly"""
# rotateoncamera(3,-90) 
# robot.moveanchor()
###############################################################################
# # """ For original left images, 90 is correct, for flipped, take note it will likely be -90
# """ For upside down: -90 for L and 90 for flipped


# # #take note that the original is 90, but after nipuni, it changed to -90 as it 
# # seems axis definition has kinda changed"""

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
df = capturerecord(robot,camera,conetype,df)


#%%capture images


# for j in range(15):
#     for i in range(4):
#         time.sleep(1)
#         print(i)
#     print("capturing image",j)
#     df = capturerecord(robot,camera,conetype,df)
#     print("Done image",j)
    
# time.sleep(10)

#%%calibrate check
# calibrate(robot)

# robot.wtrJ()

# # # robot.moveanchor()
# robot.translateglobcm(x=70)
# robot.moveanchor()
# robot.translateglobcm(z=-29) 
# robot.moveanchor()
# time.sleep(5)
# robot.wtrJ()
# ##### robot.translateglobcm(y=40)
# robot.moveanchor()
# %% Tasks

# robot.wtrJ()
# # robot.moveanchor()
# robot.rotateonspot(thetaV=30)
# robot.moveanchor()
# robot.translateglobcm(x=20)
# robot.moveanchor()

# df = capturerecord(robot,camera,conetype,df)

# df = task2(robot,camera,df,interval=8 , dim=0.8)

# robot.translateglobcm(x=-10)
# robot.moveanchor()
# df2 = task2(robot,camera,df,interval=8 , dim=0.8)

# robot.translateglobcm(x=-10)
# robot.moveanchor()
# df3 = task2(robot,camera,df,interval=8 , dim=0.8)
# [0.18,0.04,-0.34]

df.to_csv(path+'output.csv', index = None)
# # # 



print("End")






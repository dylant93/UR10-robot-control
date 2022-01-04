# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 22:39:35 2021

@author: dylan tan
"""

import socket
import time
from math import pi as pi
import numpy as np
from geometry import rpy2rot, unit, getRmatrix, rmat2rpy, rotateonaxis


"""The robot is offcenter adjusted by -60 in the x axis and 136 in the z axis for upside orientation
For upside down orientation use +60 in the x axis
"""

class robot():
    
    """
    This class is used for UR10 robotic arm. 
    
    """
    homeJoint = (28.04,-69.40,-133.33,22.82,61.34,89.96) 
    #(from base, 550,0,550) but is using upside down pointer with offset of 45mm, 0mm and 105mm x,y,z
    
    # homeJoint_newcamera = (31.07,-64.4,-134.49,18.98,58.31,89.94) #(from base, x,y,z is 550, 0, 550) use this if from upsie down camera reference
    
    homeJoint_database=(23.0,-105.0,-143.0,68.46,66.26,90)   #a safe joint posiion described in joint movements so as to avoid ur10 resolving 
                                                    #the positions weirdly if you use the 'move position' command
    k = pi/180
    
    #subnet = 255.255.255.0
    def __init__(self,IP="192.168.1.1",PORT=30002,connection=True,homedistance=1000, arm_mounted_camera = False, cameraFlip = True ):
        self.HOST_ROBOT =  IP
        self.PORT_ROBOT = PORT
        self.home = self.FL(homedistance,0,550,0,90,0) #old is 360  
        self.anchor = self.FL(550,0,550,0,90,0)
        self.flyer = self.anchor.copy()
        self.translatebox = self.gettranslatebox()
        self.translateboxinterval = 10
        self.boxpos = 0
        self.log = np.array(self.home)
        self.printformat = {'XYZ':self.flyer[:3],'Quart':self.flyer[3:6],'RPY':self.flyer[6:9],'VAxis':self.flyer[9:]}
        self.connection = connection
        
        if(arm_mounted_camera):
            self.homeJoint = self.homeJoint_database
        
        print("Starting Program.")
        if(self.connection):
            self.robot = socket.create_connection((self.HOST_ROBOT, self.PORT_ROBOT)) 
            time.sleep(1) # wait for 2 second
            print("Connected.")
        else:
            print("Robot is offline, continuing program")
    
    def FL(self,x,y,z,rx,ry,rz): 
        """Formatted List(FL).
        Takes inputs in mm and degrees, outputs in meters and rads formatted list
        This is the format that the UR10 will accept
        """
        
        outputlist = [round(x/1000,2),round(y/1000,2),round(z/1000,2),round(rx*self.k,2),round(ry*self.k,2),round(rz*self.k,2)]
        [outputlist.append(outputlist[i]) for i in range(3,6)]
        outputlist = outputlist+[0,0,1]
        return outputlist
    
    
    def printanchor(self):
        self.printformat = {'XYZ': [round(i,2) for i in self.anchor[:3]],
                            'Quart': [round(i,2) for i in self.anchor[3:6]],
                            'RPY': [round(i,2) for i in self.anchor[6:9]],
                            'VAxis': [round(i,2) for i in self.anchor[9:]]}
        # testing()
        print(self.printformat)
        
        
    
    def wtrJ(self,a=0.7,v=0.9,r=0,t=6): 
        """Write to robot via joint positions
        This is a safe bet to start as a sort of home position because writing by coordinate positions
        will sometimes have the robot move in strange ways from its internal calculations.
        This will NOT update anchors as it is necessary for certain movement types
        """
        
        pos = self.homeJoint
        string = 'movej([%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],a=%.2f,v=%.2f,r=%.2f)\n'%(pos[0]*self.k,pos[1]*self.k,pos[2]*self.k,pos[3]*self.k,pos[4]*self.k,pos[5]*self.k,a,v,r)
        print("Home joint")
        bout = str.encode(string)
        if(self.connection):
            self.robot.send(bout)
            time.sleep(t)
        return
    
    
    def wtr(self,pos,a=0.7,v=0.7,r=0,t=6): 
        """
        Parameters
        ----------
        pos : TYPE
            DESCRIPTION. len 9 list of formatted list
        a : TYPE, float
            DESCRIPTION. The default is 0.7. This is acceleration of the robot arm movement
        v : TYPE, float
            DESCRIPTION. The default is 0.7. This is velocity of arm movement
        r : TYPE, float
            DESCRIPTION. The default is 0.
        t : TYPE, optional
            DESCRIPTION. The default is 6. this is the sleep time to make sure that the movement is completed

        Returns
        -------
        None.

        """

        if(pos[0] <0.56 and pos[2] < 0.19):
            print("Did not carry out move, collision DANGER")
            return
    
        string = 'movej(p[{!s},{!s},{!s},{!s},{!s},{!s}],a={!s},v={!s},r={!s})\n'.format(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],a,v,r)
        bout = str.encode(string)
        # self.logscatter(pos)
        # self.log.append(pos)
        self.log=np.vstack([self.log,pos])
        if(self.connection):
            self.robot.send(bout)
            time.sleep(t)
            
        return
    
    def moveanchor(self,a=0.7,v=0.7,r=0,t=6):
        """
        

        Parameters
        ----------
        a : TYPE, float
            DESCRIPTION. The default is 0.7. This is acceleration of the robot arm movement
        v : TYPE, float
            DESCRIPTION. The default is 0.7. This is velocity of arm movement
        r : TYPE, float
            DESCRIPTION. The default is 0.
        t : TYPE, optional
            DESCRIPTION. The default is 6. this is the sleep time to make sure that the movement is completed

        Returns
        -------
        None.

        """
        self.wtr(self.anchor,a,v,r,t)
        self.flyer = self.anchor.copy()
        # self.
        return
    
    def getaxis(self):
        """
        

        Returns
        -------
        axis orientation of current anchor position
        normal, vert and horizontal
        
        if you were curious, you might notice that that its strange that vertical axis for getaxis is the 0th column (x axis)
        while original robot vertical axis should be the 2th column (z axis)
        This is because the arm is rotated 90 degrees about its y axis for ease of handling the camera.
        Hence rotating its local orientation axis such that z is now pointing towards home and x is pointing down.
        Resulting in the columns dictated for vert, hori and normal
        """
        rotation_matrix = getRmatrix(self.anchor[6],self.anchor[7],self.anchor[8])
        rotation_matrix[0:,0] = rotation_matrix[0:,0]*-1
        vert = np.squeeze(np.asarray(rotation_matrix[0:,0]))
        hori = np.squeeze(np.asarray(rotation_matrix[0:,1]))
        normal = np.squeeze(np.asarray(rotation_matrix[0:,2]))
        
        return normal,vert,hori
    
    
    
    def rotateonspot_local(self,thetaH=0,thetaV=0):
        """
        

        Parameters
        ----------
        thetaH : TYPE, int
            DESCRIPTION. The default is 0. Degrees to rotate horizontally (around local verical axis)
            
        thetaV : TYPE, int
            DESCRIPTION. The default is 0. Degrees to rotate vertically (around local horizontal axis)
            
    
        Returns
        None.
        
        Take note that This function rotates about arm's origin point.

        """
        
        if(thetaH!=0):
            #you can use getaxis to get the normal and horizontal axis as well.
            rotation_matrix = getRmatrix(self.anchor[6],self.anchor[7],self.anchor[8])
            newnorm = rotation_matrix[0:,2]          
            newnorm = np.squeeze(np.asarray(newnorm))

            vaxis = self.anchor[9:]
            haxis = np.cross(vaxis,newnorm)
            
            
            Hrad = (pi*thetaH)/180
            newv = rotateonaxis(vaxis,vaxis,thetaH)
            
            self.anchor[9:] = newv
            self.anchor[8] +=Hrad
            self.anchor[3:6] = rpy2rot(self.anchor)
    
        
        if(thetaV!=0):
            
            rotation_matrix = getRmatrix(self.anchor[6],self.anchor[7],self.anchor[8])
            newnorm = rotation_matrix[0:,2]
            newnorm = np.squeeze(np.asarray(newnorm))

            vaxis = self.anchor[9:]
            haxis = np.cross(vaxis,newnorm)
            
            
            Vrad = (pi*thetaV)/180
            newv = rotateonaxis(vaxis,haxis,thetaV)
            
            self.anchor[9:] = newv
            self.anchor[7] += Vrad
            self.anchor[3:6] = rpy2rot(self.anchor)
        
        return
    

    
    def translateglobcm(self,x=0,y=0,z=0):
        """
        

        Parameters
        ----------
        x : TYPE, optional
            DESCRIPTION. The default is 0.
        y : TYPE, optional
            DESCRIPTION. The default is 0.
        z : TYPE, optional
            DESCRIPTION. The default is 0.

        Returns
        -------
        None.

        """
        self.anchor[0] += x/100
        self.anchor[1] += y/100
        self.anchor[2] += z/100
        
        return
    
    
    def rotateanchor(self,thetaH=0,thetaV=0): 
        
        """
        Rotate anchor about the home position in degrees, local Horizontal and global Vertical axis specific for ease of use
        This will move the anchor's coordinate position as well, not just the orientation. It will move in a fixed radius arc, so the
        orientation and the camera will move always facing the home position.'
        """

        identity = np.eye(3)
        
        
  
        if(thetaH!=0):
            normal = [self.home[0]-self.anchor[0],self.home[1]-self.anchor[1],self.home[2]-self.anchor[2]]
            unitnormal = unit(np.array(normal))        
            vaxis = self.anchor[9:]
            zaxis = np.array([0,0,1]) #using z axis
            haxis = np.cross(zaxis,unitnormal)

            adjusted = np.array([self.anchor[0]-self.home[0],self.anchor[1]-self.home[1],self.anchor[2]-self.home[2]])
            Hrad = -(pi*thetaH)/180
            
            W = np.array([[0, -zaxis[2], zaxis[1]],
                       [zaxis[2], 0, -zaxis[0]],
                       [-zaxis[1], zaxis[0], 0]])
        
            rod = identity + np.sin(Hrad)*W + (1-np.cos(Hrad)) * np.matmul(W,W)
            output = np.matmul(rod,adjusted)
            
            output[0]+=self.home[0]
            output[1]+=self.home[1]
            output[2]+=self.home[2]
            
            self.anchor[:3] = list(output)
            newv = np.matmul(rod,vaxis)
            
            
            self.anchor[9:] = newv
            self.anchor[8] +=Hrad
            self.anchor[3:6] = rpy2rot(self.anchor)
    
        if(thetaV!=0):
            normal = [self.home[0]-self.anchor[0],self.home[1]-self.anchor[1],self.home[2]-self.anchor[2]]
            unitnormal = unit(np.array(normal))
            vaxis = self.anchor[9:]
            haxis = np.cross(vaxis,unitnormal)
            # print("rotateanchor\t\t",unitnormal)
            
            adjusted = np.array([self.anchor[0]-self.home[0],self.anchor[1]-self.home[1],self.anchor[2]-self.home[2]])
            Vrad = (pi*thetaV)/180
            W = np.array([[0, -haxis[2], haxis[1]],
                       [haxis[2], 0, -haxis[0]],
                       [-haxis[1], haxis[0], 0]])
        
            rod = identity + np.sin(Vrad)*W + (1-np.cos(Vrad)) * np.matmul(W,W)
            output = np.matmul(rod,adjusted)
            output[0]+=self.home[0]
            output[1]+=self.home[1]
            output[2]+=self.home[2]
            self.anchor[:3] = list(output)
            newv = np.matmul(rod,np.array(vaxis))
            
            
            self.anchor[9:] = newv
            self.anchor[7]+=Vrad
            self.anchor[3:6] = rpy2rot(self.anchor)
            



        return 
    #%% 
    """
    These functions are used to perform a sequence for data collection where the anchor rotates at a fixed radius around the object
    and the flyer translates in a 2d box to capture data for each angle of the anchor
    Hence the name anchor and flyer.
    """
    
    def gettranslatebox(self, interval = 10, dim = 0.5): #get translation box from anchor. This will generate a list of positions based on the dimension
                                         #input. 
        # zaxis = np.array([0,0,1])
        normal = [self.home[0]-self.anchor[0],self.home[1]-self.anchor[1],self.home[2]-self.anchor[2]]
        unitnormal = unit(np.array(normal))
        vaxis = self.anchor[9:]
    
        haxis = np.cross(vaxis,unitnormal)

        output=[]           
        interval = interval
        
        for v in range(5):
            for h in range(-int(interval/2),int(interval/2)+1):
                trans = self.anchor
                trans=[self.anchor[i]+(round(vaxis[i]*v*dim/interval,4)+round(haxis[i]*h*dim/interval,4)) for i in range(3)]
                trans+=self.anchor[3:]
                output.append(trans)
        
        self.translatebox = output
        self.boxpos = 0
        return output
    
    def getfixedtranslatebox(self,interval = 10, dim = 0.5): #get translation box from anchor. This will generate a list of positions based on the dimension
                                         #input. 
        # zaxis = np.array([0,0,1])
        normal = [self.home[0]-self.anchor[0],self.home[1]-self.anchor[1],self.home[2]-self.anchor[2]]
        unitnormal = unit(np.array(normal))
        vaxis = np.array([0,0,1]) #self.anchor[9:]
        
    
        haxis = np.cross(vaxis,unitnormal)

        output=[]           
        interval = interval
        
        for v in range(5):
            for h in range(-int(interval/2),int(interval/2)+1):
                trans = self.anchor
                trans=[self.anchor[i]+(round(vaxis[i]*v*dim/interval,4)+round(haxis[i]*h*dim/interval,4)) for i in range(3)]
                trans+=self.anchor[3:]
                output.append(trans)
        
        self.translatebox = output
        self.boxpos = 0
        return output
    
    def translateflyer(self,dim = 0.5,t=3,t2=6,interval = 10): 
        self.flyer = self.translatebox[self.boxpos]
        if self.boxpos%(interval+1) == 0:
            self.wtr(self.flyer,t=t2) 
        else: self.wtr(self.flyer,t=t)
        self.boxpos+=1
        

    def printflyer(self):
        self.printformat = {'XYZ': [round(i,2) for i in self.flyer[:3]],
                            'Quart': [round(i,2) for i in self.flyer[3:6]],
                            'RPY': [round(i,2) for i in self.flyer[6:9]],
                            'VAxis': [round(i,2) for i in self.flyer[9:]]}
        # testing()
        print(self.printformat)
    
            
    
    
        
    
    
    





        

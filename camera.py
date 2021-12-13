# -*- coding: utf-8 -*-
"""
Created on Thu Feb 25 15:25:37 2021

@author: dylan tan
"""
import pyk4a
from pyk4a import Config, PyK4A
import numpy as np
from PIL import Image
import time
from matplotlib import pyplot as plt
import cv2

class mykinectazure():
    
    
    resolution = [[720,1280],[3072,4096],[1536,2048]]
    

    path = 'temp\\'
    RGBpath = 'rgb\\'
    depthpath = 'depth\\'
    npypath = 'npy\\'
    visual = 'depthvisual\\'


    def __init__(self, connection = True, res = 1, namecounter = 0):
        self.k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_3072P,
                    camera_fps=pyk4a.FPS.FPS_15,           
                    depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,))  # If using WFOV, camera_FPS must be specified, default 30fps will not work
                                                                 # NFOV_2X2BINNED                    # camera_fps=pyk4a.FPS.FPS_15,
        self.connection = connection
        
        if(not self.connection):
            print("Camera is offline, proceeding")
        self.res = self.resolution[res]
        self.namecounter = namecounter
        self.stringcounter = str(self.namecounter).zfill(4)
        self.RGBname = self.RGBpath + self.stringcounter + ".png"
        self.DEPTHname = self.depthpath + self.stringcounter + ".png"
        self.RGBDname = self.npypath + self.stringcounter
        
        # self.RGBname = "RGB{!s}.png" .format(namecounter)
        # self.DEPTHname = "D{!s}.png" .format(namecounter)
        # self.RGBDname = "RGBD{!s}" .format(namecounter)
        
        
    def cap(self):
        if(self.connection):
            self.k4a.start()
            # time.sleep(1)
            capture = self.k4a.get_capture()
            self.k4a.stop()
    
            self.img_color = capture.color
            self.img_color = self.img_color[:, :, 2::-1]
            self.img_deptht = capture.transformed_depth
        
            img_deptht_reshape=np.reshape(self.img_deptht,(self.res[0],self.res[1],1))
            self.RGBD = np.concatenate((self.img_color,img_deptht_reshape),axis=2)
            return self.img_color, self.img_deptht, self.RGBD
        return
    
    def updatecounter(self):
        self.namecounter +=1
        self.stringcounter = str(self.namecounter).zfill(4)
        
        self.RGBname = self.RGBpath + self.stringcounter + ".png"
        self.DEPTHname = self.depthpath + self.stringcounter + ".png"
        self.RGBDname = self.npypath + self.stringcounter
        
        
        # self.RGBname = "RGB{!s}.jpeg" .format(self.namecounter)
        # self.DEPTHname = "D{!s}.png" .format(self.namecounter)
        # self.RGBDname = "RGBD{!s}" .format(self.namecounter)
        return
    
    def getintrinsics(self,path):
        if(self.connection):
            self.k4a.start()
            self.k4a.save_calibration_json(path)
            self.k4a.stop()

        return
    
    def savejpeg(self):
        if(not self.connection): return
            
        im = Image.fromarray(self.img_color)
        im.save(self.path+self.RGBname)
        print(self.img_color.shape)
        # cv2.imwrite(self.path+self.RGBname, self.img_color)
        return
            
        
    def savepng(self):
        if(not self.connection): return
        
        im = Image.fromarray(self.img_deptht)
        im.save(self.path+self.DEPTHname)
        # cv2.imwrite(self.path+self.DEPTHname, self.img_deptht)
        return
    
    def saveandvisualizepng(self):
        if(not self.connection): return
        im = Image.fromarray(self.img_color)
        plt.imshow(im)
        # plt.savefig(self.path+self.visual+str(self.namecounter).zfill(4)+'.png')
        print("Press Q to quit")
        plt.show()
        
        imd = Image.fromarray(self.img_deptht)
        plt.imshow(imd,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
        plt.clim(0,5000)
        plt.colorbar()
        plt.savefig(self.path+self.visual+str(self.namecounter).zfill(4)+'.png')
        print("Press Q again to quit")
        plt.show()
        

        
        
        return
        
    def savenpz(self):
        if(not self.connection): return
        
        np.savez_compressed(self.path+self.RGBDname, self.RGBD)
        return
        
    def saveupdate(self, jpeg = True, png = True, npz = False, visual = False):
        if(not self.connection): 
            self.updatecounter()
            return
        
        if jpeg: self.savejpeg()
        if png: self.savepng()
        if npz: self.savenpz()
        if visual: self.saveandvisualizepng()
        
        self.updatecounter()
        return
    
# import time
# camera = mykinectazure()
# camera.getintrinsics("config.json")
# camera.start()
# a = camera.calibration()

# camera.namecounter = 6

# camera.cap()   
# camera.saveupdate()
# print("next")
# time.sleep(1)

# camera.cap()   
# camera.saveupdate()
# print("next")
# time.sleep(1)

# camera.cap()   
# camera.saveupdate()
# print("next")
# time.sleep(1)

# camera.cap()   
# camera.saveupdate()
# print("next")
# time.sleep(1)

# camera.cap()   
# camera.saveupdate()
# print("next")
# time.sleep(1)

        
        
        
        
        
        

            

    
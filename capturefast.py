# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 01:22:54 2021

@author: dylantan1993@gmail.com

"""

import matplotlib.pyplot as plt
import pyk4a
from pyk4a import Config, PyK4A
# from camera import mykinectazure
# from loadimage import importpngoriginal
# from matplotlib import image
import numpy as np
from PIL import Image
import cv2



k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_3072P,
                    camera_fps=pyk4a.FPS.FPS_15,           
                    depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,))



k4a.start()


scale = 8
res = (3072,4096)
dim = (int(res[1]/scale),int(res[0]/scale))
counter = 0
path = "temp\\"
RGBpath = 'rgb\\'
depthpath = 'depth\\'

while 1:
    capture = k4a.get_capture()
    if np.any(capture.color):
        resized = cv2.resize(capture.color, dim, interpolation = cv2.INTER_AREA)
        resized = cv2.putText(resized, 'Time: {}'.format(counter), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA) 
        cv2.imshow("k4a", resized[:, :, :3])
        # key = cv2.waitKey(10)
        key = cv2.waitKey(10) & 0xFF

    if key == ord('q'):
        break
    
    if key== ord(' '):
        
        # img_color = capture.color
         
        #Use this with image lib, its the exact one we used but its really slow
        # img_color = capture.color[:, :, 2::-1]
        
        #use this with cv2, stands to be tested, but is much faster
        img_color = capture.color[:,:,:3]
        img_deptht = capture.transformed_depth
        
        stringcounter = str(counter).zfill(4)
        rgbname = path + RGBpath+ stringcounter + ".png"
        DEPTHname = path + depthpath + stringcounter + ".png"
        
        cv2.imwrite(rgbname, img_color)
        # imc = Image.fromarray(img_color)
        # imc.save(rgbname)
        
        cv2.imwrite(DEPTHname, img_deptht)
        # imd = Image.fromarray(img_deptht)
        # imd.save(DEPTHname)
        
        
        plt.imshow(img_deptht,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
        plt.clim(0,5000)
        plt.colorbar()
        plt.show()
        
        
        print("Captured: ",counter)
        counter+=1
  

cv2.destroyAllWindows()
k4a.stop()



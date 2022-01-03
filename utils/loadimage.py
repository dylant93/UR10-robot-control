# -*- coding: utf-8 -*-
"""
Created on Mon Feb 22 18:53:12 2021

@author: dylantan1993
"""
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import image
# from pyk4a import 
from pyk4a import depth_image_to_color_camera



def importpngoriginal(pngpath):
    """   
    Parameters
    ----------
    pngpath : path to png


    Returns
    -------
    uint16 nparray in original format that azure kinect gives

    """
    png = image.imread(pngpath)
    output = (png*65535).astype(np.uint16)
    output = np.clip(output,0,5000)
    return output


def toRGBD(RGB,Depth):
    """
    Parameters
    ----------
    RGB : RGB numpy array 
        
    Depth : Depth numpy array
        .

    A four channel RGBD numpy array
    -------
    

    """
    Depth=np.reshape(Depth,(Depth.shape[0],Depth.shape[1],1))
    RGBD = np.concatenate((RGB,Depth),axis=2)
    return RGBD

def extractdepth(RGBD):
    """
    Parameters
    ----------
    RGBD : np_arr
        takes in RGBD input numpy array.

    Returns
    -------
    depth : np_arr
        returns only the depth channel of the RGBD input.

    """
    depth = RGBD[:,:,3]
    return depth





# depth = importpngoriginal('G:\\Dylan\\temp\\D1.png')
# depth = importpngoriginal('Linemod_and_Occlusion\\Linemod_preprocessed\\data\\01\\depth\\0000.png')

# depthpng = importpngoriginal('S05_3/depth/0000.png')


# print(depthpng)
# depth = importpngoriginal('mask_resize/mask_resize/depth/0000.png')

# print(depth.shape)
# depth_image_to_color_camera(
# rgb = image.imread("temp\\RGB1.jpeg")
# print(rgb.shape)
# RGBDepth = toRGBD(rgb, depth)
# print(RGBDepth.shape)

# alpha = extractdepth(RGBDepth)

# plt.imshow(colorload) # BGRA to RGB , cmap="hot"

# plt.show()

# plt.imshow(depthpng,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
# plt.clim(0,5000)
# plt.colorbar()
# plt.savefig('S05_3/depthvisual/0000.png')

# plt.show()


# for i in range(1):
#     depthpng = importpngoriginal('S05_3/depth/'+str(i).zfill(4)+'.png')
#     plt.imshow(depthpng,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
#     plt.clim(0,5000)
#     plt.colorbar()
#     plt.savefig('S05_3/depthvisual/'+str(i).zfill(4)+'.png')
#     plt.show()
    




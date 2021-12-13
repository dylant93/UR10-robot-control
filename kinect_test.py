# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 15:37:32 2021

@author: holy_
"""
#https://github.com/etiennedub/pyk4a/tree/146a250b917bba2a63ed3fd63a1a3ebf353893c8
# taken from examples



from PIL import Image
import numpy as np
import pyk4a
from pyk4a import Config, PyK4A
from matplotlib import pyplot as plt
from matplotlib import image

# Load camera with the default config  depth_mode=pyk4a.DepthMode.NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_2X2BINNED, 
# if not specificed, color is BGRA 720p

res2160=[2160,3840]
res3072=[3072,4096]
res = res3072

k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_3072P,
                   camera_fps=pyk4a.FPS.FPS_15,
                   depth_mode=pyk4a.DepthMode.WFOV_UNBINNED,))

k4a.start()

# Get the next capture (blocking function)

capture = k4a.get_capture()
k4a.stop()

img_color = capture.color
img_color = img_color[:, :, 2::-1]
# k4a.stop()

img_depth = capture.depth
img_deptht = capture.transformed_depth
img_colort = capture.transformed_color
print("colour: ",img_color.shape,"depth: ",img_depth.shape,"Trans_D: ",img_deptht.shape,"Trans_C: ",img_colort.shape)


points = capture.depth_point_cloud.reshape((-1, 3))
colors = capture.transformed_color[..., (2, 1, 0)].reshape((-1, 3))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
# ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c=colors / 255,)

# Display with pyplot


############remove alpha channel 

def extractalpha(img):
    alpha = []
    row = []
    for i in range(img_colort.shape[0]):
        for j in range(img_colort.shape[1]):
            row.append(img_colort[i][j][3])
        alpha.append(row)
        row=[]
        
    alpha = np.array(alpha)
    return alpha

###########################################################convert bgr to rgb with alpha channel in tact
# img_colort[:,:,:3] = img_colort[:,:,2::-1]

# img_colort = img_colort[:,:,:3]
# # print(img_colort.shape)
# plt.imshow(img_colort) # BGRA to RGB , cmap="hot"
# plt.clim(0,5000)
# plt.colorbar()
# plt.show()
# ############################################################extract alpha and display
# # alpha = extractalpha(img_colort)

# # plt.imshow(alpha) # BGRA to RGB , cmap="hot"
# # plt.clim(0,255)
# # plt.colorbar()
# # plt.show()

# img_color = img_color[::-1]
#############################################################################

plt.imshow(img_color,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
plt.clim(0,5000)
plt.colorbar()
plt.show()

imd8 = (img_deptht/20).astype('uint8')
plt.imshow(imd8,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
plt.clim(0,255)
plt.colorbar()
plt.show()


img_deptht2=np.reshape(img_deptht,(res[0],res[1],1))
print(img_color.shape,img_deptht2.shape)
RGBD = np.concatenate((img_color,img_deptht2),axis=2)
print(RGBD.shape)

plt.imshow(RGBD,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
plt.clim(0,5000)
plt.colorbar()
plt.show()

# print(imd8[600])

# np.save("RGBD", RGBD)

np.savez_compressed('RGBDa', RGBD)
loaded = np.load('RGBDa.npz')
print(loaded['arr_0'].shape)
##############################################

# loaded = np.load("temp\\RGBD1.npy")
# loadedrgb = loaded[:,:,:3]
# loadedd = loaded[:,:,3]

# plt.imshow(loadedrgb) # BGRA to RGB , cmap="hot"
# plt.show()

# plt.imshow(loadedd,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
# plt.clim(0,5000)
# plt.colorbar()
# plt.show()
# print(loadedd)

###################################

# print(img_colort.shape)
# plt.imshow(img_colourt,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
# plt.clim(0,5000)
# plt.colorbar()
# plt.show()




# im = Image.fromarray(img_deptht)
# im.save("001d.png")

# print(img_depth[0][0])
# imd8 = (img_deptht/20).astype(np.uint8)  # 5000/256 = ~20 this is because kinects max is 5000 'uint8'

im = Image.fromarray(img_deptht)
im.save("001d.png")
print(img_deptht)

# depimg = Image.open('001d.png')
# print(depimg)

depimg = image.imread('001d.png')

depimg = (depimg*65535).astype(np.uint16)
# depimg = depimg * 20
print(depimg)
# plt.imshow(image,cmap="nipy_spectral") # BGRA to RGB , cmap="hot"
# plt.clim(0,256)
# plt.colorbar()
# plt.show()

# print(image[0])

# print(img_color.shape)
# plt.imshow(img_color)
# plt.show()

#     # getters and setters directly get and set on device
#     k4a.whitebalance = 4500
#     assert k4a.whitebalance == 4500
#     k4a.whitebalance = 4510
#     assert k4a.whitebalance == 4510
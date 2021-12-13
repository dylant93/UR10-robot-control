# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 22:38:50 2021

@author: holy_
"""
from PIL import Image
import pyk4a
from pyk4a import Config, PyK4A #, depth_image_to_color_camera
from matplotlib import pyplot as plt
import k4a_module
# from matplotlib import image

#Options: depth_mode=pyk4a.DepthMode.NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_2X2BINNED, 
# if not specificed, color is BGRA 720p

# k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_3072P,
#                     camera_fps=pyk4a.FPS.FPS_15,
#                     depth_mode=pyk4a.DepthMode.WFOV_UNBINNED,))  # If using WFOV, camera_FPS must be specified, default 30fps will not work

# k4a = PyK4A(Config(depth_mode=pyk4a.DepthMode.NFOV_2X2BINNED,)) #if you want to compare with previous images i took use this mode and resolution
k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_1536P,depth_mode=pyk4a.DepthMode.NFOV_2X2BINNED,))
k4a.start()
capture = k4a.get_capture()
# k4a.save_calibration_json("Config.json")
# a = k4a.calibration_raw
# a = k4a._configuration()
# a=k4a.calibration()
# print(a)
# res, calibration_handle = k4a_module.device_get_calibration( depth_mode, color_resolution)
# print(k4a.color_resolution())
k4a.stop()

img_color = capture.color
img_color = img_color[:, :, 2::-1]  #flip GBR to RGB
img_depth = capture.depth
# depth_image_to_color_camera()
plt.imshow(img_depth,cmap="nipy_spectral") 
plt.clim(0,5000)
plt.colorbar()
plt.show()

plt.imshow(img_color)
plt.show()


im = Image.fromarray(img_color)
im.save("001c.jpeg")
#save image as jpeg

imd8 = (img_depth/20).astype('uint8')  # 5000/256 = ~20 this is because kinects max is 5000 
im = Image.fromarray(imd8)             #save depth image as 8bit jpeg
im.save("001d.jpeg")




# np.save("RGB", img_color) numpysave
# np.save("D", img_depth) numpysave


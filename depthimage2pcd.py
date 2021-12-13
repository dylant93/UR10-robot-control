# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 13:26:11 2021

@author: dylantan1993@gmail.com

"""
import open3d as o3d

def depth2pointcloud(number,path, depth_trunc=0.7):

    depth_raw = o3d.io.read_image(path + "depth/{!s}.png".format(str(number).zfill(4)))
    color_raw = o3d.io.read_image(path + "rgb/{!s}.png".format(str(number).zfill(4)))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, depth_scale = 1000, depth_trunc = depth_trunc) #@depthscale==1, trunc=5000, @scale==1000, depth trunc = 5
    
    # [303.82123, 0.0, 325.2611, 0.0, 303.82123, 242.04899, 0.0, 0.0, 1.0]  our linemod intrinsic value
    # [1944.4559326171875, 0.0, 2049.344970703125, 0.0, 1943.5645751953125, 1556.814453125, 0.0, 0.0, 1.0] Chinthaka value
    
    
    intrin = o3d.camera.PinholeCameraIntrinsic(4096,3072,1944.4559326171875,1943.5645751953125,2049.344970703125,1556.814453125)
    # intrin = o3d.camera.PinholeCameraIntrinsic(640,480,303.82123,303.82123,325.2611,242.04899) #intrinsic for 640 480 data

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image, intrinsic = intrin )
    return pcd


number = 0                                  #number to iterate down 0000.png, 0001.png ...
target = depth2pointcloud(number,'S05_3/')  #S05_3/ input path relative to pythonfile


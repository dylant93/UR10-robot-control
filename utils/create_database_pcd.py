# -*- coding: utf-8 -*-
"""
Created on Wed May 26 17:59:51 2021

@author: dylantan1993@gmail.com

"""
import open3d as o3d


def depth2pointcloud(number,path, depth_trunc=0.7):
    # print('pcd2, target: ',number)
    depth_raw = o3d.io.read_image(path + "depth/{!s}.png".format(str(number).zfill(4)))
    color_raw = o3d.io.read_image(path + "rgb/{!s}.png".format(str(number).zfill(4)))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, depth_scale = 1000, depth_trunc = depth_trunc) #original will be 5000
    # [303.82123, 0.0, 325.2611, 0.0, 303.82123, 242.04899, 0.0, 0.0, 1.0]  our linemod value
    # [1944.4559326171875, 0.0, 2049.344970703125, 0.0, 1943.5645751953125, 1556.814453125, 0.0, 0.0, 1.0] Chinthaka value
    
    
    intrin = o3d.camera.PinholeCameraIntrinsic(4096,3072,1944.4559326171875,1943.5645751953125,2049.344970703125,1556.814453125)
    # intrin = o3d.camera.PinholeCameraIntrinsic(640,480,303.82123,303.82123,325.2611,242.04899) #intrinsic for 640 480 data

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image, intrinsic = intrin )
    return pcd

    
##################################################################################################################
# %%
def savePCD(outputname,number=0,path="parallelroute2/"):
    number = number #0
    target = depth2pointcloud(number,path) #S05_3/ parallelroute/S05/WFOVS05/
    o3d.io.write_point_cloud(outputname+str(number)+'.ply', target, write_ascii=True, compressed=False, print_progress=False)
    print("main template saved!")
    return



if __name__ == '__main__':
    outpath = '../definedpcds/test' #parallelroute/S05/WFOVS05/  #S05_3/
    depthpath = "../temp/"
    savePCD(outpath,0,depthpath)
    
    
    
    
    
    

           
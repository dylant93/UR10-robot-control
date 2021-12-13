# -*- coding: utf-8 -*-
"""
Created on Fri Aug  6 14:59:18 2021

@author: dylantan1993@gmail.com

"""
import numpy as np
import pickle
import open3d as o3d
import copy

class visualize():
    
    def __init__(self,path,picklename,cadpath):
        
        self.path = path
        self.picklename = picklename
        self.dict_tmat_pred = self.importpickle()
        self.number = 0
        self.depth_trunc = 0.7
        self.depth_scale = 1000
        self.yellow = [1, 0.706, 0]
        self.intrinsicmatrix = o3d.camera.PinholeCameraIntrinsic(4096,3072,1944.4559326171875,1943.5645751953125,2049.344970703125,1556.814453125)
        self.source = o3d.io.read_point_cloud(cadpath)
        self.target = self.createtarget()
        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])  
        self.userinput_tmat = np.array([[-0.28652734, -0.03644933, -0.95737847,  0.11881606],
                                        [-0.35311466,  0.9329455 ,  0.07016219,  0.01757124],
                                        [ 0.89062457,  0.35816775, -0.28018519, -0.39194853],
                                        [ 0.        ,  0.        ,  0.        ,  1.        ]])

        #np.eye(4)
        
        
    def importpickle(self):
        with open(self.picklename, 'rb') as f:
            return pickle.load(f)
        
        
    def printpickle(self):
        print(self.dict_tmat_pred)
        
    
    def importmy_Tmat(self,inputtmat):
        self.userinput_tmat = inputtmat
        return
    
    
    def __importdepth(self):
        self.depth_raw = o3d.io.read_image(self.path + "depth/{!s}.png".format(str(self.number).zfill(4)))
    
    
    def __importcolor(self):
        self.color_raw = o3d.io.read_image(self.path + "rgb/{!s}.png".format(str(self.number).zfill(4)))
    
    
    def depth2pointcloud(self):
        """"create point cloud using imported depth and color images"""
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_raw, self.depth_raw, depth_scale = self.depth_scale, depth_trunc = self.depth_trunc, convert_rgb_to_intensity=False) #original will be 5000
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic = self.intrinsicmatrix )
        
    
    def createtarget(self):
        self.__importdepth()
        self.__importcolor()
        return self.depth2pointcloud().transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
   
    
    def draw_geometries(self):
        o3d.visualization.draw_geometries([self.source_temp, self.target ,self.axis],
                                      top = 100,
                                      left = 0,
                                      zoom=0.45,
                                      front=[0,0,-1],
                                      lookat=[0,0,0],
                                      up=[0.0,-1.0,0.0])
        
        
    def __copysource(self):
        self.source_temp = copy.deepcopy(self.source)
    
    def paint(self,input_3d):
        input_3d.paint_uniform_color(self.yellow)
        
    
    def draw_pickle_registration(self):
        """"Colored image is the Target, or what the camera is seeing right now. Black and white is the source, which is the 
        cad that we are trying to match the image taken from the camera"""
        
        self.__copysource()
        self.source_temp.transform(self.dict_tmat_pred)
        self.paint(self.source_temp)
        self.draw_geometries()

    
    def draw_userinput_registration(self):
        """"Use this to import your own tmat for visualization"""
        self.__copysource()
        self.source_temp.transform(self.userinput_tmat)
        self.paint(self.source_temp)
        self.draw_geometries()
    
    
        
    


def main():
    cone = "F01"
    A = visualize('temp/','final_tmat.p','definedpcds/'+ cone +'_00adjusted.ply')
    # A.draw_pickle_registration()
    A.draw_userinput_registration()
    print("Running")
    A.printpickle()
    return


if __name__=='__main__':
    main()
    
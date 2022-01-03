# -*- coding: utf-8 -*-
"""
Created on Wed May 26 17:59:51 2021

@author: dylantan1993@gmail.com

"""
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import image
import copy
import pandas as pd
import json






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





#%%

def draw_registration_result(source, target, transformation,sourcecolor = [1, 0.706, 0]):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    
    # source_temp.paint_uniform_color(sourcecolor)
 
    target_temp.paint_uniform_color([0, 0.651, 0.929])
  
        
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      top = 100,
                                      left = 0,
                                      zoom=0.45,
                                      front=[0,0,1],
                                      lookat=[0,0,0],
                                      up=[0.0,1.0,0.0])
    return

# %%

def demo_manual_registration(source,target):
    
#random matrix just for viewing
    trans_init = np.asarray([[0.7071067363577805, 0.0, 0.7071068260153116, 0.0],
                          [0.0, 1.0, 0.0, 0.0],
                          [-0.7071068260153116, 0.0, 0.7071067363577805, 0.2], [0.0, 0.0, 0.0, 1.0]])
    
    draw_registration_result(source, target, trans_init)

# pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

# estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                        o3d.utility.Vector2iVector(corr))


# point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10))
    draw_registration_result(source, target, reg_p2p.transformation)
    print("Manual post ICP transformation is: ", reg_p2p.transformation)
    return reg_p2p

# %%        

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()
    
# %%

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


# %%

def prepare_dataset(voxel_size,source,target):
    
    draw_registration_result(source, target, np.identity(4))
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    
    return source, target, source_down, target_down, source_fpfh, target_fpfh

# %%

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

# %%

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

# %%

def icprefine_registration(source, target, result_fast, voxel_size=1, max_itr = 200 ):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result_icp = o3d.pipelines.registration.registration_icp(
            source, target, distance_threshold, result_fast.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_itr))
    return result_icp

    
##################################################################################################################
# %%
def savePCD(outputname,number=0,path="parallelroute2/"):
    number = number #0
    target = depth2pointcloud(number,path) #S05_3/ parallelroute/S05/WFOVS05/
    o3d.io.write_point_cloud(outputname+str(number)+'.ply', target, write_ascii=True, compressed=False, print_progress=False)
    print("main template saved!")
    return

def autofirst(cadimport = 'standard5.ply', 
              targetpath = 'parallelroute/S05/WFOVS05/', 
              number = 5, 
              voxel_size = 0.005, 
              distance_threshold = 0.03,
              icp_iterations = 10):
    
    # cadimport = cadimport
    source = o3d.io.read_point_cloud(cadimport)
    number = number
    target = depth2pointcloud(number, targetpath) #S05_3/
    voxel_size = voxel_size
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size,source,target)
    
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    draw_registration_result(source_down, target_down, result_fast.transformation)
    distance_threshold = distance_threshold #voxel_size * 0.4
    result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down, distance_threshold, result_fast.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=icp_iterations))
    
    draw_registration_result(source, target, result_icp.transformation)
    print("\nResults after icp:\n ",result_icp)
    print(result_icp.transformation)
    return result_icp


def Manual(source,target): #if autofirst doenst work well
    # pcdimport = "standard5.ply"
    # source = o3d.io.read_point_cloud(pcdimport)
    # number = 5
    # target = depth2pointcloud(number,'parallelroute/S05/WFOVS05/') #S05_3/
    voxel_size = 0.005
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size,source,target)
    result = demo_manual_registration(source_down,target_down) 
    # draw_registration_result(source, target, result.transformation)
    print('Finished manual registration: ',result.transformation)

    return result



def Main_2(sourcenumber,targetnumber):
    """
    Manual registration 

    """


    pcdimport = "definedpcds/S01_00adjusted.ply"

    cad = o3d.io.read_point_cloud(pcdimport)    #combine3 and combine3cone testcombine0246cone combine0246357cone
                                                #S05_GL3cut2_open_axisadj3scaled
                                                
    path = 'temp/'
    numbers = targetnumber #edge cases are 0,11,12,21,22,32 can try all these
    target = depth2pointcloud(numbers,path).transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) #S05_3/ parallelroute/S05/WFOVS05/

    number = targetnumber 
    source = depth2pointcloud(number,path) #S05_3/

    voxel_size = 0.01  # means 1cm for the dataset

    cad, target, cad_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size,cad,target)
    # pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # result_fast = execute_fast_global_registration(cad_down, target_down,
    #                                             source_fpfh, target_fpfh,
    #                                             voxel_size)
    result_fast = Manual(cad,target)
    

    distance_threshold = voxel_size # voxel_size* 0.4   
 
    draw_registration_result(cad_down, target_down, result_fast.transformation )
    evaluation = o3d.pipelines.registration.evaluate_registration(cad, target, distance_threshold, result_fast.transformation)
    print("\nResults for fastglobal registration:\n ",evaluation)
    # print(result_fast.transformation)


    
    result_icp = o3d.pipelines.registration.registration_icp(
            cad_down, target_down, distance_threshold, result_fast.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    
    draw_registration_result(cad_down, target_down, result_icp.transformation)
    evaluation = o3d.pipelines.registration.evaluate_registration(cad, target, distance_threshold, result_icp.transformation)

    print("\nResults after icp:\n ",evaluation)
    # print()
    # print(result_icp.transformation)

    # finalTmatrix = np.matmul(result_icp.transformation,rmation) #standardtrans05
    draw_registration_result(cad, target, result_icp.transformation,sourcecolor=[0, 0.651, 0.929])
    print(result_icp.transformation)
    

    


if __name__ == '__main__':
    Main_2(0,0)



           
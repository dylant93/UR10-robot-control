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



"""
R =  np.array([[0.7071067363577805, 0.0, 0.7071068260153116],[0.0, 1.0, 0.0],[-0.7071068260153116, 0.0, 0.7071067363577805]])
pcd.rotate(R, center=(0, 0, 0))
pcd.translate((0,0,450)) 
# o3d.visualization.draw_geometries([pcd], zoom=1.0, front=[0,0,-1], lookat=[0,0,0], up=[0.0,-1.0,0.0]) #zoom is just for looking only
"""



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


 

""" Compare depth image and our own version of imread

png = image.imread("S05_3/depth/{!s}.png".format(str(number).zfill(4)))
output = (png*65535).astype(np.uint16) #65535
output = np.clip(output,0,5000)

plt.subplot(1, 2, 1)
plt.title('depthimage image')
# plt.imshow(rgbd_image.color)
plt.imshow(output,cmap="nipy_spectral" )
# plt.clim(0,5000)
plt.colorbar()

plt.subplot(1, 2, 2)
plt.title('Depth image')

plt.imshow(rgbd_image.depth,cmap="nipy_spectral")
# plt.clim(0,5000)
plt.colorbar()
plt.show()
"""


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


    

def Main_1(sourcenumber,targetnumber,sourcetmat,path,cadpath,draw = True):


    # pcdimport = "combine3.ply"
    # pcdimport = "S05_GL3cut2_open_axisadj3scaled.ply"
    pcdimport = cadpath

    cad = o3d.io.read_point_cloud(pcdimport)    #combine3 and combine3cone testcombine0246cone combine0246357cone
                                                #S05_GL3cut2_open_axisadj3scaled
                                                
    path = path
    numbers = targetnumber #edge cases are 0,11,12,21,22,32 can try all these
    target = depth2pointcloud(numbers,path) #S05_3/ parallelroute/S05/WFOVS05/

    number = sourcenumber
    source = depth2pointcloud(number,path) #S05_3/

    voxel_size = 0.01  # means 1cm for the dataset

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size,source,target)
    # pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    result_fast = execute_fast_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    if draw:     draw_registration_result(source_down, target_down, result_fast.transformation)
    print("\nResults after fast global registration:\n ",result_fast)
    # print(result_fast.transformation)
    
    
    
    distance_threshold = voxel_size # voxel_size* 0.4
    

    result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down, distance_threshold, result_fast.transformation, #
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
    
    if draw: draw_registration_result(source, target, result_icp.transformation)
    evaluation = o3d.pipelines.registration.evaluate_registration(source_down, target_down, distance_threshold, result_icp.transformation)

    print("\nResults after icp:\n ",evaluation)
    # print(result_icp.transformation)

    finalTmatrix = np.matmul(result_icp.transformation,sourcetmat) #standardtrans05
    if draw: draw_registration_result(cad, target, finalTmatrix,sourcecolor=[0, 0.651, 0.929])
    # print(finalTmatrix)
    # evaluation = o3d.pipelines.registration.evaluate_registration(cad, target, 0.02, finalTmatrix)
    
    cad, target, cad_down, target_down, cad_fpfh, target_fpfh = prepare_dataset(voxel_size,cad,target)
    
    evaluation = o3d.pipelines.registration.evaluate_registration(cad_down, target, distance_threshold, finalTmatrix)
    print('\n\nFinalmatrix evaluation\n\n',evaluation)
    
    result_ic2p = o3d.pipelines.registration.registration_icp(
            cad_down, target_down, distance_threshold, finalTmatrix,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
    
    evaluation = o3d.pipelines.registration.evaluate_registration(cad_down, target_down, distance_threshold, result_ic2p.transformation)
    
    print("\nResults after icp:\n ",evaluation)
    draw_registration_result(cad, target, result_ic2p.transformation, sourcecolor=[0, 0.0, 0.929])
    
    return result_ic2p

def view(viewtransformation, gtpath):
    viewtransformation = viewtransformation
    f = open(gtpath,)
    data = json.load(f)
    print("Fitness: ",data['{!s}'.format(viewtransformation).zfill(4)]['Fitness'])
    print("RMSE: ",data['{!s}'.format(viewtransformation).zfill(4)]['Inlier_rmse'])
    inputtmat = np.array(data['{!s}'.format(viewtransformation).zfill(4)]['Tmat'])
    inputtmat = np.reshape(inputtmat, (4,4))
    cad = o3d.io.read_point_cloud(cadpath)
    target = depth2pointcloud(viewtransformation,path)
    draw_registration_result(cad, target, inputtmat, sourcecolor=[0, 0.0, 0.929])

def Main_2(sourcenumber,targetnumber):


    # pcdimport = "combine3.ply"
    # pcdimport = "S05_GL3cut2_open_axisadj3scaled.ply"
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
    
    # temp = np.array([[1,0,0,0.],
    #             [0,1,0,0.0],
    #             [0,0,1,0],
    #             [0,0,0,1]])
 
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
    
    
    # # evaluation = o3d.pipelines.registration.evaluate_registration(cad, target, 0.02, finalTmatrix)
    # evaluation = o3d.pipelines.registration.evaluate_registration(cad, target, distance_threshold, finalTmatrix)
    # print('Finalmatrix evaluation\n',evaluation)
    # return finalTmatrix, evaluation
    
# Main_2(5,0)
#this is for S05_3 0000.png
trans00 = np.asarray([[0.79284568, -0.16378287, 0.58700161, -0.03031006], 
                      [0.20855506, 0.97797086, -0.00881958, -0.04941181],
                      [-0.57262598, 0.12941472, 0.80953772,0.44790431], 
                      [0.0, 0.0, 0.0, 1.0]])

#this is for newparallel route 0005.png
trans05 = np.asarray([[0.18656294, -0.08154958, 0.97905257, -0.04873522], 
                      [0.04166556, 0.99630912, 0.07504739, -0.00191811],
                      [-0.98155909, 0.02679172, 0.18927217, 0.47325619], 
                      [0.0, 0.0, 0.0, 1.0]])

standardtrans05 = np.asarray([[0.05009544, 0.0166471, 0.99860569, -0.04047158], 
                              [-0.02402853, 0.99959175, -0.01545814, -0.00101348],
                              [-0.99845535, -0.02322064, 0.05047499, 0.46689229], 
                              [0.0, 0.0, 0.0, 1.0]])

#try method main2, which uses the nearest neighbour as a fast proposal, then use icp on the original one
Main_2(0,0)

# temp = np.array([[1,0,0,0.5],
#                 [0,1,0,0],
#                 [0,0,1,0],
#                 [0,0,0,1]])

gtpath = 'GT.json'   #GTrot.json GT.json
path = 'parallelroute/S05/WFOVS05/' #parallelroute/S05/WFOVS05/  #S05_3/
cadpath = "S05_GL3cut2_open_axisadj3scaled.ply"

#%%

# for i in range(22,33):
#     try:
#         f = open(gtpath,)
#         data = json.load(f)
#         # f.close()
        
#         if i<=5:
#             targetpcd = 5-i
#             sourcepcd = 5-i if i==0 else 5-i+1
#         elif i>5 and i<11:
#             targetpcd = i
#             sourcepcd = i-1    
#         elif i>=11 and i<22:
#             targetpcd = i
#             sourcepcd = i-11 if i==11 else i-1  
#         elif i>=22 and i<33:
#             targetpcd = i
#             sourcepcd = i-11 if i==22 else i-1 
            
            
  #%%      
        
        
        
#this is for rotation dataset
        # targetpcd = i
        # # sourcepcd = 7
        # divisor = 7
        # if i < 42:
        #     sourcepcd = 7*((targetpcd/7)-1) if targetpcd%7 == 0 else targetpcd-1
        # elif i > 42 and i < 78:
        #     sourcepcd = 7*(((targetpcd-42)/6)) if (targetpcd-42)%6 == 0 else targetpcd-1
        # elif i >= 78 and i < 120: 
        #     sourcepcd = 7*(((targetpcd-78)/7))+71 if (targetpcd-78)%7 == 0 else targetpcd-1
        #     if i ==78: sourcepcd = 0
        # elif i >= 120: 
        #     sourcepcd = 7*(((targetpcd-120)/6))+78 if (targetpcd-120)%6 == 0 else targetpcd-1
        #     if i ==120: sourcepcd = 78
        # sourcepcd = int((abs(sourcepcd) + sourcepcd) /2)
        
#%%        
        
    #     print(targetpcd,sourcepcd)
    
        
    #     inputtmat = np.array(data['{!s}'.format(sourcepcd).zfill(4)]['Tmat'])
    #     inputtmat = np.reshape(inputtmat, (4,4))
    #     result = Main_1(sourcepcd,targetpcd,inputtmat,path,cadpath,draw = False)
        
        
        
        
    #     finalTmatrix = list(result.transformation.flatten())
    #     data["{!s}".format(str(targetpcd).zfill(4))] = {'Tmat':finalTmatrix, 
    #                                                       'Fitness': result.fitness,
    #                                                       'Inlier_rmse':result.inlier_rmse }
    #     f = open(gtpath,"w")
    #     json.dump(data, open(gtpath,"w"))
    #     f.close()
        
    #     print(targetpcd,sourcepcd)
        
    #     # print("this works",data)
    # except:
    #     print("GT.json does not exist, generating first data point")  
    #     firstpcd = 5
    #     firstresult = autofirst()
    #     firstmatrix = list(firstresult.transformation.flatten())
    #     data = {"{!s}".format(str(firstpcd).zfill(4)) : {'Tmat':firstmatrix, 
    #                                                       'Fitness': firstresult.fitness , 
    #                                                       'Inlier_rmse':firstresult.inlier_rmse }}
    #     f = open("GT.json","w")
    #     json.dump(data, f)
    #     f.close()

# i = 150
# targetpcd = i
# # sourcepcd = 7
# divisor = 7



# if i < 42:
#     sourcepcd = 7*((targetpcd/7)-1) if targetpcd%7 == 0 else targetpcd-1
# elif i >= 42 and i < 78:
#     sourcepcd = 7*(((targetpcd-42)/6)) if (targetpcd-42)%6 == 0 else targetpcd-1
# elif i >= 78 and i < 120: 
#     sourcepcd = 7*(((targetpcd-78)/7))+71 if (targetpcd-78)%7 == 0 else targetpcd-1
#     if i ==78: sourcepcd = 0
# elif i >= 120: 
#     sourcepcd = 7*(((targetpcd-120)/6))+78 if (targetpcd-120)%6 == 0 else targetpcd-1
#     if i ==120: sourcepcd = 78
    
# # sourcepcd = int((abs(sourcepcd) + sourcepcd) /2)
# print(targetpcd,sourcepcd)

# data = {'as':1}
# f = open("test.json","w")
# json.dump(data, f)
# f.close()

# f = open('test.json',)
# data = json.load(f)
# data['assd'] = 2

# json.dump(data, open("test.json", "w"))
# # # inputtmat = np.array(data['0005']['Tmat'])
# # # inputtmat = np.reshape(inputtmat, (4,4))
# # # # data 
    
# f.close()
    
# sourcepcd = 5
# targetpcd = 5
# first,result = Main_1(sourcepcd,targetpcd,standardtrans05)
# Manual()

#######################################################################
# savePCD('definedpcds/test',0,"parallelroute2/C12/normal/")
#########################################################################


# savePCD('definedpcds/S05_0',0,"parallelroute2/S05/")
# dict_file={}
# dict_file[5] = {
#         'Tmat': 1, 
#         'fitness': 2,
#         'inlier_rmse':3
#         }






# loaded = pd.read_csv("newGT.csv", index_col=0)
# Tmat = loaded.iloc[number]['Tmat']
# Tmat = Tmat.strip('][').split(',')
# Tmat = [float(e) for e in Tmat]
# Tmat = np.reshape(np.array(Tmat),(4,4))
# draw_registration_result(source, target, Tmat)   #source is yellow

# draw_registration_result(source, target, trans05)

# result = demo_manual_registration(source,target) 
# print(result)

# loaded = pd.DataFrame(columns=['png','Tmat'])
# loaded = pd.read_csv("newGT.csv", index_col=0)
# loaded.at[number, 'png'] = str(number).zfill(4)+'.png'
# loaded.at[number, 'Tmat'] = list(result.flatten())
# loaded.to_csv("newGT.csv")

# o3d.io.write_point_cloud('test'+str(number)+'.ply', pcd2, write_ascii=True, compressed=False, print_progress=False)


# R =  np.array([[0.7071067363577805, 0.0, 0.7071068260153116],[0.0, 1.0, 0.0],[-0.7071068260153116, 0.0, 0.7071067363577805]])
# T = np.array([[0.0],[0.0],[0.45]])
# trans_init = np.hstack((R,T))
# trans_init = np.vstack((trans_init,np.array([0.0, 0.0, 0.0, 1.0])))



# print("\nInitial alignment")
# evaluation = o3d.pipelines.registration.evaluate_registration(
#     pcd, pcd2, threshold, trans_init2)
# print(evaluation)

# print("Apply point-to-point ICP")
# reg_p2p = o3d.pipelines.registration.registration_icp(
#     pcd, pcd2, threshold, trans_init2,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint(), #TransformationEstimationPointToPlane 
#                                                                         #TransformationEstimationPointToPoint
#     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
# print(reg_p2p)
# print("Transformation is:")
# print(reg_p2p.transformation)

# draw_registration_result(pcd, pcd2, reg_p2p.transformation)
  
# evaluation = o3d.pipelines.registration.evaluate_registration(
# pcd, pcd2, threshold, trans_init)
# print(evaluation)                     
                                
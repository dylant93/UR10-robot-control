# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'

# %%
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
import copy
import numpy as np
import pandas as pd
import pickle


# %%
pcd_cad = o3d.io.read_point_cloud('../3d_models/S05_GL3cut2_open_axisadj2.ply')
pcd_cad = pcd_cad.scale(0.001, [0,0,0])
pcd_cad = pcd_cad.voxel_down_sample(0.001)
radius_normal = 0.002
pcd_cad.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
pcd_cad.orient_normals_consistent_tangent_plane(100)

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd_cad, mesh_frame])


# %%
pcd_cad_new = o3d.io.read_point_cloud('../3d_models/S05_GL3cut2_open_axisadj3.ply')
pcd_cad_new = pcd_cad_new.scale(0.001, [0,0,0])
pcd_cad_new = pcd_cad_new.voxel_down_sample(0.001)
radius_normal = 0.002
pcd_cad_new.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
pcd_cad_new.orient_normals_consistent_tangent_plane(100)

o3d.visualization.draw_geometries([pcd_cad_new, mesh_frame])


# %%
def demo_crop_geometry(pcd_source):
    # print("Demo for manual geometry cropping")
    # print("1) Press 'Y' twice to align geometry with negative direction of y-axis")
    # print("2) Press 'K' to lock screen and to switch to selection mode")
    # print("3) Drag for rectangle selection,")
    # print("   or use ctrl + left click for polygon selection")
    # print("4) Press 'C' to get a selected geometry and to save it")
    # print("5) Press 'F' to switch to freeview mode")
    # pcd = o3d.io.read_point_cloud(source_path)
    pcd = pcd_source
    o3d.visualization.draw_geometries_with_editing([pcd])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def pick_points(pcd):
    # print("")
    # print("1) Please pick at least three correspondences using [shift + left click]")
    # print("   Press [shift + right click] to undo point picking")
    # print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    # print("")
    return vis.get_picked_points()


def demo_manual_registration(source_path, target_path):
    # print("Demo for manual ICP")
    # source = o3d.io.read_point_cloud(source_path)
    # target = o3d.io.read_point_cloud(target_path)
    source = source_path
    target = target_path
    # print("Visualization of two point clouds before manual alignment")
    # draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    # estimate rough transformation using correspondences
    # print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    # print("Perform point-to-point ICP refinement")
    threshold = 0.001  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    # print("")

    return reg_p2p.transformation


# %%
for i in range(0,50,5):

    if i < 10:
        filename = '000' + str(i)
    else:
        filename = '00' + str(i)

    print('Processing and Cropping PCD' , filename)

    npzfile = np.load('../data_dylan/S05_green_normal_backgnd/normal/npy/' + filename + '.npz')
    np_rgbd = npzfile['arr_0']

    color_raw = np_rgbd[:,:,0:3]
    depth_raw = np_rgbd[:,:,3]

    color_npy = np.asarray(color_raw).astype(np.uint8)
    depth_npy =  np.asarray(depth_raw).astype(np.float32) / 1000.0

    color_image = o3d.geometry.Image(color_npy)
    depth_image = o3d.geometry.Image(depth_npy)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image, depth_scale=1.0, convert_rgb_to_intensity=False)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(4096, 3072, 1944.4559326171875, 1943.5645751953125, 2049.344970703125, 1556.814453125)
    pcd_temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

    # o3d.visualization.draw_geometries([pcd_0005, mesh_frame])
    demo_crop_geometry(pcd_temp)


# %%
for i in range(0,50,5):
    pcd_cropped = o3d.io.read_point_cloud('pcd_cropped/' + str(i) + '.ply')
    pcd_cropped = pcd_cropped.voxel_down_sample(0.001)
    radius_normal = 0.002
    pcd_cropped.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    pcd_cropped.orient_normals_consistent_tangent_plane(100)
    o3d.io.write_point_cloud('pcd_cropped/' + str(i) + '.ply', pcd_cropped)


# %%
dict_tmat_manual = {}
for i in range(0,50,5):
    pcd_cropped = o3d.io.read_point_cloud('pcd_cropped/' + str(i) + '.ply')
    # o3d.visualization.draw_geometries([pcd_cropped, pcd_cad, mesh_frame])

    print('Manual registration for PCD', i)
    dict_tmat_manual[i] = demo_manual_registration(pcd_cropped, pcd_cad)


# %%
print(dict_tmat_manual[0])
with open('dict_tmat_manual_0-45.p', 'wb') as handle:
    pickle.dump(dict_tmat_manual, handle, protocol=pickle.HIGHEST_PROTOCOL)

with open('dict_tmat_manual_0-45.p', 'rb') as handle:
    dict_tmat_manual = pickle.load(handle)

# %% [markdown]
# ## Read the csv and get transformation matrices

# %%
pd_data = pd.read_csv('outputWR.csv', delimiter=',')
pd_data.head
# print(pd_data.loc[[0],['RelPose']])
# pd_data.columns = ['Cycle','Cone','RGBname','DEPTHname','RGBD','Pose','RelPose','Theta','Rotation']
# print(pd_data.columns)

dict_pose = {}
dict_rotation = {}
for i in range(0,50,5):
    pose = pd_data.iloc[i]['RelPose']
    pose = pose.replace('[', '')
    pose = pose.replace(']', '')
    pose = list(map(float, pose.split()))
    # print(pose)
    # dict_pose[i] = np.array(pose)
    dict_pose[i] = np.array([0, 0, 0.45])

    rotation = pd_data.iloc[i]['Rotation']
    rotation = rotation.replace('[', '')
    rotation = rotation.replace(']', '')
    rotation = list(map(float, rotation.split(',')))
    # print(rotation)
    dict_rotation[i] = np.reshape(np.array(rotation), (3,3))

dict_pose_inv = {}
dict_rotation_inv = {}
for i in range(0,50,5):
    dict_rotation_inv[i] = np.transpose(dict_rotation[i])
    dict_pose_inv[i] = -1 * np.matmul(dict_rotation_inv[i], np.transpose(dict_pose[i]))
    # dict_pose_inv[i] = np.transpose(dict_pose_inv[i])

print(dict_pose[45])
print(dict_pose_inv[45])
print(dict_rotation[45])
print(dict_rotation_inv[45])


# %%
pcd_list = []
for i in range(0, 45, 5):
    pcd_cropped = o3d.io.read_point_cloud('pcd_cropped/' + str(i) + '.ply')
    transformation_matrix = np.identity(4)
    transformation_matrix[0:3,0:3] = dict_rotation_inv[i]
    transformation_matrix[0:3,3] = dict_pose_inv[i]
    pcd_cropped.transform(transformation_matrix)
    pcd_list.append(pcd_cropped)
    o3d.visualization.draw_geometries([pcd_cropped.paint_uniform_color([1, 0.706, 0]), pcd_cad, mesh_frame])

o3d.visualization.draw_geometries(pcd_list + [pcd_cad, mesh_frame])


# %% [markdown]
# ## Visualization for presentations

# %%
for i in range(0,15,5):
    pcd_cropped = o3d.io.read_point_cloud('pcd_cropped/' + str(i) + '.ply')
    o3d.io.write_point_cloud('pcd_cropped/' + str(i) + '.pcd', pcd_cropped)


# %%
for i in range(0,15,5):
    pcd_smooth = o3d.io.read_point_cloud('pcd_cropped_smooth/' + str(i) + '.pcd')
    o3d.visualization.draw_geometries([pcd_smooth])


# %%
# o3d.visualization.draw_geometries([pcd_cad])
o3d.io.write_point_cloud('../3d_models/S05_GL3cut2_open_axisadj2.pcd', pcd_cad)


# %%
pcd_erode = o3d.io.read_point_cloud('pcd_cropped/0.ply')
o3d.visualization.draw_geometries([pcd_cad, pcd_erode, mesh_frame])

# %% [markdown]
# ## Calculating error between manual registraion and experimental transformaton matrices

# %%
dict_tmat_experimental = {}
for i in range(0, 45, 5):
    transformation_matrix = np.identity(4)
    transformation_matrix[0:3,0:3] = dict_rotation_inv[i]
    transformation_matrix[0:3,3] = dict_pose_inv[i]
    dict_tmat_experimental[i] = transformation_matrix

with open('dict_tmat_manual_0-45.p', 'rb') as handle:
    dict_tmat_manual = pickle.load(handle)

error_mat = np.zeros((4,4))
count = 0
for i in range(0, 45, 5):
    count += 1
    error_mat += (dict_tmat_manual[i] - dict_tmat_experimental[i])
    print(dict_tmat_manual[i] - dict_tmat_experimental[i])
error_mat /= count
print(error_mat)


# %%
def refine_registration(source, target):
    distance_threshold = 0.01
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def draw_registration_result(source, target, window_name):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name=window_name)


# %%
pcd_list = []
for i in range(0, 45, 5):
    pcd_cropped = o3d.io.read_point_cloud('pcd_cropped/' + str(i) + '.ply')
    transformation_matrix = np.identity(4)
    transformation_matrix[0:3,0:3] = dict_rotation_inv[i]
    transformation_matrix[0:3,3] = dict_pose_inv[i]
    pcd_cropped.transform(transformation_matrix)
    o3d.visualization.draw_geometries([pcd_cropped.paint_uniform_color([1, 0.706, 0]), pcd_cad])
    # pcd_cropped.rotate(error_mat[0:3,0:3])
    pcd_cropped.translate(np.transpose(error_mat[0:3,3]))
    o3d.visualization.draw_geometries([pcd_cropped.paint_uniform_color([1, 0.706, 0]), pcd_cad])

    result_icp = refine_registration(pcd_cropped, pcd_cad)
    pcd_cropped.transform(result_icp.transformation)
    pcd_list.append(pcd_cropped)
    o3d.visualization.draw_geometries([pcd_cropped.paint_uniform_color([1, 0.706, 0]), pcd_cad])

o3d.visualization.draw_geometries(pcd_list + [pcd_cad, mesh_frame])


# %%




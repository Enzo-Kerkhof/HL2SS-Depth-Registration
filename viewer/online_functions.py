''' File with all functions to perform online registration with the HoloLens 2'''

import numpy as np
import pandas as pd
import cv2
import open3d as o3d
from pathlib import Path
import sys
import hl2ss
import hl2ss_io
import hl2ss_utilities
import hl2ss_3dcv
import hl2ss_mp
from matplotlib import pyplot as plt
from probreg import filterreg
import copy
from scipy.spatial.transform import Rotation as R
import rus


def get_model_path(model):
    '''
    Get path of obj file based on model number,
    you can add a flag here if the model is not in CT (mm) scale
    '''
    project_folder = "E:/Enzo/TM_Thesis/00_AR_DATA/hl2ss/depth_reg_aruco_app/Assets/"
    CT_scale = True
    if model == 0:
        source_file_path = project_folder + "LEGO.obj"
    if model == 1:
        source_file_path = project_folder + "Skull.obj"
    if model == 2:
        source_file_path = project_folder + "Chest.obj"
    if model == 3:
        source_file_path = project_folder + "Spine_outside.obj"
    if model == 4:
        source_file_path = project_folder + "Voet.obj"
        CT_scale=False
    if model == 5:
        source_file_path = project_folder + "Abdomen.obj"
    
    return source_file_path, CT_scale


def get_source_point_cloud(source_file_path: str, CT_scale=True):
    '''
    Read .stl file and convert to Open3D mesh.
    Input file path as string.
    '''
    source_mesh = o3d.io.read_triangle_mesh(source_file_path)                             # open3d mesh
    source_point_cloud = o3d.geometry.PointCloud()
    source_point_cloud.points = source_mesh.vertices
    # source_point_cloud.paint_uniform_color([1, 0.706, 0])                               # visualisation purposes o3d
    # source_point_cloud.translate((0, 0, 0), relative=False)                             # translate model to origin
    if CT_scale:
        # source_point_cloud.scale(0.001, center=source_point_cloud.get_center())         # mm to meter
        source_point_cloud.scale(0.001, center=(0, 0, 0))                                 # mm to meter
    else:
        pass
    return source_point_cloud


def send_transform(host, model, position, rotation):
    'Sends the unity app the found transformation'

    # Ports
    port = hl2ss.IPCPort.UNITY_MESSAGE_QUEUE

    # Scale
    # 0.001 since parent in Unity is scaled from meter to mm
    scale = [0.001, 0.001, 0.001]

    # Start client
    ipc = hl2ss.ipc_umq(host, port)
    ipc.open()
    key = 0

    print('Sending registration to {iid}'.format(iid=host))

    # Get Unity Instance ID
    display_list = rus.command_buffer()
    display_list.begin_display_list()           # Begin command sequence
    display_list.create_primitive(model)        # --> changed function behaviour now: server will return id of # model chosen in editor
    display_list.end_display_list()             # End command sequence
    ipc.push(display_list)                      # Send commands to server
    results = ipc.pull(display_list)             # Get results from server
    key = results[1]                            # Get the model id, got by the 2nd command in the list, create primitive

    print('Found model with id {iid}'.format(iid=key))

    # Send commands concerning the found Instance ID
    display_list = rus.command_buffer()
    display_list.begin_display_list()                                       # Begin command sequence
    display_list.set_world_transform(key, position, rotation, scale)        # Set the world transform of the model
    display_list.remove(key)                                                # remove Instance ID and Game Object from private dict at the C# server side: else a memory exception will occur.
    display_list.end_display_list()                                         # End command sequence
    ipc.push(display_list)                                                  # Send commands to server
    results = ipc.pull(display_list)                                        # Get results from server

    print(f'Interacted with model {key}')

    ipc.close()


def get_transform(host, model, verbal=False):
    'Gets the transformation of the hologram in unity app space, and coverts left to right handed.'

    # Ports
    port = hl2ss.IPCPort.UNITY_MESSAGE_QUEUE

    # Start client
    ipc = hl2ss.ipc_umq(host, port)
    ipc.open()
    key = 0

    print(f'Getting location of {model} from {host}')

    # Get Unity Instance ID
    display_list = rus.command_buffer()
    display_list.begin_display_list()           # Begin command sequence
    display_list.create_primitive(model)        # --> changed function behaviour now: server will return id of # model chosen in editor
    display_list.end_display_list()             # End command sequence
    ipc.push(display_list)                      # Send commands to server
    results = ipc.pull(display_list)             # Get results from server
    key = results[1]                            # Get the model id, got by the 2nd command in the list, create primitive

    if verbal:
        print('Found model with id {iid}'.format(iid=key))

    # Send commands concerning the found Instance ID
    display_list = rus.command_buffer()
    display_list.set_local_transform(key)        # Set the world transform of the model
    ipc.push(display_list)                                                  # Send commands to server
    results = ipc.pull(display_list, is_transform=True)                     # Get results from server

    # Remove key from private dict
    display_list = rus.command_buffer()
    display_list.remove(key)        # Set the world transform of the model
    ipc.push(display_list)                                                  # Send commands to server
    
    if verbal:
        print(f'Found space and rotation \n{np.reshape(results, (4,4))}')

    ipc.close()

    unity_location = np.reshape(results, (4,4))

    flip = np.array([[1,  0,   0, 0],   # Flip Z axis for left to right handed
                     [0,  1,   0, 0],
                     [0,  0,  -1, 0],
                     [0,  0,   0, 1]])

    o3d_location = flip @ unity_location
    o3d_location[:,0] = -o3d_location[:,0]

    return o3d_location


def o3d_matrix_to_unity(result, verbal=False):
    '''
    Converts found point cloud registration in Open3D to Unity coordinate system.
    '''
    
    try:
        transformation = result.transformation

    except:
        transformation = result

    # Go from right handed Open3D/OpenCV to left handed Unity for transformation
    position = [transformation[0,3],     # x
                transformation[1,3],     # y
                -transformation[2,3]]    #-z

    # Convert rotation matrix
    rotation_matrix = transformation[:3, :3]
    flip = np.array([[-1.0000000,  0.0000000,  0.0000000],              # Create 180 y flip because of reasons I cannot figure out....
                    [ 0.0000000,  1.0000000,  0.0000000],
                    [-0.0000000,  0.0000000, -1.0000000]])
    
    rotation_matrix = flip @ rotation_matrix                            # Matrix multiplication of 180 flip with found registration
    rotation_matrix = rotation_matrix.copy()                            # Fix read only bug with Scipy
    rotation_quat = R.from_matrix(rotation_matrix).as_quat()            # Get Quaternation rotation using Scipy, this is x, y, z, w
    rotation = [rotation_quat[0], -rotation_quat[1],
                -rotation_quat[2], rotation_quat[3]]                    # Go from right handed Open3D/OpenCV to left handed Unity for rotation: this is x, -y, -z, w
    
    if verbal is True:
        #Print results of registration/conversion
        print(result)
        print(f'Found registration matrix in Open3D:\n{transformation}')
        print(f'Rotation matrix with 180 y:\n{rotation_matrix}')
        print(f'Rotation quaternation:\n{rotation_quat}')
        print(f'Unity translation:\n{position}')
        print(f'Unity rotation as quateration:\n{rotation}')

    return position, rotation


def point_cloud_from_data_in_world(data, min_depth=0, max_depth=1.0, calibration_path='./calibration', lt=False):
    '''
    Get point cloud in world space from depth hl2ss data packet.
    '''
    if lt is True:
        # Get camera calibration
        calibration = hl2ss_3dcv.get_calibration_rm(None, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)

        # Compute depth-to-rgb registration constants
        xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(calibration.uv2xy, calibration.scale)

    else:
        # Get camera calibration
        calibration = hl2ss_3dcv.get_calibration_rm(None, hl2ss.StreamPort.RM_DEPTH_AHAT, calibration_path)

        # Compute depth-to-rgb registration constants
        xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(calibration.uv2xy, calibration.scale / hl2ss.Parameters_RM_DEPTH_AHAT.FACTOR)

    # Convert depth payload to 3D points
    depth = hl2ss_3dcv.rm_depth_normalize(data.payload.depth, scale)
    xyz = hl2ss_3dcv.rm_depth_to_points(depth, xy1)
    xyz = hl2ss_3dcv.block_to_list(xyz)
    xyz = xyz[(xyz[:, 2] > min_depth) & (xyz[:, 2] < max_depth), :] 

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # 4x4 matrix that converts 3D points in depth camera space to world space
    camera2world = hl2ss_3dcv.camera_to_rignode(calibration.extrinsics) @ hl2ss_3dcv.reference_to_world(data.pose)
    pcd.transform(camera2world.transpose())
    
    return pcd


def create_fit_bounding_box(source_pcd):
    ''' Create bouding box with min and max bounds '''
    source_bounding_box = source_pcd.get_axis_aligned_bounding_box()
    source_bounding_box.color = (1, 0, 0)
    min_bound = np.min(source_bounding_box.get_min_bound())
    max_bound = np.max(source_bounding_box.get_max_bound())
    source_bounding_box.min_bound = [min_bound, min_bound, min_bound]
    source_bounding_box.max_bound = [max_bound, max_bound, max_bound]

    return source_bounding_box


def preprocess_point_cloud(pcd, voxel_size):
    '''
    Preprocess point cloud to get downsampeld and fpfh
    '''
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


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5 # 50?
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold,
            iteration_number=64,
            # decrease_mu=False,
            # division_factor=1.4,
            use_absolute_scale=False))
    return result


def save_vuforia_gt(host, model_vuforia, save_folder=None):
    '''Save the ground truth position from the multimodal vuforia NDI system'''
    # Ports
    port = hl2ss.IPCPort.UNITY_MESSAGE_QUEUE

    # Start client
    ipc = hl2ss.ipc_umq(host, port)
    ipc.open()
    key = 0

    print(f'Getting location of {model_vuforia} from {host}')

    # Get Unity Instance ID
    display_list = rus.command_buffer()
    display_list.begin_display_list()           # Begin command sequence
    display_list.create_primitive(model_vuforia)        # --> changed function behaviour now: server will return id of # model chosen in editor
    display_list.end_display_list()             # End command sequence
    ipc.push(display_list)                      # Send commands to server
    results = ipc.pull(display_list)             # Get results from server
    key = results[1]                            # Get the model id, got by the 2nd command in the list, create primitive

    # Send commands concerning the found Instance ID
    display_list = rus.command_buffer()
    display_list.set_local_transform(key)        # Get the world transform of the model
    ipc.push(display_list)                                                  # Send commands to server
    results = ipc.pull(display_list, is_transform=True)                     # Get results from server

    # Remove key from private dict
    display_list = rus.command_buffer()
    display_list.remove(key)                    
    ipc.push(display_list)                                                  # Send commands to server
    
    print(f'Found space and rotation \n{np.reshape(results, (4,4))}')

    ipc.close()

    unity_location = np.reshape(results, (4,4))

    if save_folder is not None:
        if save_folder.is_dir():
            filename = save_folder / f'registration_QR.npy'
            np.save(str(filename), unity_location)

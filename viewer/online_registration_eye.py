'''
The registration function
'''

import numpy as np
import open3d as o3d
import hl2ss
import hl2ss_utilities
import copy
import time
import hl2ss_3dcv
import cv2
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

from online_functions import get_source_point_cloud
from online_functions import create_fit_bounding_box
from online_functions import preprocess_point_cloud
from online_functions import execute_global_registration
from online_functions import execute_fast_global_registration
from online_functions import o3d_matrix_to_unity
from online_functions import send_transform
from online_functions import get_model_path



def register(host, model, target_pcd, data_si, save_folder=None):
    #-------------------------------------------------------------------------------------------------------------------------
    # Start REGISTRATION PIPELINE

    # Noise filter params
    down_sample_vox = 0.005  # cm? voxel size
    alpha_shape = 0.10      # alpah factor for mesh calculation
    eye_quartile = 0.5      # The x quartile for eye median/filter

    # Grab the source point cloud
    source_file_path, CT_scale = get_model_path(model)

    start_time = time.time()

    eye_points = []
    head_frames = []
    eye_pointers = []
    eye_lines = []

    mesh = o3d.geometry.TriangleMesh()

    # Define o3d objects for eye ray and head frame
    # For visualisation
    head_frame_size = 0.1
    eye_pointer_radius = 0.02
    eye_pointer_color = np.array([0, 1, 0])

    eye_pointer_radius_median = 0.04
    eye_pointer_color_median = np.array([1, 0, 1])

    eye_pointer_median = o3d.geometry.TriangleMesh.create_octahedron(radius=eye_pointer_radius_median)
    eye_pointer_median.paint_uniform_color(eye_pointer_color_median)

    #---------------------------------------------------------------------------------------------------
    # Filter target pcd
    print(f'Target pcd points: {len(target_pcd.points)}')

    target_pcd = target_pcd.voxel_down_sample(voxel_size=down_sample_vox)
    print(f'Target pcd points downsampled: {len(target_pcd.points)}')

    # create one mesh of all acq pcds
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(target_pcd, alpha_shape)
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    
    # Loop to get x amount of point clouds
    for data in data_si:
        # Calculate eye ray + head pose and visualise on cropped pointcloud
        si = hl2ss.unpack_si(data.payload)

        if (si.is_valid_head_pose()) and (si.is_valid_eye_ray()):
            
            # Creat new headframe
            head_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=head_frame_size, origin=np.array([0.0, 0.0, 0.0]))
            head_previous_rotation = np.eye(3, 3)
            
            # Create new eye pointer
            eye_pointer = o3d.geometry.TriangleMesh.create_octahedron(radius=eye_pointer_radius)
            eye_pointer.paint_uniform_color(eye_pointer_color)

            # Create new eye line
            eye_line = o3d.geometry.LineSet()
            eye_line.colors = o3d.utility.Vector3dVector(np.array([0, 1, 0]).reshape((1, 3)))
            eye_line.points = o3d.utility.Vector3dVector(np.array([[0, 0, 0], [0, 1, 0]]).reshape((2, 3)))
            eye_line.lines = o3d.utility.Vector2iVector(np.array([0, 1]).reshape((1,2)))
            
            # Get head pose
            head_pose = si.get_head_pose()
            head_frame.translate(head_pose.position, False)
            head_rotation = hl2ss_utilities.si_head_pose_rotation_matrix(head_pose)
            head_frame.rotate(head_rotation @ head_previous_rotation.transpose())
            head_previous_rotation = head_rotation
            
            # Make list of valid head poses
            head_frames.append(head_frame)
            
            # Get eye ray
            eye_ray = si.get_eye_ray()
        
            # Raycast eye ray on mesh of target point cloud
            ray = np.hstack((eye_ray.origin.reshape((1, -1)), eye_ray.direction.reshape((1,-1))))
            
            rs = o3d.t.geometry.RaycastingScene()
            rs.add_triangles(mesh)

            rc = rs.cast_rays(ray)
            d = rc['t_hit'].numpy()

            # If raycast hit, save point
            if (not np.isinf(d)):
                eye_point = ray[0, 0:3] + d*ray[0, 3:6]
                eye_points.append(eye_point)
                
                eye_pointers.append(eye_pointer.translate(eye_point, False))
                eye_line.points = o3d.utility.Vector3dVector(np.array([eye_point, ray[0, 0:3]]).reshape((2, 3)))
                eye_lines.append(eye_line)

    print(f'\nEye raycasting took: {(time.time()-start_time)} seconds\n')

    # Get source point cloud
    source_pcd = get_source_point_cloud(source_file_path, CT_scale=CT_scale)

    # Get bounding box of source, resize X and Z as rectangle
    centered_pcd = copy.deepcopy(source_pcd)
    centered_pcd.translate((0, 0, 0), relative=False)
    source_bounding_box = create_fit_bounding_box(centered_pcd)

    # Calculate median eye point
    if eye_points:
        eye_point_median = np.array(eye_points)
        eye_point_median = np.quantile(eye_point_median, eye_quartile, axis=0)
    else:
        print('No valid eye initialisation')
        pass

    # Translate purple pointer in 3D space
    eye_pointer_median.translate(eye_point_median, False)

    # Scale and translate source bounding box on eye median
    source_bounding_box.scale(1.2, center=source_bounding_box.get_center())
    source_bounding_box.translate(eye_point_median, relative=False)

    # Crop target point cloud
    target_pcd_eye_crop = target_pcd.crop(source_bounding_box)

    #-------------------------------------------------------------------------------------------------------------------------
    # FAST COARSE REGISTRATION

    voxel_size = 0.01

    source_temp = copy.deepcopy(source_pcd)

    source_down, source_fpfh = preprocess_point_cloud(source_temp, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target_pcd_eye_crop, voxel_size)

    coarse_result  = execute_fast_global_registration(source_down, target_down,
                                                      source_fpfh, target_fpfh,
                                                      voxel_size)
    
    coarse_transform = coarse_result.transformation

    source_temp_coarse = copy.deepcopy(source_down)
    source_temp_coarse.transform(coarse_transform)
    source_temp_coarse.paint_uniform_color([1, 0.706, 0])

    #-------------------------------------------------------------------------------------------------------------------------
    # COARSE REGISTRATION

    source_temp_2 = copy.deepcopy(source_temp_coarse)

    coarse_result_2  = execute_global_registration(source_temp_2, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)

    coarse_transform_2 = coarse_result_2.transformation

    source_temp_coarse_2 = copy.deepcopy(source_temp_2)
    source_temp_coarse_2.transform(coarse_transform_2)
    source_temp_coarse_2.paint_uniform_color([1, 0, 0])

    #-------------------------------------------------------------------------------------------------------------------------
    # COARSE FILTERREG REGISTRATION
    # start_time = time.time()

    # # Coarse registration
    # from probreg import filterreg

    # tf_param, _, _ = filterreg.registration_filterreg(source_down,
    #                                                   target_pcd_eye_crop,
    #                                                   sigma2=None,
    #                                                   update_sigma2=True,
    #                                                   tf_init_params={"rot": np.identity(3),"t": eye_point_median})

    # coarse_transform = np.vstack([[tf_param.rot[0,0], tf_param.rot[0,1], tf_param.rot[0,2], tf_param.t[0]],
    #                               [tf_param.rot[1,0], tf_param.rot[1,1], tf_param.rot[1,2], tf_param.t[1]],
    #                               [tf_param.rot[2,0], tf_param.rot[2,1], tf_param.rot[2,2], tf_param.t[2]],
    #                               [0, 0, 0, 1]])

    # print(f'Found transform:\n{coarse_transform}')

    # print(f'Global Registration took: {(time.time()-start_time)} seconds')

    # source_temp_coarse = copy.deepcopy(source_down)
    # source_temp_coarse.transform(coarse_transform)
    # source_temp_coarse.paint_uniform_color([1, 0.706, 0])

    #-------------------------------------------------------------------------------------------------------------------------
    # FINE REGISTRATION

    # Crop target point cloud based on coarse reg result
    # coarse_bbx = source_temp_coarse.get_axis_aligned_bounding_box()
    coarse_bbx = source_temp_coarse_2.get_axis_aligned_bounding_box()
    coarse_bbx.color = (0, 0, 1)
    coarse_bbx.scale(1.1, center=coarse_bbx.get_center())
    target_pcd_eye_crop_fine = target_pcd_eye_crop.crop(coarse_bbx)

    # Add normals to target point cloud
    target_pcd_eye_crop_fine.estimate_normals()

    # ICP parameters
    threshold = 0.01
    sigma = 1.0

    criteria  = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)

    # Set initial transform from coars registratino
    # trans_init = coarse_transform
    trans_init = coarse_transform_2 @ coarse_transform

    # Use robust kernel
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    print("Using robust loss:", loss)

    fine_result = o3d.pipelines.registration.registration_icp(source_down,
                                                              target_pcd_eye_crop_fine,
                                                              threshold,
                                                              trans_init,
                                                              o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
                                                              criteria=criteria)

    registration_result = fine_result.transformation

    source_temp_fine = copy.deepcopy(source_pcd)
    source_temp_fine.transform(registration_result)
    source_temp_fine.paint_uniform_color([1, 0.706, 1])

    #-------------------------------------------------------------------------------------------------------------------------
    # UNITY INTERACTION

    if host is not None:
        position, rotation = o3d_matrix_to_unity(fine_result)

        pos = [[position[0]], [position[1]], [position[2]]]
        tx_registration = np.concatenate([R.from_quat(rotation).as_matrix(), pos], axis=1)
        registration_result_unity = np.concatenate([tx_registration, [[0, 0, 0, 1]]], axis=0)

        # Send fine registration transformation matrix
        send_transform(host, model, position, rotation)

    timer = time.time()-start_time
    print(f'\n Eye RANSAC Registration took: {timer} seconds\n')

    if save_folder is not None:
        if save_folder.is_dir():
            filename = save_folder / f'registration_eye_ransac_o3d.npy'
            np.save(str(filename), registration_result)
            filename = save_folder / f'registration_eye_ransac.npy'
            np.save(str(filename), registration_result_unity)
            filename = save_folder / f'time_eye_ransac_registration.txt'
            np.savetxt(str(filename), np.array([timer]), fmt='%.4f')

    # # Show registration result
    # o3d.visualization.draw_geometries([target_pcd,
    #                                    eye_pointer_median,
    #                                    source_bounding_box,
    #                                    coarse_bbx,
    #                                    source_temp_coarse,
    #                                    source_temp_coarse_2,
    #                                    source_temp_fine]
    #                                    + head_frames + eye_pointers + eye_lines)
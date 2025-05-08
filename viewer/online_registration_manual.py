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
from online_functions import o3d_matrix_to_unity
from online_functions import send_transform
from online_functions import get_transform
from online_functions import get_model_path


def register(host, model, target_pcd, data_si, save_folder=None):
    #-------------------------------------------------------------------------------------------------------------------------
    # Start REGISTRATION PIPELINE

    # Noise filter params
    down_sample_vox = 0.005  # cm? voxel size

    # Grab the source point cloud
    source_file_path, CT_scale = get_model_path(model)

    start_time = time.time()

    #---------------------------------------------------------------------------------------------------
    # Filter target pcd
    print(f'Target pcd points: {len(target_pcd.points)}')

    target_pcd = target_pcd.voxel_down_sample(voxel_size=down_sample_vox)
    print(f'Target pcd points downsampled: {len(target_pcd.points)}')

    # Get unity app location of hologram
    manual_init = get_transform(host, model)

    # Get source point cloud
    source_pcd = get_source_point_cloud(source_file_path, CT_scale=CT_scale)
    source_pcd.transform(manual_init)
    source_pcd.paint_uniform_color([0.5, 0, 0])

    # Get bounding box of source, resize X and Z as rectangle
    source_bounding_box = source_pcd.get_axis_aligned_bounding_box()

    # Scale and translate source bounding box on eye median
    source_bounding_box.scale(1.5, center=source_bounding_box.get_center())

    # Crop target point cloud
    target_pcd_init_crop = target_pcd.crop(source_bounding_box)
    #-------------------------------------------------------------------------------------------------------------------------
    # FINE REGISTRATION

    # Add normals to target point cloud
    target_pcd_init_crop.estimate_normals()

    # ICP parameters
    threshold = 0.01
    sigma = 1.0
    criteria  = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    trans_init = np.identity(4)

    # Use robust kernel
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    print("Using robust loss:", loss)

    fine_result = o3d.pipelines.registration.registration_icp(source_pcd,
                                                              target_pcd_init_crop,
                                                              threshold,
                                                              trans_init,
                                                              o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
                                                              criteria=criteria)

    fine_transform = fine_result.transformation

    print("Inlier Fitness: ", fine_result.fitness)
    print("\nInlier RMSE: ", fine_result.inlier_rmse)

    source_temp_fine = copy.deepcopy(source_pcd)
    source_temp_fine.transform(fine_transform)
    source_temp_fine.paint_uniform_color([1, 0.706, 1])

    #-------------------------------------------------------------------------------------------------------------------------
    # UNITY INTERACTION

    registration_result = fine_transform @ manual_init

    if host is not None:
        position, rotation = o3d_matrix_to_unity(registration_result)

        pos = [[position[0]], [position[1]], [position[2]]]
        tx_registration = np.concatenate([R.from_quat(rotation).as_matrix(), pos], axis=1)
        registration_result_unity = np.concatenate([tx_registration, [[0, 0, 0, 1]]], axis=0)

        # Send fine registration transformation matrix
        send_transform(host, model, position, rotation)

    timer = time.time()-start_time
    print(f'\n Manual fine Registration took: {timer} seconds\n')

    if save_folder is not None:
        if save_folder.is_dir():
            filename = save_folder / f'registration_manual_o3d.npy'
            np.save(str(filename), registration_result)
            filename = save_folder / f'registration_manual.npy'
            np.save(str(filename), registration_result_unity)
            filename = save_folder / f'time_manual_registration.txt'
            np.savetxt(str(filename), np.array([timer]), fmt='%.4f')
            
    # Show registration result
    # o3d.visualization.draw_geometries([target_pcd,
    #                                    source_pcd,
    #                                    source_bounding_box,
    #                                 #    source_temp_coarse,
    #                                 #    source_temp_coarse_2,
    #                                    source_temp_fine])
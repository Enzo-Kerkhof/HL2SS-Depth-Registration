'''To call from other scripts when you need a volume integration'''
import hl2ss_3dcv
import time
from tqdm import tqdm
import hl2ss
import open3d as o3d
import cv2
import numpy as np

def volume_integration(data_depth_list,
                       data_lf_list,
                       calibration_path='./calibration',
                       voxel_length = 1/750,
                       sdf_trunc = 0.002,
                       max_depth = 1.0,
                       min_depth = 0.35,
                       lt=True,
                       save_folder=None):
    
    start_time = time.time()

    calibration_vlc = hl2ss_3dcv.get_calibration_rm(None, hl2ss.StreamPort.RM_VLC_LEFTFRONT, calibration_path)

    if lt:
        calibration = hl2ss_3dcv.get_calibration_rm(None, hl2ss.StreamPort.RM_DEPTH_LONGTHROW, calibration_path)
        uv2xy = hl2ss_3dcv.compute_uv2xy(calibration.intrinsics, hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT)
        xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration.scale)
        intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_LONGTHROW.WIDTH, hl2ss.Parameters_RM_DEPTH_LONGTHROW.HEIGHT, calibration.intrinsics[0, 0], calibration.intrinsics[1, 1], calibration.intrinsics[2, 0], calibration.intrinsics[2, 1])

    else:
        calibration = hl2ss_3dcv.get_calibration_rm(None, hl2ss.StreamPort.RM_DEPTH_AHAT, calibration_path)
        uv2xy = hl2ss_3dcv.compute_uv2xy(calibration.intrinsics, hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT)
        xy1, scale = hl2ss_3dcv.rm_depth_compute_rays(uv2xy, calibration.scale / hl2ss.Parameters_RM_DEPTH_AHAT.FACTOR)
        intrinsics_depth = o3d.camera.PinholeCameraIntrinsic(hl2ss.Parameters_RM_DEPTH_AHAT.WIDTH, hl2ss.Parameters_RM_DEPTH_AHAT.HEIGHT, calibration.intrinsics[0, 0], calibration.intrinsics[1, 1], calibration.intrinsics[2, 0], calibration.intrinsics[2, 1])


    volume = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel_length, sdf_trunc=sdf_trunc, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    pcd_counter = 0

    for data_depth, data_vlc in tqdm(zip(data_depth_list, data_lf_list)):
        depth = hl2ss_3dcv.rm_depth_undistort(data_depth.payload.depth, calibration.undistort_map)
        depth = hl2ss_3dcv.rm_depth_normalize(depth, scale)
        
        color = cv2.remap(data_vlc.payload, calibration_vlc.undistort_map[:, :, 0], calibration_vlc.undistort_map[:, :, 1], cv2.INTER_LINEAR)

        lt_points         = hl2ss_3dcv.rm_depth_to_points(xy1, depth)
        lt_to_world       = hl2ss_3dcv.camera_to_rignode(calibration.extrinsics) @ hl2ss_3dcv.reference_to_world(data_depth.pose)
        world_to_lt       = hl2ss_3dcv.world_to_reference(data_depth.pose) @ hl2ss_3dcv.rignode_to_camera(calibration.extrinsics)
        world_to_pv_image = hl2ss_3dcv.world_to_reference(data_vlc.pose) @ hl2ss_3dcv.rignode_to_camera(calibration_vlc.extrinsics) @ hl2ss_3dcv.camera_to_image(calibration_vlc.intrinsics)
        world_points      = hl2ss_3dcv.transform(lt_points, lt_to_world)
        pv_uv             = hl2ss_3dcv.project(world_points, world_to_pv_image)
        color             = cv2.remap(color, pv_uv[:, :, 0], pv_uv[:, :, 1], cv2.INTER_LINEAR)

        mask_uv = hl2ss_3dcv.slice_to_block((pv_uv[:, :, 0] < 0) | (pv_uv[:, :, 0] >= hl2ss.Parameters_RM_VLC.WIDTH) | (pv_uv[:, :, 1] < 0) | (pv_uv[:, :, 1] >= hl2ss.Parameters_RM_VLC.HEIGHT))
        depth[mask_uv] = 0
        
        # Mask min depth
        depth[depth < min_depth] = 0
        
        color = hl2ss_3dcv.rm_vlc_to_rgb(color)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(color), o3d.geometry.Image(depth), depth_scale=1, depth_trunc=max_depth, convert_rgb_to_intensity=False)
        depth_world_to_camera = hl2ss_3dcv.world_to_reference(data_depth.pose) @ hl2ss_3dcv.rignode_to_camera(calibration.extrinsics)

        pcd_counter += 1
        volume.integrate(rgbd, intrinsics_depth, depth_world_to_camera.transpose())

    timer = time.time()- start_time

    print(f'TSDF integration took: {timer}')
    print(f'Used {pcd_counter} point clouds')

    target_pcd = volume.extract_point_cloud()

    if save_folder is not None:
        if save_folder.is_dir():
            filename = save_folder / f'target_{pcd_counter}_pcd.pcd'
            o3d.io.write_point_cloud(str(filename), target_pcd)

            filename = save_folder / f'time_volume_integration.txt'
            np.savetxt(str(filename), np.array([timer]), fmt='%.4f')

    return target_pcd
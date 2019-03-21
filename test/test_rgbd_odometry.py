# examples/Python/Tutorial/Basic/rgbd_odometry.py

from open3d import *
import numpy as np
PROJECT_PATH = '/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/'

# include my lib
import sys, os
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"
sys.path.append(PYTHON_FILE_PATH + "../include")
from lib_cloud_registration import *


Z_MAX=0.8
Z_MIN=0.2

if __name__ == "__main__":
    pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(
            PROJECT_PATH+"/config/cam_params.json")
    print(pinhole_camera_intrinsic.intrinsic_matrix)
    
    cfolder = PROJECT_PATH+"data/mydata/image/image"
    dfolder = PROJECT_PATH+"data/mydata/depth/depth"

    source_color = read_image(cfolder+"00001.png")
    source_depth = read_image(dfolder+"00001.png")
    target_color = read_image(cfolder+"00002.png")
    target_depth = read_image(dfolder+"00002.png")

    source_rgbd_image = create_rgbd_image_from_color_and_depth(
            source_color, source_depth)
    target_rgbd_image = create_rgbd_image_from_color_and_depth(
            target_color, target_depth)
    target_pcd = create_point_cloud_from_rgbd_image(
            target_rgbd_image, pinhole_camera_intrinsic)
    target_pcd = filtCloudByRange(target_pcd, zmin=Z_MIN, zmax=Z_MAX)

    option = OdometryOption()
    odo_init = np.identity(4)
    option.min_depth=Z_MIN
    option.max_depth=Z_MAX
    print(option)

    # [success_color_term, trans_color_term, info] = compute_rgbd_odometry(
    #         source_rgbd_image, target_rgbd_image,
    #         pinhole_camera_intrinsic, odo_init,
    #         RGBDOdometryJacobianFromColorTerm(), option)
    [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromHybridTerm(), option)

    # if success_color_term:
    #     print("Using RGB-D Odometry")
    #     print(trans_color_term)
    #     source_pcd_color_term = create_point_cloud_from_rgbd_image(
    #             source_rgbd_image, pinhole_camera_intrinsic)
    #     source_pcd_color_term.transform(trans_color_term)
    #     draw_geometries([target_pcd, source_pcd_color_term])
    if success_hybrid_term:
        print("Using Hybrid RGB-D Odometry")
        print(trans_hybrid_term)
        source_pcd_hybrid_term = create_point_cloud_from_rgbd_image(
                source_rgbd_image, pinhole_camera_intrinsic)
        source_pcd_hybrid_term = filtCloudByRange(source_pcd_hybrid_term, zmin=Z_MIN, zmax=Z_MAX)

        source_pcd_hybrid_term.transform(trans_hybrid_term)
        draw_geometries([target_pcd, source_pcd_hybrid_term])
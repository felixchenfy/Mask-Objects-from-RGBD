#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Include common
import open3d
import numpy as np
import sys, os
import copy
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# Include my lib
sys.path.append(PYTHON_FILE_PATH + "../include")
# from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
from lib_cloud_registration import *


def display_inlier_outlier(cloud, ind):
    inlier_cloud = select_down_sample(cloud, ind)
    outlier_cloud = select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":
    folder=PYTHON_FILE_PATH+"/../data/"
    # filename="datapcd_005.pcd"
    # filename="datapcd_006.pcd"
    # filename="datapcd_007.pcd"
    filename="datapcd_016.pcd"
    cloud = open3d.read_point_cloud(folder+filename)

    cloud = filtCloudByRange(cloud, zmin=0.2, zmax=1.0)
    draw_geometries([cloud])

    # print("Downsample the point cloud with a voxel of 0.02")
    # voxel_down_pcd = voxel_down_sample(cloud, voxel_size = 0.005)
    # draw_geometries([voxel_down_pcd])
    # cl,ind = statistical_outlier_removal(voxel_down_pcd,
    #         nb_neighbors=20, std_ratio=2.0)
    # display_inlier_outlier(voxel_down_pcd, ind)


    print "end"
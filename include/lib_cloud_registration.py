#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
from open3d import *
import time
from lib_point_cloud import *
import os
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# --------------  REGISTRATION Function s(Mainly copied from Open3D website) ----------------
 
def computeFeaturesForGlobalRegistration(pcd, voxel_size):
    # http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return pcd_down, pcd_fpfh

def execute_global_registration(
    source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    # http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration
    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(4000000, 500))
    return result.transformation

def execute_fast_global_registration(source_down, target_down,
        source_fpfh, target_fpfh, voxel_size):
    # http://www.open3d.org/docs/tutorial/Advanced/fast_global_registration.html
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            FastGlobalRegistrationOption(
            maximum_correspondence_distance = distance_threshold))
    return result.transformation
    
def registerClouds_Global(src, dst, voxel_size=0.01, FAST_REGI=True):
    # http://www.open3d.org/docs/tutorial/Advanced/fast_global_registration.html
    src_down, src_fpfh = computeFeaturesForGlobalRegistration(src, voxel_size)
    dst_down, dst_fpfh = computeFeaturesForGlobalRegistration(dst, voxel_size)
    if FAST_REGI:
        T = execute_fast_global_registration(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size)
    else:
        T = execute_global_registration(src_down, dst_down, src_fpfh, dst_fpfh, voxel_size)
    return T, src_down, dst_down

def registerClouds_Local(src, target, voxel_size=0.01, current_T=None, 
    ICP = True, COLORED_ICP = False, ICP_OPTION="PointToPlane",
    DRAW_INIT_POSE = False, DRAW_ICP = False, DRAW_COLORED_ICP = False):
    
    # -- Use colored ICP to register src onto dst, and return the combined cloud
    # This function is mainly copied from here.
    # http://www.open3d.org/docs/tutorial/Advanced/colored_pointcloud_registration.html
    
    # -- Params
    ICP_distance_threshold = voxel_size*4
    voxel_radiuses = [voxel_size*2.0, voxel_size, voxel_size/2.0]
    max_iters = [50, 25, 10]
    if current_T is None:
        current_T = np.identity(4)

    if DRAW_INIT_POSE:
        tmp = mergeClouds(src, target, voxel_size)
        drawCloudWithCoord(tmp, coord_axis_length=0.1, num_points_in_axis=50)

    # -- Point to plane ICP
    if ICP:
        print("Running ICP ...")
        src_down = voxel_down_sample(src, voxel_size)
        target_down = voxel_down_sample(target, voxel_size)
        estimate_normals(src_down, KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))
        
        if ICP_OPTION == "PointToPlane": # or "PointToPoint"
            result_trans = registration_icp(src_down, target_down, ICP_distance_threshold,
                current_T, 
                TransformationEstimationPointToPlane())
        else:
            result_trans = registration_icp(src_down, target_down, ICP_distance_threshold, 
                current_T,
                TransformationEstimationPointToPoint())
            
        current_T = result_trans.transformation
        if DRAW_ICP:
            print("-- Draw ICP result:")
            print(result_trans)
            my_sleep(1)
            drawTwoClouds(
                src, target, result_trans.transformation)

    # -- Colored pointcloud registration
    if COLORED_ICP:
        print "Running colored-ICP ...",
        for ith_loop in range(len(voxel_radiuses)):
            # Set param in this loop
            max_iter = max_iters[ith_loop]
            radius = voxel_radiuses[ith_loop]
            print " radius {:.4f}...".format(radius),
    
            # Downsample
            src_down = voxel_down_sample(src, radius)
            target_down = voxel_down_sample(target, radius)
    
            # Estimate normal
            estimate_normals(src_down, KDTreeSearchParamHybrid(
                radius=radius * 2, max_nn=30))
            estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius=radius * 2, max_nn=30))
    
            # Applying colored point cloud registration
            result_trans = registration_colored_icp(src_down, target_down,
                  radius, current_T,
                  ICPConvergenceCriteria(relative_fitness=1e-5,
                 relative_rmse=1e-5, max_iteration=max_iter))
            
            current_T = result_trans.transformation
        print "Complete!"
        if DRAW_COLORED_ICP:
            print("-- Draw Colored ICP result:")
            print(result_trans)
            my_sleep(1)
            drawTwoClouds(
                src, target, result_trans.transformation)
    print "Local registration completes.\n",    
    return current_T



class CloudRegister(object):
    def __init__(self, voxel_size_regi=0.005, global_regi_ratio=2.0, voxel_size_output=0.005, 
        USE_GLOBAL_REGI=True, USE_ICP=True, USE_COLORED_ICP=False):

        # copy params
        self.voxel_size_regi=voxel_size_regi # for registration
        self.global_regi_ratio=global_regi_ratio 
        self.voxel_size_output=voxel_size_output # for downsampling the res_cloud and output
        self.USE_GLOBAL_REGI = USE_GLOBAL_REGI
        self.USE_ICP = USE_ICP
        self.USE_COLORED_ICP = USE_COLORED_ICP
        
        # init vars
        self.res_cloud = open3d.PointCloud()
        self.new_cloud = open3d.PointCloud()
        self.prev_res_cloud = open3d.PointCloud()
        self.cnt_cloud=0

    def addCloud(self, new_cloud):
        self.new_cloud = copy.deepcopy(new_cloud)
        self.prev_res_cloud = copy.deepcopy(self.res_cloud)
        self.cnt_cloud+=1
        if self.cnt_cloud==1:
            self.res_cloud=copy.deepcopy(self.new_cloud)
        else:
            # compute transformation matrix to rotate new_cloud to the res_cloud frame
            T=np.identity(4)

            # Global regi
            if self.USE_GLOBAL_REGI:
                T, src_down, dst_down = registerClouds_Global(
                    self.new_cloud, self.res_cloud, self.voxel_size_regi * self.global_regi_ratio,
                    FAST_REGI=True)
            
            # Local regi
            if self.USE_ICP or self.USE_COLORED_ICP:
                T = registerClouds_Local(self.new_cloud, self.res_cloud, self.voxel_size_regi, 
                    T, ICP=self.USE_ICP, COLORED_ICP=self.USE_COLORED_ICP)

            # Merge
            self.res_cloud = mergeClouds(
                self.new_cloud, self.res_cloud, self.voxel_size_output, T)

        # self.res_cloud, 
        return self.res_cloud

    def drawRegisterInput(self):
        tmp = mergeClouds(self.new_cloud, self.prev_res_cloud, self.voxel_size_output)
        drawCloudWithCoord(tmp)
    
    def drawRegisterOutput(self):
        drawCloudWithCoord(self.res_cloud)

    def getResult(self):
        return self.res_cloud

# ====================================================================
# ============================ TESTS =================================
# ====================================================================

    
def test_registration():
  
    # -- Settings
    filename_=PYTHON_FILE_PATH+"../data/data/driller/segmented_"
    # filename_=PYTHON_FILE_PATH+"../data/data/driller_floor/segmented_0"
    # filename_=PYTHON_FILE_PATH+"../data/data/segmented_0"
    cloud_register = CloudRegister(
        voxel_size_regi=0.005, global_regi_ratio=2.0, 
        voxel_size_output=0.005,
        USE_ICP=True, USE_COLORED_ICP=True)

    # -- Loop
    FILE_INDEX_BEGIN=1
    FILE_INDEX_END=11
    cnt = 0
    for file_index in range(FILE_INDEX_BEGIN, FILE_INDEX_END+1):
        print "==================== {}th file ======================".format(file_index)
        cnt+=1
        
        # Read point cloud
        filename = filename_+"{:02d}".format(file_index)+".pcd"
        new_cloud = read_point_cloud(filename)
        
        # Process cloud
        cloud_register.addCloud(new_cloud) 
        
        # Print and plot
        print(cloud_register.res_cloud)
        # my_sleep(0.3)
        # cloud_register.drawRegisterInput()
        # cloud_register.drawRegisterOutput()

    cloud_register.drawRegisterOutput()


if __name__ == "__main__":
    # test_registration()

    if 1:
        filename="/home/feiyu/baxterws/src/winter_prj/scan3d_by_baxter/data/data/segmented_06.pcd"
        cloud_disp = read_point_cloud(filename)
        drawCloudWithCoord(cloud_disp)
    if 0:
        res_cloud = open3d.PointCloud()
        for i in range(1, 1+10):
            print i
            
            cloud_disp = read_point_cloud(PYTHON_FILE_PATH+"../data/data/driller_1/segmented_"+"{:02d}".format(i)+".pcd")
            # cloud_disp = read_point_cloud(PYTHON_FILE_PATH+"../data/data/driller_color_board/segmented_"+"{:02d}".format(i)+".pcd")
            # cloud_disp = read_point_cloud(PYTHON_FILE_PATH+"../data/data/driller_floor/segmented_"+"{:02d}".format(i)+".pcd")
             # drawCloudWithCoord(cloud_disp)
            res_cloud = mergeClouds(res_cloud, cloud_disp)
        drawCloudWithCoord(res_cloud)

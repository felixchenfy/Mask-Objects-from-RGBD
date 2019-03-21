#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
from open3d import *
import open3d
import time
import os
import cv2
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# -------------- Basic operations on cloud ----------------

def my_sleep(t):
    if 1:
        time.sleep(t)

def clearCloud(cloud):
    cloud.points=Vector3dVector(np.ndarray((0,0)))
    cloud.colors=Vector3dVector(np.ndarray((0,0)))

def copyOpen3dCloud(src, dst):
    dst.points = copy.deepcopy(src.points)
    dst.colors = copy.deepcopy(src.colors)

def getCloudContents(cloud):
    return np.asarray(cloud.points), np.asarray(cloud.colors)

def formNewCloud(np_points, np_colors):
    cloud = PointCloud()
    cloud.points = Vector3dVector(np_points)
    cloud.colors = Vector3dVector(np_colors)
    return cloud

def mergeClouds(cloud1, cloud2, radius_downsample=None, T=None):
    if T is not None:
        cloud1_transed = copy.deepcopy(cloud1)
        cloud1_transed.transform(T)
    else:
        cloud1_transed = copy.deepcopy(cloud1)
    cloud1_points,cloud1_colors = getCloudContents(cloud1_transed)
    cloud2_points,cloud2_colors = getCloudContents(cloud2)

    out_points = np.vstack((cloud1_points, cloud2_points))
    out_colors = np.vstack((cloud1_colors, cloud2_colors))

    result = formNewCloud(out_points, out_colors)
    if radius_downsample is not None:
        result = voxel_down_sample(result, radius_downsample)
    return result

def resizeCloudXYZ(cloud, scale=1.0):
    cloud_points, cloud_colors = getCloudContents(cloud)
    cloud_points = cloud_points * scale
    new_cloud = formNewCloud(cloud_points, cloud_colors)
    return new_cloud

def moveCloudToCenter(cloud):
    cloud_points, cloud_colors = getCloudContents(cloud)
    cloud_points = cloud_points-np.mean(cloud_points,axis=0)
    new_cloud = formNewCloud(cloud_points, cloud_colors)
    return new_cloud

# -------------- Display ----------------
    
def getCloudSize(cloud):
    return np.asarray(cloud.points).shape[0]

def drawTwoClouds(c1, c2, T_applied_to_c1=None):
    c1_tmp = copy.deepcopy(c1)
    if T_applied_to_c1 is not None:
        c1_tmp.transform(T_applied_to_c1)
    draw_geometries([c1_tmp, c2])

color_map = {'r':[1.0, 0.0, 0.0],'g':[0.0, 1.0, 0.0], 'b':[0.0, 0.0, 1.0]}
def getColor(color):
    color_map[color]

def createXYZAxis(coord_axis_length=1.0, num_points_in_axis=10):
    xyz_offset=[0,0,0]
    xyz_color=['r','g','b']
    NUM_AXIS=3
    data_xyz=np.zeros((num_points_in_axis*NUM_AXIS,NUM_AXIS))
    data_rgb=np.zeros((num_points_in_axis*NUM_AXIS,NUM_AXIS))
    cnt_row=0
    for axis in range(3):
        color = color_map[xyz_color[axis]]
        offset = xyz_offset[axis]
        for cnt_points in range(1, 1+num_points_in_axis):
            data_xyz[cnt_row, axis]=offset+coord_axis_length*cnt_points/num_points_in_axis
            data_rgb[cnt_row,:]=color
            cnt_row+=1
    cloud_XYZaxis = formNewCloud(data_xyz, data_rgb)
    return cloud_XYZaxis


def drawCloudWithCoord(cloud, coord_axis_length=0.1, num_points_in_axis=150):
    cloud_XYZaxis = createXYZAxis(coord_axis_length, num_points_in_axis)
    cloud_with_axis = mergeClouds(cloud, cloud_XYZaxis)
    draw_geometries([cloud_with_axis])

# -------------- Filters ----------------

def filtCloudByRange(cloud, xmin=None, xmax=None, 
                        ymin=None, ymax=None, zmin=None, zmax=None):
    none2maxnum =  lambda val: +99999.9 if val is None else val
    none2minnum =  lambda val: -99999.9 if val is None else val
    xmax=none2maxnum(xmax)
    ymax=none2maxnum(ymax)
    zmax=none2maxnum(zmax)
    xmin=none2minnum(xmin)
    ymin=none2minnum(ymin)
    zmin=none2minnum(zmin)
    criteria = lambda x,y,z: \
        x>=xmin and x<=xmax and y>=ymin and y<=ymax and z>=zmin and z<=zmax
    return filtCloud(cloud, criteria)

def filtCloud(cloud, criteria):
    points, colors = getCloudContents(cloud)
    num_pts=points.shape[0]
    valid_indices=np.zeros(num_pts, np.int)
    cnt_valid=0
    for i in range(num_pts):
        x,y,z=points[i][0],points[i][1],points[i][2]
        if criteria(x,y,z):
            valid_indices[cnt_valid]=i
            cnt_valid+=1
    return formNewCloud(
        points[valid_indices[:cnt_valid],:],
        colors[valid_indices[:cnt_valid],:]
    )
    

# -------------- Input and Output (IO)----------------

# Input folder name and index "ith_image", output color and depth image
def read_color_depth_images(folder, ith_image,
        output_img_format="open3d", index_len=5,
        image_folder="image", image_name="image",
        depth_doler="depth", depth_name="depth",
    ):
    '''
    test case{
        images_folder = "/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/data/mydata"
        cam_param_file="/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/config/cam_params.json"
        color, depth = read_color_depth_images(images_folder, 1)
        pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(cam_param_file)
        cloud = rgbd2cloud(color, depth, pinhole_camera_intrinsic)
        cloud = filtCloudByRange(cloud, zmin=0.2, zmax=0.8)
        open3d.draw_geometries([cloud])
    }
    '''
    cfolder = folder+"/"+image_folder+"/"+image_name
    dfolder = folder+"/"+depth_doler+"/"+depth_name

    str_idx = ("{:0"+str(index_len)+"d}").format(ith_image)
    suffix = str_idx+".png"

    if output_img_format=="cv" or output_img_format=="cv2":
        color_image = cv2.imread(cfolder+suffix, cv2.IMREAD_UNCHANGED)
        depth_image = cv2.imread(dfolder+suffix, cv2.IMREAD_UNCHANGED)
    else:
        color_image = read_image(cfolder+suffix)
        depth_image = read_image(dfolder+suffix)

    return color_image, depth_image

# convert color and depth image into point cloud
def rgbd2cloud(color, depth, pinhole_camera_intrinsic, output_img_format="open3d"):
    '''
    test case{
        images_folder = "/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/data/mydata"
        cam_param_file="/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/config/cam_params.json"
        color, depth = read_color_depth_images(images_folder, 1)
        pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(cam_param_file)
        cloud = rgbd2cloud(color, depth, pinhole_camera_intrinsic)
    }
    '''
    if output_img_format=="cv" or output_img_format=="cv2":
        rgbd_image = create_rgbd_image_from_color_and_depth(open3d.Image(cv2.cvtColor(color,cv2.COLOR_BGR2RGB)), open3d.Image(depth),
            convert_rgb_to_intensity=False)
    else:
        rgbd_image = create_rgbd_image_from_color_and_depth(color, depth,
            convert_rgb_to_intensity=False)
    cloud = create_point_cloud_from_rgbd_image(
        rgbd_image, pinhole_camera_intrinsic)
    return cloud

def image_conversion_bewteen_open3d_opencv():
    # img = open3d.Image(some_numpy_array)
    # arr = np.asarray(some_open3d_image)
    None # See above two lines
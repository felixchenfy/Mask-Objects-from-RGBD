#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
import cv2
from matplotlib import pyplot as plt
from open3d import *
import time
import os, sys
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# ROS
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import String

# my libriries
sys.path.append(PYTHON_FILE_PATH + "../include")
from lib_cloud import *

# ==================================================================================================


images_folder = "/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/data/mydata"
cam_param_file="/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/config/cam_params.json"
color, depth = read_color_depth_images(images_folder, 1, output_img_format="cv2")
pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(cam_param_file)
cloud = rgbd2cloud(color, depth, pinhole_camera_intrinsic, output_img_format="cv2")
cloud = filtCloudByRange(cloud, zmin=0.2, zmax=0.5)
print cloud
print np.asarray(cloud.points).shape
open3d.draw_geometries([cloud])

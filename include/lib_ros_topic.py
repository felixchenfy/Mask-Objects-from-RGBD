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
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

# My libraries
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

# ======================================== Subscriber ========================================
# Subscribe image topic
class ImageSubscriber(object):
    def __init__(self, topic_name):
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(topic_name, Image, self._call_back)

        self.cnt = 0  # count images
        self.rosImage=None
        self.t=None
        self.is_image_updated=False

    
    def _call_back(self, rosImage):
        self.rosImage =rosImage
        self.t=rospy.get_time()
        self.cnt+=1
        self.is_image_updated=True


    def _get_image(self):
        self.is_image_updated=False
        return self.rosImage, self.t

    def isReceiveImage(self):
        return self.is_image_updated

# Subscribe color image topic
class ColorImageSubscriber(ImageSubscriber):
    def __init__(self, topic_name):
        super(ColorImageSubscriber, self).__init__(topic_name)
   
    def get_image(self):
        rosImage, t = self._get_image()
        return self.bridge.imgmsg_to_cv2(rosImage, "bgr8"), t

# Subscribe depth image topic
class DepthImageSubscriber(ImageSubscriber):
    def __init__(self, topic_name):
        super(DepthImageSubscriber, self).__init__(topic_name)
   
    def get_image(self):
        rosImage, t = self._get_image()
        return self.bridge.imgmsg_to_cv2(rosImage, "16UC1"), t# not 32FC1


# ======================================== Publisher ========================================
class ImagePublisher(object):
    def __init__(self, image_topic, img_format="rgb", queue_size=10):
        self.bridge = CvBridge()
        self.pub=rospy.Publisher(image_topic, Image, queue_size=queue_size)
        if img_format=="rgb" or img_format=="color":
            self.format="bgr8"
        else: # depth
            self.format="16UC1"
            
    def publish(self, image):
        self.pub.publish(self.bridge.cv2_to_imgmsg(image, self.format))

class CloudPublisher(object):
    
    def __init__(self, topic_name):
        self.pub = rospy.Publisher(topic_name, PointCloud2, queue_size=5)
        self.cnt = 0
    def publish(self, cloud, cloud_format="open3d"):
        if cloud_format=="open3d":
            cloud = convertCloudFromOpen3dToRos(cloud)
        else: # ROS cloud
            None
        self.pub.publish(cloud)
        self.cnt += 1

class CameraInfoPublisher():
    def __init__(self, topic_name):
        self.pub = rospy.Publisher(topic_name, CameraInfo, queue_size=5)

    def publish(self, width, height, intrinsic_matrix):
        cam_info=CameraInfo()
        cam_info.height=height
        cam_info.width=width
        K=list()
        for i in range(3):
            for j in range(3):
                K.append(intrinsic_matrix[i][j])
        cam_info.K=K
        self.pub.publish(cam_info)

    def publish_open3d_format(self, camera_intrinsic): # 
        K = camera_intrinsic.intrinsic_matrix
        width = camera_intrinsic.width
        height = camera_intrinsic.height
        self.publish(width, height, K)
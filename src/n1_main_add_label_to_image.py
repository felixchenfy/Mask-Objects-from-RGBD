#!/usr/bin/env python
# -*- coding: utf-8 -*-


# ROS
import rospy, math
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2
import sensor_msgs

# Common
import numpy as np
import copy
import cv2
from matplotlib import pyplot as plt
import open3d
import time
import os, sys
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"
from collections import deque
import datetime


# my libriries
sys.path.append(PYTHON_FILE_PATH + "../include")
from lib_ros_topic import ColorImageSubscriber, DepthImageSubscriber
from detect_object_from_rgbd import ObjectDetectorFromRGBD
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

def draw_images(color, depth):
    depth_in_color = cv2.cvtColor((depth/10).astype(np.uint8),cv2.COLOR_GRAY2RGB)
    cv2.imshow("rgb + depth", np.hstack([color, depth_in_color]))
    q = cv2.waitKey(1)
    return chr(q)

def get_time():
    s=str(datetime.datetime.now())[5:].replace(' ','-').replace(":",'-').replace('.','-')[:-3]
    return s # day, hour, seconds: 02-26-15-51-12-556

def int2str(num, blank):
    return ("{:0"+str(blank)+"d}").format(num)

class SaveDetectionResult(object):
    def __init__(self):
        self.idx = 0
        
        # file name
        self.simage = 'image'
        self.sdepth = 'depth'
        self.smask = 'mask'
        self.sresimg = 'resimg'
        self.sobjects = 'objects'
        self.sclouds = 'clouds'

        # file folder
        self.folder = PYTHON_FILE_PATH+"/../data/"+get_time()+"/" 
        self.fsimage = self.folder + self.simage
        self.fsdepth = self.folder + self.sdepth
        self.fsmask = self.folder + self.smask
        self.fsresimg = self.folder + self.sresimg
        self.fsobjects = self.folder + self.sobjects
        self.fsclouds = self.folder + self.sclouds
        os.mkdir(self.folder)
        os.mkdir(self.fsimage)
        os.mkdir(self.fsdepth)
        os.mkdir(self.fsmask)
        os.mkdir(self.fsresimg)
        os.mkdir(self.fsobjects)
        os.mkdir(self.fsclouds)

    def save(self, color, depth, mask, img_disp, obj_bboxes, obj_3d_centers, cloud):
        self.idx += 1
        print "saveing {:05d}th data to file ... ".format(self.idx)

        index = int2str(self.idx, 5) + "_"
        
        # Write images
        cv2.imwrite(self.fsimage + "/" + index + self.simage + ".png", color)
        cv2.imwrite(self.fsdepth + "/" + index + self.sdepth + ".png", depth)
        cv2.imwrite(self.fsmask + "/" + index + self.smask + ".png", mask)
        cv2.imwrite(self.fsresimg + "/" + index + self.sresimg + ".png", img_disp)

        # Write bounding box and other info
        with open(self.fsobjects + "/" + index + self.sobjects + ".txt", 'w') as f:
            for i in range(len(obj_bboxes)):

                # object index
                f.write(str(i))
                
                # image size
                f.write(": " + str(color.shape[0:2]))

                # bounding box
                f.write(": " + obj_bboxes[i].converToStr())
                
                # object 3d centers
                f.write(": " + str(obj_3d_centers[i]))

                f.write("\n")

        # Write cloud data
        open3d.write_point_cloud(
                self.fsclouds + "/" + index + self.sclouds + ".pcd", cloud)
            
        return

# =============================================================================
if __name__=="__main__":
    
    # Init node
    node_name='detect_object_from_pc'
    rospy.init_node(node_name)
    rospy.sleep(0.3)
    
    # Image subscriber
    sub_color = ColorImageSubscriber(rospy.get_param("topic_color_image"))
    sub_depth = DepthImageSubscriber(rospy.get_param("topic_depth_image"))

    # Object detector
    th = 0.5
    detector = ObjectDetectorFromRGBD(
            "filename_camera_info",
            "topic_point_cloud",
            "topic_point_cloud_objects",
            "topic_num_objects",
            "topic_objects_on_image",
            max_wait_time = 0.5,
            xmin=-th, xmax=th, ymin=-th, ymax=th,
            zmin = 0.1, zmax = 0.5, voxel_size = 0.003)

    # Save
    result_saver = SaveDetectionResult()

    # Process
    cnt = 0
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        if(sub_color.isReceiveImage() and sub_depth.isReceiveImage):
            try:
                color, t1 = sub_color.get_image()
                depth, t2 = sub_depth.get_image()
            except:
                continue
            cnt+=1

            print "\n======================================"
            print "Node 1: Publishes {:02d}th cloud to .cpp node for detecting objects".format(cnt)

            # Input color and depth image; Receive point clouds of each object
            obj_clouds, cloud_src = detector.get_objects_clouds_from_RGBD(color, depth)

            # Get bounding box and mask
            obj_bboxes, mask, img_disp = detector.get_bbox_and_mask_from_clouds(obj_clouds, color)

            # Get x,y,z coordinate of each object cloud
            obj_3d_centers = detector.getClouds3dCenters(obj_clouds)

            # Write to file
            if 1:
                cv2.imshow("detected_object", img_disp)
                q = cv2.waitKey(10)
                print(chr(q))
                if q!=-1 and chr(q)=='s':
                    print("!!!!!!!!!!!!!!!!!!!")
                    print("!!! SAVE RESULT !!!")
                    print("!!!!!!!!!!!!!!!!!!!")
                    result_saver.save(color, depth, mask, img_disp, obj_bboxes, obj_3d_centers, cloud_src)
            
    # plt.show()
    cv2.destroyAllWindows()
    print "Node stops"


    

    
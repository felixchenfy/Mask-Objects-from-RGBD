#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
import cv2
from matplotlib import pyplot as plt
from open3d import *
import time, datetime
import os, sys
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

# ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# my libriries
sys.path.append(PYTHON_FILE_PATH + "../../include")
from lib_cloud import *
from lib_ros_topic import ColorImageSubscriber, DepthImageSubscriber
PYTHON_FILE_PATH = os.path.join(os.path.dirname(__file__))+"/"

def get_time():
    s=str(datetime.datetime.now())[5:].replace(' ','-').replace(":",'-').replace('.','-')[:-3]
    return s # day, hour, seconds: 02-26-15-51-12-556

def write_images_to_file(color, depth, folder, index):
    idx="{:05d}".format(cnt)
    f1=folder+"depth"+idx+".png"
    f2=folder+"image"+idx+".png"
    cv2.imwrite(f1, depth) 
    cv2.imwrite(f2, color)
    print "saving "+idx+"th image to: ", f1
    
if __name__ == '__main__':
    node_name='read_image_from_realsense'
    rospy.init_node(node_name)

    TOPIC_COLOR_IMAGE="/camera/color/image_raw"
    TOPIC_DEPTH_IMAGE="/camera/aligned_depth_to_color/image_raw"
    DATA_FOLDER = PYTHON_FILE_PATH+'../../data/source/'

    sub_color = ColorImageSubscriber(TOPIC_COLOR_IMAGE)
    sub_depth = DepthImageSubscriber(TOPIC_DEPTH_IMAGE)
    rospy.sleep(1)

    print  node_name+": node starts!!!"
    print " === Press 's' to save image === "
    
    flag_record = False
    while not rospy.is_shutdown():
        if(sub_color.isReceiveImage() and sub_depth.isReceiveImage):
            color, t1 = sub_color.get_image()
            depth, t2 = sub_depth.get_image()
            
            # draw
            depth_in_color = cv2.cvtColor((depth/10).astype(np.uint8),cv2.COLOR_GRAY2RGB)
            cv2.imshow("rgb + depth", np.hstack([color, depth_in_color]))
            q = cv2.waitKey(1)

            if q==-1:
                q = 0
            if chr(q)=='q':
                break
            elif chr(q)=='s': # Record
                if flag_record == False:
                    print("\n\n==========================")
                    print("Start recording\n")
                    flag_record = True
                    folder = DATA_FOLDER + get_time() + "/"
                    if not os.path.exists(folder):
                        os.mkdir(folder)
                    cnt = 0
            elif chr(q)=='d':
                flag_record = False
                print("\n\n==========================")
                print("Stop recording\n")

            if flag_record==True:
                cnt+=1
                write_images_to_file(color, depth, folder, cnt)

        rospy.sleep(0.01)

    plt.show()
    cv2.destroyAllWindows()
    print "Node stops"
    # rospy.spin()
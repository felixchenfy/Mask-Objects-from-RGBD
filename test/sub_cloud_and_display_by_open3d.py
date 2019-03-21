#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Include common
import open3d
import numpy as np
import sys, os
import copy
PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"

# Include ros
import rospy
from sensor_msgs.msg import PointCloud2

# Include my lib
rospy.loginfo("----------------------------:"+PYTHON_FILE_PATH)
sys.path.append(PYTHON_FILE_PATH + "../include")
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

# Global Vars
# PC_TOPIC = "camera/depth_registered/points"
PC_TOPIC = "/camera/depth/color/points"

def copyOpen3dCloud(src, dst):
    dst.points=src.points
    dst.colors=src.colors

# Main
def main_display_cloud():
    global received_ros_cloud
    # Set viewer
    open3d_cloud= open3d.PointCloud()
    vis = open3d.Visualizer()
    vis.create_window()
    vis.add_geometry(open3d_cloud)

    # Loop
    rate = rospy.Rate(100)
    cnt = 0
    while not rospy.is_shutdown():
        if received_ros_cloud is not None:
            cnt+=1
            idx_method=1
            if idx_method==1:
                tmp = convertCloudFromRosToOpen3d(received_ros_cloud)
                copyOpen3dCloud(tmp, open3d_cloud)
                vis.add_geometry(open3d_cloud) 
            if idx_method==2: # this is not working!!!
                open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
            if idx_method==3: # this is keep adding clouds to viewer
                open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
                vis.add_geometry(open3d_cloud)
            vis.update_geometry()
            print("Updating geometry for the {}th time".format(cnt))
            received_ros_cloud = None # clear
        vis.poll_events()
        vis.update_renderer()
        rate.sleep()
        # print("Updating viewer")

    # Return
    vis.destroy_window()

def main_save_cloud():
    global received_ros_cloud
    rate = rospy.Rate(100)
    cnt = 0
    file_folder="/home/feiyu/baxterws/src/winter_prj/mask_objects_from_rgbd/data/"
    file_name="pcd_"
    while not rospy.is_shutdown():
        if received_ros_cloud is not None:
            cnt+=1
            open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
            filename_to_save = file_folder+file_name+"{:03d}".format(cnt)+".pcd"
            open3d.write_point_cloud(filename_to_save, open3d_cloud)
            print("Saving"+filename_to_save)
            received_ros_cloud = None # clear
        rate.sleep()
        # print("Updating viewer")

if __name__ == "__main__":
    node_name = "sub_cloud_and_display_by_open3d"
    rospy.init_node(node_name, anonymous=True)

    # Params settings
    topic_name_rgbd_cloud = rospy.get_param("topic_name_rgbd_cloud", PC_TOPIC)

    # Set subscriber
    global received_ros_cloud
    received_ros_cloud = None
    def callback(ros_cloud):
        global received_ros_cloud
        if received_ros_cloud is None:
            received_ros_cloud=ros_cloud
            rospy.loginfo("Received ROS PointCloud2 message.")
    rospy.Subscriber(topic_name_rgbd_cloud, PointCloud2, callback)      

    # Call main   
    # main_display_cloud()    
    main_save_cloud()
    rospy.loginfo("This node stops: " + node_name)
    

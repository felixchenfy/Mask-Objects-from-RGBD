/*
Function:
- Input: A point cloud containing a table surface and multiple object on the table.
- Output: point cloud cluster of each object.
    First publish the number of objects on "topic_num_objects"
    Then successively publish the point cloud of each object onto the topic of "topic_point_cloud_objects".
    Meanwhile, for visualization, this node publishes the same cloud as input but with annotated color onto "topic_point_cloud_clustering_res"

- Input topic:
topic_point_cloud

- Output topic:
topic_num_objects
topic_point_cloud_objects
topic_point_cloud_clustering_res

*/

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

#include "detect_object_from_cloud.h"

using namespace std;
using namespace pcl;
typedef PointCloud<PointXYZRGB>::Ptr CloudPtr;

void main_loop()
{
    ObjectDetectorFromCloud detector;
    while (ros::ok())
    {
        if (detector.hasCloud())
        {
            CloudPtr cloud_src = detector.popCloud();
            vector<CloudPtr> cloud_clusters = detector.detect_objects_from_cloud(cloud_src);
            detector.publishResults(cloud_clusters);
        }
        ros::spinOnce(); // In python, sub is running in different thread. In C++, only one thread. So need this.
        ros::Duration(0.01).sleep();
    }
}

int main(int argc, char **argv)
{
    string node_name = "node_detect_object_from_cloud";
    ros::init(argc, argv, node_name);

    // Call main
    main_loop();

    // End
    ROS_INFO("Node2 stops");
    return 0;
}

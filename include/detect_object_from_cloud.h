// This is for ROS node of "node_detect_object_from_cloud.cpp"

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"

#include "my_basics/basics.h"
#include "my_pcl/pcl_visualization.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_filters.h"
#include "my_pcl/pcl_advanced.h"
#include "my_pcl/pcl_io.h"

using namespace std;
using namespace pcl;
typedef PointCloud<PointXYZRGB>::Ptr CloudPtr;

class ObjectDetectorFromCloud
{
  public:
    ObjectDetectorFromCloud();
    void callback_sub_pointcloud_(const sensor_msgs::PointCloud2 &ros_cloud);
    void pubPclCloudToTopic(ros::Publisher &pub, CloudPtr pcl_cloud);
    int numCloud() { return (int)buff_cloud_src.size(); }
    bool hasCloud() { return numCloud() > 0; }
    CloudPtr popCloud();
    vector<CloudPtr> detect_objects_from_cloud(CloudPtr cloud_src);
    // CloudPtr remove_points_beneath_the_plane(CloudPtr cloud_src, ); // TO DO
    void publishResults(vector<CloudPtr> cloud_clusters);

  private:
    // ============================= Variables =============================
    string topic_point_cloud, topic_point_cloud_clustering_res, topic_point_cloud_objects, topic_num_objects;
    string folder_data;
    queue<CloudPtr> buff_cloud_src; // When the node sub cloud from topic, save it to here

    // ===================== Subscriber and Publisher ======================
    ros::Subscriber sub_pointcloud_;
    ros::Publisher pub_pc_clustering_result_;
    ros::Publisher pub_pc_objects_;
    ros::Publisher pub_num_objects_;

    // ============================= Parameters =============================
    // Filter: plane segmentation
    float plane_distance_threshold_;
    int plane_max_iterations_;
    int num_planes_;
    float ratio_of_rest_points_;

    // Filter: divide cloud into clusters
    bool flag_do_clustering_;
    double cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_, max_num_objects_;
};

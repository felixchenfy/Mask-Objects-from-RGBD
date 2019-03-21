
// This is for ROS node of "node_detect_object_from_cloud.cpp"

#include "detect_object_from_cloud.h"

#define NH_GET_PARAM(param_name, returned_val)                              \
    if (!nh.getParam(param_name, returned_val))                             \
    {                                                                       \
        cout << "Error in reading ROS param named: " << param_name << endl; \
        assert(0);                                                          \
    }

ObjectDetectorFromCloud::ObjectDetectorFromCloud()
{
    {
        ros::NodeHandle nh;
        NH_GET_PARAM("topic_point_cloud", topic_point_cloud);
        NH_GET_PARAM("topic_point_cloud_clustering_res", topic_point_cloud_clustering_res);
        NH_GET_PARAM("topic_point_cloud_objects", topic_point_cloud_objects);
        NH_GET_PARAM("topic_num_objects", topic_num_objects);
        NH_GET_PARAM("folder_data", folder_data);
    }
    {
        ros::NodeHandle nh("~");

        // -- Segment plane
        NH_GET_PARAM("plane_distance_threshold_", plane_distance_threshold_)
        NH_GET_PARAM("plane_max_iterations_", plane_max_iterations_)
        NH_GET_PARAM("num_planes_", num_planes_)
        ratio_of_rest_points_ = -1; // disabled

        // -- Clustering
        NH_GET_PARAM("flag_do_clustering_", flag_do_clustering_)
        NH_GET_PARAM("cluster_tolerance_", cluster_tolerance_)
        NH_GET_PARAM("min_cluster_size_", min_cluster_size_)
        NH_GET_PARAM("max_cluster_size_", max_cluster_size_)
        NH_GET_PARAM("max_num_objects_", max_num_objects_)
    }
    {
        ros::NodeHandle nh;

        sub_pointcloud_ = nh.subscribe(topic_point_cloud, 5,
                                      &ObjectDetectorFromCloud::callback_sub_pointcloud_, this);

        pub_pc_clustering_result_ = nh.advertise<sensor_msgs::PointCloud2>(topic_point_cloud_clustering_res, 5);
        pub_pc_objects_ = nh.advertise<sensor_msgs::PointCloud2>(topic_point_cloud_objects, 5);
        pub_num_objects_ = nh.advertise<std_msgs::Int32>(topic_num_objects, 5);
    }
}

CloudPtr ObjectDetectorFromCloud::popCloud()
{
    CloudPtr cloud_src = buff_cloud_src.front();
    buff_cloud_src.pop();
    return cloud_src;
}

void ObjectDetectorFromCloud::publishResults(vector<CloudPtr> cloud_clusters)
{

    // Number of objects
    std_msgs::Int32 _num_objects;
    _num_objects.data = (int)cloud_clusters.size();
    pub_num_objects_.publish(_num_objects);

    // Object clustering result
    for (int i = 0; i < cloud_clusters.size(); i++)
    {
        CloudPtr cloud = cloud_clusters[i];
        // if (i > 0)
        //     ros::Duration(0.01).sleep(); // If lost message, I shall comment this out.
        printf("node 2 pubs %dth object of size %d\n", i, (int)cloud->points.size());
        pubPclCloudToTopic(pub_pc_objects_, cloud);
    }
}

void ObjectDetectorFromCloud::callback_sub_pointcloud_(const sensor_msgs::PointCloud2 &ros_cloud)
{
    static int cnt = 0;
    CloudPtr tmp(new PointCloud<PointXYZRGB>);
    fromROSMsg(ros_cloud, *tmp);
    buff_cloud_src.push(tmp);
    printf("\nNode 2 has subscribed %dth cloud. Ready to detect object...\n", ++cnt);
}

void ObjectDetectorFromCloud::pubPclCloudToTopic(ros::Publisher &pub, CloudPtr pcl_cloud)
{
    sensor_msgs::PointCloud2 ros_cloud_to_pub;
    pcl::toROSMsg(*pcl_cloud, ros_cloud_to_pub);
    ros_cloud_to_pub.header.frame_id = "base";
    pub.publish(ros_cloud_to_pub);
}

vector<CloudPtr> ObjectDetectorFromCloud::detect_objects_from_cloud(CloudPtr cloud_src)
{
    // Set output
    vector<CloudPtr> cloud_clusters;
    CloudPtr cloud_clustering_res(new PointCloud<PointXYZRGB>); // This is for visualizing

    // Init vars
    CloudPtr cloud_no_plane(new PointCloud<PointXYZRGB>);
    pcl::copyPointCloud(*cloud_src, *cloud_no_plane);

    // ---------------------------------------------
    // -- Remove planes
    CloudPtr plane(new PointCloud<PointXYZRGB>);
    int num_removed_planes = my_pcl::removePlanes(
        cloud_no_plane, plane,
        plane_distance_threshold_, plane_max_iterations_,
        num_planes_, ratio_of_rest_points_);

    // -- Clustering: Divide the remaining point cloud into different clusters
    // -- Do clustering
    vector<PointIndices> clusters_indices;
    clusters_indices = my_pcl::divideIntoClusters(
        cloud_no_plane, cluster_tolerance_, min_cluster_size_, max_cluster_size_);
    if (clusters_indices.size() > max_num_objects_)
        clusters_indices.resize(max_num_objects_);
    int num_object = clusters_indices.size();
    if (num_object == 0)
    {
        pcl::copyPointCloud(*cloud_src, *cloud_clustering_res);
    }
    else
    {
        // -- Extract sub clouds
        cloud_clusters = my_pcl::extractSubCloudsByIndices(cloud_no_plane, clusters_indices);

        // -- add colors
        for (PointXYZRGB &p : cloud_no_plane->points)
            my_pcl::setPointColor(p, 255, 255, 255); // Set all points' color to white

        for (PointXYZRGB &p : plane->points)
            my_pcl::setPointColor(p, 255, 255, 255); // Set all points' color to white

        vector<vector<unsigned char>> colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}});
        assert(max_num_objects_ <= colors.size());
        int i = 0;

        for (std::vector<PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
        { // iterate clusters_indices
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            { // iterate each point index
                my_pcl::setPointColor(cloud_no_plane->points[*pit], colors[i][0], colors[i][1], colors[i][1]);
            }
            i++;
        }

        // copy data
        pcl::copyPointCloud(*cloud_no_plane, *cloud_clustering_res);
        *cloud_clustering_res += *plane;
    }

    // Print result
    if (0)
    {
        cout << "size of cloud_src: ";
        my_pcl::printCloudSize(cloud_src);

        cout << "size of the " << cloud_clusters.size() << " objects: ";
        for (auto cloud : cloud_clusters)
            cout << cloud->width * cloud->height << ", ";

        cout << "\nsize of cloud_clustering_res: ";
        my_pcl::printCloudSize(cloud_clustering_res);
    }

    //
    pubPclCloudToTopic(pub_pc_clustering_result_, cloud_clustering_res);

    return cloud_clusters;
}
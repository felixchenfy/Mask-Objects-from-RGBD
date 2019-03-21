
#include "my_pcl/pcl_advanced.h"
#include "my_pcl/pcl_filters.h"
#include "my_pcl/pcl_commons.h"
#include "my_pcl/pcl_io.h"
#include "my_basics/basics.h"

namespace my_pcl
{

// Given a cloud and a vector of indices, return a vector of point clouds corresponding to the indices. 
vector<PointCloud<PointXYZRGB>::Ptr> extractSubCloudsByIndices(
    const PointCloud<PointXYZRGB>::Ptr cloud, const vector<PointIndices> &clusters_indices)
{
    vector<PointCloud<PointXYZRGB>::Ptr> cloud_clusters;
    for (std::vector<PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
    {
        PointCloud<PointXYZRGB>::Ptr cloud_cluster(new PointCloud<PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_clusters.push_back(cloud_cluster);
    }
    return cloud_clusters;
}

// Remove planes
int removePlanes(PointCloud<PointXYZRGB>::Ptr &cloud,
    PointCloud<PointXYZRGB>::Ptr &plane,
    float plane_distance_threshold_, int plane_max_iterations_,
    int stop_criteria_num_planes_, float stop_criteria_rest_points_ratio,
    bool print_res)
{
    assert(stop_criteria_num_planes_>=0 || stop_criteria_rest_points_ratio>=0);
    plane->clear();
    int total_points = (int)cloud->points.size();
    int cnt_planes=0;
    while(1){
        if(stop_criteria_num_planes_>=0){
            if(cnt_planes>=stop_criteria_num_planes_)break;
        }else{
            if(cloud->points.size() <= stop_criteria_rest_points_ratio * total_points)break;
        }        
        cnt_planes++;
        // -- detectPlane
        ModelCoefficients::Ptr coefficients;
        PointIndices::Ptr inliers;
        bool res = detectPlane(cloud, coefficients, inliers,
            plane_distance_threshold_, plane_max_iterations_);
        if (res==false){
            cnt_planes--;
            cout<<"my WARNING: removePlanes' iteration fails to reach the desired times."<<endl;
            break;
        }

        // -- extractSubCloudByIndices
        bool invert_indices = false;
        PointCloud<PointXYZRGB>::Ptr new_plane = extractSubCloudByIndices(cloud, inliers, invert_indices);
        cloud = extractSubCloudByIndices(cloud, inliers, !invert_indices);
        *plane+=*new_plane;

        if(print_res){
            printf("\n-----------------------------\n");
            printf("Detecting the %dth plane:\n\n", cnt_planes);

            printPlaneCoef(coefficients);
            cout << endl;

            cout << "Detected plane: ";
            printCloudSize(plane);
            // write_point_cloud("seg_res_plane_" + to_string(cnt_planes) + ".pcd", plane);

            cout << "The rest part: ";
            printCloudSize(cloud);
            // write_point_cloud("seg_res_remained_" + to_string(cnt_planes) + ".pcd", cloud);
            cout << endl;
        }
    }
    return cnt_planes;
}

// Do clustering using pcl::EuclideanClusterExtraction. Return the indices of each cluster.
vector<PointIndices> divideIntoClusters(const PointCloud<PointXYZRGB>::Ptr cloud,
        double cluster_tolerance_, int min_cluster_size_, int max_cluster_size_)
{

    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
    tree->setInputCloud(cloud);

    // Set extractor
    EuclideanClusterExtraction<PointXYZRGB> ec;
    ec.setClusterTolerance(cluster_tolerance_); // similar to mean distance between points inside a point cloud
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    vector<PointIndices> clusters_indices;
    ec.extract(clusters_indices);

    // sort based on size
    int N=clusters_indices.size();
    vector<PointIndices> clusters_indices_res;
    vector<int> indices_sizes;
    for(PointIndices &indices:clusters_indices)
        indices_sizes.push_back(indices.indices.size());
    vector<int> indices_size_order = my_basics::sort_indexes(indices_sizes);
    for(int i=N-1;i>=0;i--){
        clusters_indices_res.push_back(clusters_indices[indices_size_order[i]]);
    }
    return clusters_indices_res;
}


} // namespace my_pcl
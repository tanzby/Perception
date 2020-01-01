#include "lidar_object_detector/lidar_object_detector.hpp"
#include "lidar_object_detector/visualize_utils.hpp"

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_datatypes.h>
#include <random>

namespace lidar_object_detector
{
    LidarObjectDetector::LidarObjectDetector()
    :node_handle_(), private_node_handle_("~"){ init();}

    void LidarObjectDetector::init()
    {
        in_cloud_subscriber_ = node_handle_.subscribe("/filtered_rslidar_points", 1,
                &LidarObjectDetector::cloud_callback, this);

        clusters_cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>("/clusters_cloud", 1, true);
        clusters_msg_publisher_ = node_handle_.advertise<perception_msgs::DetectedObjectArray>("/detected_objects", 1, false);

        int min_n_pt;
        int max_n_pt;
        float min_dist;
        float cluster_tolerance;
        Eigen::Array3f min_size, max_size;
        private_node_handle_.param<float>("cluster_min_dist", min_dist, 0.5f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_min_dist: "<< min_dist);
        private_node_handle_.param<float>("cluster_tolerance", cluster_tolerance, 0.5f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_tolerance: "<< cluster_tolerance);
        private_node_handle_.param<int>("cluster_min_pts", min_n_pt, 8);
        ROS_INFO_STREAM("[ClusterDetector] cluster_min_pts: "<< min_n_pt);
        private_node_handle_.param<int>("cluster_max_pts", max_n_pt, 512);
        ROS_INFO_STREAM("[ClusterDetector] cluster_max_pts: "<< max_n_pt);

        private_node_handle_.param<float>("cluster_min_size_x", min_size.x(), 0.2f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_min_size_x: "<< min_size.x());
        private_node_handle_.param<float>("cluster_min_size_y", min_size.y(), 0.2f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_min_size_y: "<< min_size.y());
        private_node_handle_.param<float>("cluster_min_size_z", min_size.z(), 0.2f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_min_size_z: "<< min_size.z());

        private_node_handle_.param<float>("cluster_max_size_x", max_size.x(), 6.0f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_max_size_x: "<< max_size.x());
        private_node_handle_.param<float>("cluster_max_size_y", max_size.y(), 6.0f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_max_size_y: "<< max_size.y());
        private_node_handle_.param<float>("cluster_max_size_z", max_size.z(), 3.0f);
        ROS_INFO_STREAM("[ClusterDetector] cluster_max_size_z: "<< max_size.z());

        box_estimator_ = std::make_unique<BoxShapeEstimator>();
        cluster_detector_ = std::make_unique<ClusterDetector>(min_dist, min_size, max_size, min_n_pt, max_n_pt, cluster_tolerance);
    }

    void LidarObjectDetector::cloud_callback(const PointCloudT::ConstPtr &msg)
    {
        auto clusters = cluster_detector_->detect(msg);

        std_msgs::Header header = pcl_conversions::fromPCL(msg->header);
        pcl::PointCloud<pcl::PointXYZI> ret;
        perception_msgs::DetectedObjectArray object_array;

        for (int i = 0 ; i < clusters.size(); ++i)
        {
            auto cluster = clusters[i];
            perception_msgs::DetectedObject cluster_box;
            if (box_estimator_->estimate(cluster, cluster_box))
            {
                geometry_msgs::PolygonStamped polygon;
                polygon.header = header;
                PointCloudT convex_hull_points = cluster->get_convex();
                for(auto& polygon_p: convex_hull_points.points)
                {
                    geometry_msgs::Point32 p;
                    p.x = polygon_p.x;
                    p.y = polygon_p.y;
                    p.z = 0;
                    polygon.polygon.points.emplace_back(p);
                }
                cluster_box.id = i;
                cluster_box.label = "unknown";
                cluster_box.convex_hull = polygon;

                float color = ((i*49+21)%255);
                auto  cloud = cluster->cloud;
                for (auto& p: cloud->points)
                {
                    p.intensity = color;
                    ret.push_back(p);
                }

                object_array.objects.emplace_back(cluster_box);
            }
        }

        object_array.header = header;
        clusters_msg_publisher_.publish(object_array);
        clusters_cloud_publisher_.publish(ret);
    }
}
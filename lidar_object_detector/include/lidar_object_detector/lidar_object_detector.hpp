#pragma once

#include <ros/ros.h>
#include "lidar_object_detector/box_shape_estimation.hpp"
#include "lidar_object_detector/cluster_detector.hpp"
#include "lidar_object_detector/box_shape_estimation.hpp"

namespace lidar_object_detector
{
    class LidarObjectDetector
    {
        void init();
        void cloud_callback(const PointCloudT::ConstPtr &msg);
        void send_clusters(const std::vector<Cluster::Ptr>& cluster, const std_msgs::Header& header);

        std::unique_ptr<ClusterDetector>   cluster_detector_;
        std::unique_ptr<BoxShapeEstimator> box_estimator_;

        ros::Subscriber in_cloud_subscriber_;
        ros::Publisher  clusters_box_publisher_;
        ros::Publisher  clusters_cloud_publisher_;
        ros::Publisher  clusters_msg_publisher_;
        ros::Publisher  clusters_convex_publisher_;
        ros::NodeHandle node_handle_, private_node_handle_;

    public:

        LidarObjectDetector();

    };
}
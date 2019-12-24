#pragma once

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <hdmap_msgs/PolygonArrayStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <utility>

#include "points_preprocess/points_preprocess_type.hpp"
#include "points_preprocess/map_roi_filter/map_roi_filter.hpp"
#include "points_preprocess/ground_removal.hpp"
#include "points_preprocess/rules_based_filter.hpp"

namespace points_preprocess
{
    class PointsPreProcessor
    {
        void init();

        void cloud_callback(const PointCloudT::ConstPtr &msg);

        void construct_local_boundary();
        void map_roi_callback(const hdmap_msgs::PolygonArrayStamped::ConstPtr &msg);

        std::unique_ptr<RulesBasedFilter> rules_based_filter_;
        std::unique_ptr<GroundRemoval> ground_removal_;
        std::unique_ptr<MapROIFilter>  map_roi_filter_;
        hdmap_msgs::PolygonArrayStamped  latest_map_roi_concave_;

        tf::TransformListener tf_listener_;
        ros::NodeHandle node_handle_, private_node_handle_;

        ros::Publisher  out_cloud_publisher_;
        ros::Subscriber in_cloud_subscriber_;
        ros::Subscriber map_roi_subscriber_;


    public:

        PointsPreProcessor();

        void filter(const PointCloudT::ConstPtr& in_cloud, PointCloudT& out_cloud);
    };
}
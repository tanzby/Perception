#pragma once

#include <memory>
#include <pcl_ros/point_cloud.h>

namespace points_preprocess
{
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT>  PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;

    class Filter
    {
    public:
        virtual void process(const PointCloudT::ConstPtr& in_cloud,
                            const std::vector<int>& valid_indices,
                            std::vector<int>& inliers_indices) = 0;
    };
}

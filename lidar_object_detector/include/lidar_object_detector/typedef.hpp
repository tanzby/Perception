#pragma once

#include <memory>
#include <pcl_ros/point_cloud.h>

namespace lidar_object_detector
{
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT>  PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;
}

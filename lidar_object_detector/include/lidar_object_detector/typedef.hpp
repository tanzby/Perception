#pragma once

#include <memory>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include "perception_msgs/DetectedObject.h"
#include "perception_msgs/DetectedObjectArray.h"

namespace lidar_object_detector
{
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT>  PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;

    inline geometry_msgs::Quaternion getOrientFromYaw(double angle)
    {
        auto orient = tf::createQuaternionFromYaw(angle);
        geometry_msgs::Quaternion quaternion;
        quaternion.x = orient.x();
        quaternion.y = orient.y();
        quaternion.z = orient.z();
        quaternion.w = orient.w();
        return quaternion;
    }
    inline double getYawFromOrient(const geometry_msgs::Quaternion& quaternion)
    {
        return tf::getYaw(quaternion);
    }
}

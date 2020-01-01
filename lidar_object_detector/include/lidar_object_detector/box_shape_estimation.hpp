#pragma once

#include "lidar_object_detector/cluster.hpp"
#include "lidar_object_detector/typedef.hpp"
#include <vector>

namespace lidar_object_detector
{
    class BoxShapeEstimator
    {
        double calculate_closeness(const std::vector<double>& C_1, const std::vector<double>& C_2);

    public:

        BoxShapeEstimator();

        bool estimate(const Cluster::Ptr& cluster, perception_msgs::DetectedObject& box);

    };
}
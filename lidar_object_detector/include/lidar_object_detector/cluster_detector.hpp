#pragma once

#include "lidar_object_detector/cluster.hpp"
#include <Eigen/Dense>
#include <vector>


namespace lidar_object_detector
{
    class ClusterDetector
    {
        float min_dist_;
        int min_n_pt_;
        int max_n_pt_;
        float cluster_tolerance;
        Eigen::Array3f min_size_, max_size_;

    public:

        ClusterDetector(float min_dist,
                        const Eigen::Array3f& min_size,
                        const Eigen::Array3f& max_size,
                        int min_n_pt = 10, int max_n_pt = 512, float cluster_tolerance = 0.2f);

        std::vector<Cluster::Ptr> detect(const PointCloudT::ConstPtr& in_cloud);
    };
}

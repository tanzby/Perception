#pragma once

#include <memory>
#include <Eigen/Dense>
#include <pcl/surface/convex_hull.h>
#include "lidar_object_detector/typedef.hpp"

namespace lidar_object_detector
{
    struct Cluster
    {
        using Ptr = std::shared_ptr<Cluster>;

        Cluster(const PointCloudT::Ptr& cloud): cloud(cloud)
        {
            min_pt = max_pt = cloud->at(0).getArray3fMap();
            for(int i=1; i<cloud->size(); i++) {
                min_pt = cloud->at(i).getArray3fMap().min(min_pt);
                max_pt = cloud->at(i).getArray3fMap().max(max_pt);
            }

            size = max_pt - min_pt;
            centroid = (min_pt + max_pt) / 2.0f;
        }
        PointCloudT get_convex(int dim = 2)
        {
            PointCloudT convex_hull_points;
            pcl::ConvexHull<PointT> convex_hull;
            convex_hull.setDimension(dim);
            convex_hull.setInputCloud(cloud);
            convex_hull.reconstruct(convex_hull_points);
            return convex_hull_points;
        }

        Eigen::Array3f min_pt;
        Eigen::Array3f max_pt;
        Eigen::Array3f size;
        Eigen::Array3f centroid;
        PointCloudT::Ptr cloud;
    };
}
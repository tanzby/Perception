#include "lidar_object_detector/cluster_detector.hpp"
#include <pcl/segmentation/extract_clusters.h>

namespace lidar_object_detector
{

    ClusterDetector::ClusterDetector(float min_dist,
            const Eigen::Array3f& min_size,
            const Eigen::Array3f& max_size,
            int min_n_pt, int max_n_pt, float cluster_tolerance)
            : min_dist_(min_dist),
              min_n_pt_(min_n_pt),
              max_n_pt_(max_n_pt),
              min_size_(min_size),
              max_size_(max_size),
              cluster_tolerance(cluster_tolerance){}

    std::vector<Cluster::Ptr> ClusterDetector::detect(const PointCloudT::ConstPtr &in_cloud)
    {
        auto scaled = PointCloudT().makeShared();
        scaled->resize(in_cloud->size());

        Eigen::Array3f scale(1.0f, 1.0f, 0.01f);
        for (size_t i = 0; i<in_cloud->size(); ++i)
        {
            scaled->at(i).getArray3fMap() = in_cloud->at(i).getArray3fMap() * scale;
        }
        scaled->width = scaled->size();
        scaled->height = 1;
        scaled->is_dense = false;

        pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
        kdtree->setInputCloud(scaled);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::extractEuclideanClusters<PointT>(*scaled, kdtree, cluster_tolerance,
                cluster_indices, min_n_pt_, max_n_pt_);

        std::vector<Cluster::Ptr> clusters;
        clusters.reserve(24);

        for (auto & cluster_indice : cluster_indices) {
            auto cluster_cloud = PointCloudT::Ptr(new PointCloudT(*in_cloud, cluster_indice.indices));
            Cluster::Ptr cluster(new Cluster(cluster_cloud));
            float dist = cluster->centroid.matrix().head<2>().norm();
            if ((cluster->size < min_size_).any() || (cluster->size > max_size_).any() || dist < min_dist_) {
                continue;
            }
            clusters.emplace_back(cluster);
        }
        return clusters;
    }
}
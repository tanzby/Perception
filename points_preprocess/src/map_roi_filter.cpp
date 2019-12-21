#include <thread>
#include "points_preprocess/map_roi_filter/map_roi_filter.hpp"


namespace points_preprocess
{

    MapROIFilter::MapROIFilter(double range, double cell_size, double extend_dist):
    range_(range), cell_size_(cell_size), extend_dist_(extend_dist)
    {
        // init bitmap
        Eigen::Vector2d min_range(-range_, -range_);
        Eigen::Vector2d max_range(range_, range_);
        Eigen::Vector2d e_cell_size(cell_size_, cell_size_);
        bitmap_.Init(min_range, max_range, e_cell_size);
    }

    void
    MapROIFilter::process(const PointCloudT::ConstPtr& in_cloud,
                const std::vector<int>& valid_indice, std::vector<int>& inliers_indice)
    {
        if (bitmap_.Empty()) {
            ROS_WARN_STREAM("Invalid roi region");
            return;
        }
        std::vector<int> ret; ret.reserve(valid_indice.size());
        polygons_mt_.lock();
        for(size_t i = 0; i < valid_indice.size(); ++i) {
            int x = static_cast<int>(in_cloud->points[valid_indice[i]].x);
            int y = static_cast<int>(in_cloud->points[valid_indice[i]].y);
            Eigen::Vector2d e_pt(x, y);
            if (!bitmap_.IsExists(e_pt)) {
                continue;
            }
            if (bitmap_.Check(e_pt)) {
                ret.emplace_back(static_cast<int>(valid_indice[i]));
            }
        }
        polygons_mt_.unlock();
        inliers_indice.swap(ret);
    }

    Bitmap2D::DirectionMajor
    MapROIFilter::get_major_direction(const std::vector<Polygon>& polygons)
    {
        double min_x =  range_, min_y =  range_;
        double max_x = -range_, max_y = -range_;

        for (auto& polygon: polygons)
        {
            for(auto& p: polygon)
            {
                double x = p.x();
                double y = p.y();
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }
        }

        min_x = std::max(min_x, -range_);
        max_x = std::min(max_x, range_);
        min_y = std::max(min_y, -range_);
        max_y = std::min(max_y, range_);
        return (max_x - min_x) < (max_y - min_y) ? Bitmap2D::DirectionMajor::XMAJOR
                                                 : Bitmap2D::DirectionMajor::YMAJOR;
    }

    void MapROIFilter::set_boundary(const std::vector<Polygon>& polygons)
    {
        auto major_dir = get_major_direction(polygons);
        bitmap_.SetUp(major_dir);
        DrawPolygonsMask<double>(polygons, &bitmap_, extend_dist_);
        // ROS_INFO_STREAM("Set map ROI boundary...");
    }
}
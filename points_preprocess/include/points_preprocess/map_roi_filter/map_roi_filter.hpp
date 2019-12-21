#pragma once

#include "points_preprocess/points_preprocess_type.hpp"
#include "bitmap_2d.hpp"
#include "polygon_mask.hpp"
#include <mutex>

namespace points_preprocess
{
    class MapROIFilter: public Filter
    {
    public:

        using Polygon = PolygonScanConverter<double>::Polygon;

        MapROIFilter(double range, double cell_size, double extend_dist);

        void process(const PointCloudT::ConstPtr& p,
                    const std::vector<int>& valid_indices,
                    std::vector<int>& inliers_indices) override;

        void set_boundary(const std::vector<Polygon>& polygons);

    private:

        Bitmap2D::DirectionMajor get_major_direction(const std::vector<Polygon>& polygons);

        double range_;
        double cell_size_;
        double extend_dist_;

        Bitmap2D bitmap_;
        std::mutex polygons_mt_;
    };
}
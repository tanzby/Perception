#pragma once

#include "points_preprocess/points_preprocess_type.hpp"
#include <vector>

namespace points_preprocess
{
    class GroundRemoval: public Filter {

        struct PointRI
        {
            int original_index;
            float radius;
        };
        typedef std::vector<PointRI> PointCloudRI;

        void convert_to_rtz(const PointCloudT::ConstPtr& in_cloud, const std::vector<int>& valid_indices,
                            PointCloudRI& out_organized_points,
                            std::vector<std::vector<int>>& out_radial_divided_indices,
                            std::vector<PointCloudRI>&     out_radial_ordered_clouds);

        void classify_point_cloud(const PointCloudT::ConstPtr& in_cloud,
                const std::vector<PointCloudRI>& in_radial_ordered_clouds,
                std::vector<int>& out_ground_indices,
                std::vector<int>& out_no_ground_indices);

        float radial_dividers_num_;
        float radial_divider_angle_;
        float concentric_divider_distance_;

        float local_max_slope_;
        float general_max_slope_;
        float min_height_threshold_;
        float reclass_distance_threshold_;

    public:

        struct GroundRemovalOption
        {
            float radial_divider_angle;
            float concentric_divider_distance;
            float local_max_slope;
            float general_max_slope;
            float min_height_threshold;
            float reclass_distance_threshold;
            GroundRemovalOption():
                radial_divider_angle(1.0),
                concentric_divider_distance(0.3),
                local_max_slope(10.0),
                general_max_slope(10.0),
                min_height_threshold(0.2),
                reclass_distance_threshold(0.5){}
        };

        GroundRemoval();

        void set_param(GroundRemovalOption& option);

        void process(const PointCloudT::ConstPtr &in_cloud, const std::vector<int> &valid_indices,
                     std::vector<int> &inliers_indices) override;
    };
}
#include "points_preprocess/ground_removal.hpp"

namespace points_preprocess
{

    void GroundRemoval::convert_to_rtz(const PointCloudT::ConstPtr& in_cloud, const std::vector<int>& valid_indices,
                        PointCloudRI& out_organized_points,
                        std::vector<std::vector<int>>& out_radial_divided_indices,
                        std::vector<PointCloudRI>& out_radial_ordered_clouds)
    {
        out_organized_points.resize(valid_indices.size());
        out_radial_divided_indices.clear();
        out_radial_divided_indices.resize(radial_dividers_num_);
        out_radial_ordered_clouds.resize(radial_dividers_num_);

        for (size_t i = 0; i < valid_indices.size(); i++)
        {
            auto& in_p = in_cloud->points[valid_indices[i]];
            auto  radius = static_cast<float>(std::sqrt(in_p.x * in_p.x + in_p.y * in_p.y));
            auto  theta  = static_cast<float>(std::atan2(in_p.y, in_p.x) * 180 / M_PI);

            if (theta < 0)     theta += 360;
            if (theta >= 360)  theta -= 360;

            auto radial_div = (int)std::floor(theta / radial_divider_angle_);

            PointRI new_point{valid_indices[i], radius};

            out_organized_points.at(i) = new_point;

            // radial divisions
            out_radial_divided_indices.at(radial_div).emplace_back(i);
            out_radial_ordered_clouds.at(radial_div).emplace_back(new_point);
        }

        // order radial points on each division
        #pragma omp for
        for (size_t i = 0; i < radial_dividers_num_; i++)
        {
            std::sort(out_radial_ordered_clouds.at(i).begin(), out_radial_ordered_clouds.at(i).end(),
                      [](const PointRI& a, const PointRI& b) { return a.radius < b.radius; });  // NOLINT
        }

    }

    void GroundRemoval::classify_point_cloud(const PointCloudT::ConstPtr& in_cloud,
            const std::vector<PointCloudRI>& in_radial_ordered_clouds,
            std::vector<int>& out_ground_indices,
            std::vector<int>& out_no_ground_indices)
    {
        out_ground_indices.clear();
        out_no_ground_indices.clear();

        #pragma omp for
        for (const auto & in_radial_ordered_cloud : in_radial_ordered_clouds)  // sweep through each radial division
        {
            float prev_radius = 0.f;
            float prev_height = 0.f;
            bool prev_ground = false;
            bool current_ground = false;
            for (auto cur_p : in_radial_ordered_cloud)  // loop through each point in the radial div
            {
                float points_distance = cur_p.radius - prev_radius;
                float height_threshold = std::tan(DEG2RAD(local_max_slope_)) * points_distance;
                float current_height = in_cloud->points[cur_p.original_index].z;
                float general_height_threshold = std::tan(DEG2RAD(general_max_slope_)) * cur_p.radius;

                // for points which are very close causing the height threshold to be tiny, set a minimum value
                if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
                {
                    height_threshold = min_height_threshold_;
                }

                // check current point height against the LOCAL threshold (previous point)
                if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
                {
                    // Check again using general geometry (radius from origin) if previous points wasn't ground
                    if (!prev_ground)
                    {
                        current_ground = current_height <= general_height_threshold &&
                                         current_height >= -general_height_threshold;
                    }
                    else
                    {
                        current_ground = true;
                    }
                }
                else
                {
                    // check if previous point is too far from previous one, if so classify again
                    current_ground = points_distance > reclass_distance_threshold_ &&
                                     (current_height <= height_threshold && current_height >= -height_threshold);
                }

                if (current_ground)
                {
                    out_ground_indices.emplace_back(cur_p.original_index);
                    prev_ground = true;
                }
                else
                {
                    out_no_ground_indices.emplace_back(cur_p.original_index);
                    prev_ground = false;
                }

                prev_radius = cur_p.radius;
                prev_height = current_height;
            }
        }
    }

    void
    GroundRemoval::process(const PointCloudT::ConstPtr &in_cloud,
            const std::vector<int> &valid_indices,
            std::vector<int> &inliers_indices)
    {
        if (in_cloud->empty() || valid_indices.empty()) return;
    
        // ray filter
        PointCloudRI  out_organized_points;
        std::vector<std::vector<int>>  radial_division_indices;
        std::vector<PointCloudRI>     radial_ordered_clouds;
        std::vector<int> ground_indices, no_ground_indices;

        convert_to_rtz(in_cloud, valid_indices, out_organized_points,
                       radial_division_indices, radial_ordered_clouds);
        classify_point_cloud(in_cloud, radial_ordered_clouds, ground_indices, no_ground_indices);

        inliers_indices.swap(no_ground_indices);
    }

    GroundRemoval::GroundRemoval() {
        auto temp_opt = GroundRemovalOption();
        set_param(temp_opt);
    }

    void GroundRemoval::set_param(GroundRemovalOption& option)
    {
        this->radial_divider_angle_ = option.radial_divider_angle;
        this->concentric_divider_distance_ = option.concentric_divider_distance;
        this->local_max_slope_ = option.local_max_slope;
        this->general_max_slope_ = option.general_max_slope;
        this->min_height_threshold_ = option.min_height_threshold;
        this->reclass_distance_threshold_ = option.reclass_distance_threshold;

        radial_dividers_num_ = std::ceil(360 / radial_divider_angle_);
    }
}
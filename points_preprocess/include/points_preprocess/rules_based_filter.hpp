#pragma once

#include "points_preprocess/points_preprocess_type.hpp"

namespace points_preprocess
{
    class RulesBasedFilter: public Filter
    {
    public:

        struct Rules
        {
            float in_max_x_range;
            float in_min_x_range;
            float in_max_y_range;
            float in_min_y_range;
            float out_max_x_range;
            float out_min_x_range;
            float out_max_y_range;
            float out_min_y_range;
            float clip_height;
            Rules():in_max_x_range(3.0),
                in_min_x_range(-1.2),
                in_max_y_range(1.5),
                in_min_y_range(-1.5),
                out_max_x_range(90),
                out_min_x_range(-20),
                out_max_y_range(20),
                out_min_y_range(-20),
                    clip_height(1.5){}
        };

        RulesBasedFilter();

        void set_param(Rules& rules);

        void process(const PointCloudT::ConstPtr &in_cloud, const std::vector<int> &valid_indices,
                     std::vector<int> &inliers_indices) override;

    private:

        Rules rules_;
    };

}
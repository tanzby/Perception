#include "points_preprocess/rules_based_filter.hpp"

namespace points_preprocess
{
    void RulesBasedFilter::process(const PointCloudT::ConstPtr &in_cloud, const std::vector<int> &valid_indices,
                                   std::vector<int> &inliers_indices) {

        std::vector<int> ret; ret.reserve(valid_indices.size());
        #pragma omp for
        for (int i = 0; i < valid_indices.size(); ++i)
        {
            auto& p = in_cloud->points[valid_indices[i]];

            if (p.z > rules_.clip_height) continue;

            if (p.x > rules_.out_max_x_range) continue;
            if (p.y > rules_.out_max_y_range) continue;
            if (p.x < rules_.out_min_x_range) continue;
            if (p.y < rules_.out_min_y_range) continue;

            if (p.x < rules_.in_max_x_range && p.y < rules_.in_max_y_range &&
                p.x > rules_.in_min_x_range && p.y > rules_.in_min_y_range) continue;

            ret.emplace_back(valid_indices[i]);
        }
        ret.swap(inliers_indices);
    }

    RulesBasedFilter::RulesBasedFilter() {
        Rules rules;
        set_param(rules);
    }

    void RulesBasedFilter::set_param(Rules & rules)
    {
        rules_ = rules;
    }
}
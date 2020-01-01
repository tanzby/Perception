#include "lidar_object_detector/box_shape_estimation.hpp"


namespace lidar_object_detector
{
    BoxShapeEstimator::BoxShapeEstimator(){}

    bool BoxShapeEstimator::estimate(const Cluster::Ptr& cluster, perception_msgs::DetectedObject& box)
    {

        // calc min and max z for cylinder length
        double min_z = cluster->cloud->points.front().z;
        double max_z = cluster->cloud->points.front().z;
        for (auto& p: cluster->cloud->points)
        {
            if (p.z < min_z) min_z = p.z;
            if (p.z > max_z) max_z = p.z;
        }

        /*
         * Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners
         * Authors : Xio Zhang, Wenda Xu, Chiyu Dong and John M. Dolan
         */

        // Paper : Algo.2 Search-Based Rectangle Fitting
        std::vector<std::pair<double /*theta*/, double /*q*/>> Q;
        const double max_angle = M_PI / 2.0;
        const double angle_reso = M_PI / 180.0;
        for (double theta = 0; theta < max_angle; theta += angle_reso)
        {
            Eigen::Vector2d e_1;
            e_1 << std::cos(theta), std::sin(theta);   // col.3, Algo.2
            Eigen::Vector2d e_2;
            e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
            std::vector<double> C_1;                   // col.5, Algo.2
            std::vector<double> C_2;                   // col.6, Algo.2
            for (const auto& point: cluster->cloud->points)
            {
                C_1.emplace_back(point.x * e_1.x() + point.y * e_1.y());
                C_2.emplace_back(point.x * e_2.x() + point.y * e_2.y());
            }
            double q = calculate_closeness(C_1, C_2);  // col.7, Algo.2
            Q.emplace_back(theta, q);                  // col.8, Algo.2
        }

        double theta_star;  // col.10, Algo.2
        double max_q;
        for (size_t i = 0; i < Q.size(); ++i)
        {
            if (max_q < Q.at(i).second || i == 0)
            {
                max_q = Q.at(i).second;
                theta_star = Q.at(i).first;
            }
        }

        Eigen::Vector2d e_1_star;  // col.11, Algo.2
        Eigen::Vector2d e_2_star;
        e_1_star << std::cos(theta_star), std::sin(theta_star);
        e_2_star << -std::sin(theta_star), std::cos(theta_star);
        std::vector<double> C_1_star;  // col.11, Algo.2
        std::vector<double> C_2_star;  // col.11, Algo.2
        for (const auto& point: cluster->cloud->points)
        {
            C_1_star.emplace_back(point.x * e_1_star.x() + point.y * e_1_star.y());
            C_2_star.emplace_back(point.x * e_2_star.x() + point.y * e_2_star.y());
        }

        // col.12, Algo.2
        const double min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
        const double max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
        const double min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
        const double max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

        const double a_1 = std::cos(theta_star);
        const double b_1 = std::sin(theta_star);
        const double c_1 = min_C_1_star;
        const double a_2 = -1.0 * std::sin(theta_star);
        const double b_2 = std::cos(theta_star);
        const double c_2 = min_C_2_star;
        const double a_3 = std::cos(theta_star);
        const double b_3 = std::sin(theta_star);
        const double c_3 = max_C_1_star;
        const double a_4 = -1.0 * std::sin(theta_star);
        const double b_4 = std::cos(theta_star);
        const double c_4 = max_C_2_star;

        // calc center of bounding box
        double intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
        double intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
        double intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
        double intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

        // calc dimention of bounding box
        Eigen::Vector2d e_x;
        Eigen::Vector2d e_y;
        e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
        e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
        Eigen::Vector2d diagonal_vec;
        diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

        // calc yaw
        box.pose.orientation = getOrientFromYaw(std::atan2(e_1_star.y(), e_1_star.x()));

        box.pose.position.x = (intersection_x_1 + intersection_x_2) / 2.0;
        box.pose.position.y = (intersection_y_1 + intersection_y_2) / 2.0;
        box.pose.position.z = cluster->centroid.z();
        
        constexpr double ep = 0.001;
        box.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
        box.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
        box.dimensions.z = std::max((max_z - min_z), ep);

        // check wrong output
        if (box.dimensions.x < ep && box.dimensions.y < ep)
            return false;
        box.dimensions.x = std::max(box.dimensions.x, ep);
        box.dimensions.y = std::max(box.dimensions.y, ep);
        return true;
    }

    double
    BoxShapeEstimator::calculate_closeness(
            const std::vector<double>& C_1, const std::vector<double>& C_2)
    {
        // Paper : Algo.4 Closeness Criterion
        const double min_c_1 = *std::min_element(C_1.begin(), C_1.end());  // col.2, Algo.4
        const double max_c_1 = *std::max_element(C_1.begin(), C_1.end());  // col.2, Algo.4
        const double min_c_2 = *std::min_element(C_2.begin(), C_2.end());  // col.3, Algo.4
        const double max_c_2 = *std::max_element(C_2.begin(), C_2.end());  // col.3, Algo.4

        std::vector<double> D_1;  // col.4, Algo.4
        for (const auto& c_1_element : C_1)
        {
            const double v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
            D_1.push_back(std::fabs(v));
        }

        std::vector<double> D_2;  // col.5, Algo.4
        for (const auto& c_2_element : C_2)
        {
            const double v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
            D_2.push_back(v * v);
        }

        const double d_min = 0.05;
        const double d_max = 0.50;
        double beta = 0;  // col.6, Algo.4
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const double d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            beta += 1.0 / d;
        }
        return beta;
    }

}
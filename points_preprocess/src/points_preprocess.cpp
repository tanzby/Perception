#include <numeric>

#include <pcl/filters/approximate_voxel_grid.h>
#include "points_preprocess/points_preprocess.hpp"

namespace points_preprocess
{

    void PointsPreProcessor::filter(const PointCloudT::ConstPtr &in_cloud, PointCloudT &out_cloud)
    {
        std::vector<int> inliers_indices(in_cloud->size());
        std::iota(inliers_indices.begin(), inliers_indices.end(), 0);

        if (map_roi_filter_)
        {
            construct_local_boundary();
            map_roi_filter_->process(in_cloud, inliers_indices, inliers_indices);
        }
        
        if (rules_based_filter_)
        {
            rules_based_filter_->process(in_cloud, inliers_indices, inliers_indices);
        }

        if (ground_removal_)
        {
            ground_removal_->process(in_cloud, inliers_indices, inliers_indices);
        }

        PointCloudT(*in_cloud, inliers_indices).swap(out_cloud);
        out_cloud.header = in_cloud->header;
    }

    void PointsPreProcessor::cloud_callback(const PointCloudT::ConstPtr &msg)
    {
        if (out_cloud_publisher_.getNumSubscribers() == 0) return;
        if (!map_roi_filter_ && !ground_removal_ && !rules_based_filter_)
        {
            ROS_ERROR_STREAM("There is not any filter applied.");
            out_cloud_publisher_.publish(msg);
        }
        else
        {
            PointCloudT out_pointcloud;

            // remove Nan
            auto prefiltered_cloud = PointCloudT().makeShared();
            std::vector<int> index;
            pcl::removeNaNFromPointCloud(*msg, *prefiltered_cloud,index);

            // down sample
            pcl::ApproximateVoxelGrid<PointT> voxel_filter;
            voxel_filter.setInputCloud(prefiltered_cloud);
            voxel_filter.setLeafSize(0.1f,0.1f,0.1f);
            voxel_filter.filter(*prefiltered_cloud);

            this->filter(prefiltered_cloud, out_pointcloud);
            if (!out_pointcloud.empty()) {
                out_cloud_publisher_.publish(out_pointcloud);
            }
        }
    }

    void PointsPreProcessor::construct_local_boundary()
    {
        assert(map_roi_filter_ != nullptr);

        try
        {
            tf::StampedTransform  transform;
            tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
            double tx  = transform.getOrigin().getX();
            double ty  = transform.getOrigin().getY();
            auto quad  = transform.getRotation();
            Eigen::Quaterniond r(quad.w(),quad.x(),quad.y(),quad.z());
            Eigen::Matrix2d rot = r.toRotationMatrix().topLeftCorner(2,2);

            auto& world_polygon = latest_map_roi_concave_.polygon.points;

            std::vector<MapROIFilter::Polygon>  boundary_polygons(1, MapROIFilter::Polygon(world_polygon.size()));

            for (int i = 0; i < world_polygon.size(); ++i)
            {
                boundary_polygons[0][i].x() =  world_polygon[i].x - tx;
                boundary_polygons[0][i].y() =  world_polygon[i].y - ty;
                boundary_polygons[0][i] = rot.transpose()*boundary_polygons[0][i];
            }

            map_roi_filter_->set_boundary(boundary_polygons);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }

    void PointsPreProcessor::map_roi_callback(const geometry_msgs::PolygonStamped::ConstPtr &msg)
    {
        latest_map_roi_concave_ = *msg;
    }

    void PointsPreProcessor::init()
    {
        ////////////////////////////////////////    data api    /////////////////////////////////////////////

        in_cloud_subscriber_ = node_handle_.subscribe("/concat_rslidar_points", 1, &PointsPreProcessor::cloud_callback, this);
        out_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/filtered_rslidar_points", 1, false);

        //////////////////////////////////////// map_roi_filter /////////////////////////////////////////////

        bool use_map_roi_filter;
        private_node_handle_.param<bool>("use_map_roi_filter", use_map_roi_filter, false);
        ROS_WARN_STREAM("[PreProcess] use_map_roi_filter: "<< (use_map_roi_filter?"true":"false"));

        if (use_map_roi_filter)
        {
            double range, cell_size, extend_dist;
            private_node_handle_.param<double>("range", range, 100.0);
            ROS_INFO_STREAM("[MapROIFilter] range: "<< range);
            private_node_handle_.param<double>("extend_dist", extend_dist, 2.0);
            ROS_INFO_STREAM("[MapROIFilter] extend_dist: "<< extend_dist);
            private_node_handle_.param<double>("cell_size", cell_size, 0.5);
            ROS_INFO_STREAM("[MapROIFilter] cell_size: "<< cell_size);

            map_roi_filter_ = std::make_unique<MapROIFilter>(range, cell_size, extend_dist);
            map_roi_subscriber_ = node_handle_.subscribe("/map_roi_region", 1, &PointsPreProcessor::map_roi_callback, this);
        }

        ////////////////////////////////////// rules_base_filter ///////////////////////////////////////////

        bool use_rules_base_filter;
        private_node_handle_.param<bool>("use_rules_base_filter", use_rules_base_filter, true);
        ROS_WARN_STREAM("[PreProcess] use_rules_base_filter: "<< (use_rules_base_filter?"true":"false"));

        if (use_rules_base_filter)
        {
            RulesBasedFilter::Rules rules;
            private_node_handle_.param<float>("in_max_x_range", rules.in_max_x_range, rules.in_max_x_range);
            ROS_INFO_STREAM("[RulesBasedFilter] in_max_x_range: "<< rules.in_max_x_range);
            private_node_handle_.param<float>("in_max_y_range", rules.in_max_y_range, rules.in_max_y_range);
            ROS_INFO_STREAM("[RulesBasedFilter] in_max_y_range: "<< rules.in_max_y_range);
            private_node_handle_.param<float>("in_min_x_range", rules.in_min_x_range, rules.in_min_x_range);
            ROS_INFO_STREAM("[RulesBasedFilter] in_min_x_range: "<< rules.in_min_x_range);
            private_node_handle_.param<float>("in_min_y_range", rules.in_min_y_range, rules.in_min_y_range);
            ROS_INFO_STREAM("[RulesBasedFilter] in_min_y_range: "<< rules.in_min_y_range);
            private_node_handle_.param<float>("out_max_x_range", rules.out_max_x_range, rules.out_max_x_range);
            ROS_INFO_STREAM("[RulesBasedFilter] out_max_x_range: "<< rules.out_max_x_range);
            private_node_handle_.param<float>("out_max_y_range", rules.out_max_y_range, rules.out_max_y_range);
            ROS_INFO_STREAM("[RulesBasedFilter] out_max_y_range: "<< rules.out_max_y_range);
            private_node_handle_.param<float>("out_min_x_range", rules.out_min_x_range, rules.out_min_x_range);
            ROS_INFO_STREAM("[RulesBasedFilter] out_min_x_range: "<< rules.out_min_x_range);
            private_node_handle_.param<float>("out_min_y_range", rules.out_min_y_range, rules.out_min_y_range);
            ROS_INFO_STREAM("[RulesBasedFilter] out_min_y_range: "<< rules.out_min_y_range);
            private_node_handle_.param<float>("clip_height", rules.clip_height, rules.clip_height);
            ROS_INFO_STREAM("[RulesBasedFilter] clip_height: "<< rules.clip_height);

            rules_based_filter_ = std::make_unique<RulesBasedFilter>();
            rules_based_filter_->set_param(rules);
        }

        //////////////////////////////////////// ground_removal /////////////////////////////////////////////
        bool use_ground_removal;
        private_node_handle_.param<bool>("use_ground_removal", use_ground_removal, true);
        ROS_WARN_STREAM("[PreProcess] use_ground_removal: "<< (use_ground_removal?"true":"false"));

        if (use_ground_removal)
        {
            GroundRemoval::GroundRemovalOption option;

            private_node_handle_.param<float>("radial_divider_angle", option.radial_divider_angle, option.radial_divider_angle);
            ROS_INFO_STREAM("[GroundRemoval] radial_divider_angle: "<< option.radial_divider_angle);
            private_node_handle_.param<float>("concentric_divider_distance", option.concentric_divider_distance, option.concentric_divider_distance);
            ROS_INFO_STREAM("[GroundRemoval] concentric_divider_distance: "<< option.concentric_divider_distance);
            private_node_handle_.param<float>("local_max_slope", option.local_max_slope, option.concentric_divider_distance);
            ROS_INFO_STREAM("[GroundRemoval] local_max_slope: "<< option.local_max_slope);
            private_node_handle_.param<float>("general_max_slope", option.general_max_slope, option.general_max_slope);
            ROS_INFO_STREAM("[GroundRemoval] general_max_slope: "<< option.general_max_slope);
            private_node_handle_.param<float>("min_height_threshold", option.min_height_threshold, option.min_height_threshold);
            ROS_INFO_STREAM("[GroundRemoval] min_height_threshold: "<< option.min_height_threshold);
            private_node_handle_.param<float>("reclass_distance_threshold", option.reclass_distance_threshold, option.reclass_distance_threshold);
            ROS_INFO_STREAM("[GroundRemoval] reclass_distance_threshold: "<< option.reclass_distance_threshold);

            ground_removal_ = std::make_unique<GroundRemoval>();
            ground_removal_->set_param(option);
        }

    }

    PointsPreProcessor::PointsPreProcessor()
    :node_handle_(), private_node_handle_("~"), tf_listener_()
    {
        init();
    }
}
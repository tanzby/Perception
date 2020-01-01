#pragma once
#include "lidar_object_detector/ukf.hpp"
#include <tf/transform_listener.h>
#include <vector>

namespace lidar_object_detector
{
    class ImmUkfPda
    {
    private:
        int target_id_;
        bool init_;
        double timestamp_;

        std::vector<UKF> targets_;

        // probabilistic data association params
        double gating_threshold_;
        double gate_probability_;
        double detection_probability_;

        // object association param
        int life_time_threshold_;

        // static classification param
        double static_velocity_threshold_;
        int static_num_history_threshold_;

        // switch sukf and ImmUkfPda
        bool use_sukf_;

        // prevent explode param for ukf
        double prevent_explosion_threshold_;

        double merge_distance_threshold_;
        const double CENTROID_DISTANCE = 0.2;//distance to consider centroids the same

        std::string input_topic_;
        std::string output_topic_;

        std::string tracking_frame_;

        tf::TransformListener tf_listener_;
        tf::StampedTransform local2global_;

        ros::NodeHandle node_handle_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_detected_array_;
        ros::Publisher pub_object_array_;

        std_msgs::Header input_header_;

        void callback(const perception_msgs::DetectedObjectArray& input);

        void transformPoseToGlobal(const perception_msgs::DetectedObjectArray& input,
                                   perception_msgs::DetectedObjectArray& transformed_input);
        void transformPoseToLocal(perception_msgs::DetectedObjectArray& detected_objects_output);

        geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose,
                                               const tf::StampedTransform& tf_stamp);

        bool updateNecessaryTransform();

        void measurementValidation(const perception_msgs::DetectedObjectArray& input, UKF& target, const bool second_init,
                                   const Eigen::VectorXd& max_det_z, const Eigen::MatrixXd& max_det_s,
                                   std::vector<perception_msgs::DetectedObject>& object_vec, std::vector<bool>& matching_vec);

        void updateBehaviorState(const UKF& target, const bool use_sukf, perception_msgs::DetectedObject& object);

        void initTracker(const perception_msgs::DetectedObjectArray& input, double timestamp);
        void secondInit(UKF& target, const std::vector<perception_msgs::DetectedObject>& object_vec, double dt);

        void updateTrackingNum(const std::vector<perception_msgs::DetectedObject>& object_vec, UKF& target);

        bool probabilisticDataAssociation(const perception_msgs::DetectedObjectArray& input, const double dt,
                                          std::vector<bool>& matching_vec,
                                          std::vector<perception_msgs::DetectedObject>& object_vec, UKF& target);
        void makeNewTargets(const double timestamp, const perception_msgs::DetectedObjectArray& input,
                            const std::vector<bool>& matching_vec);

        void staticClassification();

        void makeOutput(const perception_msgs::DetectedObjectArray& input,
                        const std::vector<bool>& matching_vec,
                        perception_msgs::DetectedObjectArray& detected_objects_output);

        void removeUnnecessaryTarget();

        void tracker(const perception_msgs::DetectedObjectArray& transformed_input,
                     perception_msgs::DetectedObjectArray& detected_objects_output);

        perception_msgs::DetectedObjectArray
        removeRedundantObjects(const perception_msgs::DetectedObjectArray& in_detected_objects,
                               const std::vector<size_t> in_tracker_indices);

        bool
        arePointsClose(const geometry_msgs::Point& in_point_a,
                       const geometry_msgs::Point& in_point_b,
                       float in_radius);

        bool
        arePointsEqual(const geometry_msgs::Point& in_point_a,
                       const geometry_msgs::Point& in_point_b);

        bool
        isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                      const geometry_msgs::Point& in_point);

        void updateTargetWithAssociatedObject(const std::vector<perception_msgs::DetectedObject>& object_vec,
                                              UKF& target);

    public:
        ImmUkfPda();
        void run();
    };

}

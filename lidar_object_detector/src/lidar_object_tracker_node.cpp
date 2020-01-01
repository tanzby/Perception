#include "lidar_object_detector/imm_ukf_pda.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_object_tracker_node");

    lidar_object_detector::ImmUkfPda app;

    ROS_WARN_STREAM("Ready");
    ros::spin();
}
#include "lidar_object_detector/lidar_object_detector.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_object_detector_node");

    lidar_object_detector::LidarObjectDetector app;

    ROS_WARN_STREAM("Ready");
    ros::spin();
}
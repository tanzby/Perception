#include "points_preprocess/points_preprocess.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_preprocess_node");
    points_preprocess::PointsPreProcessor processor;

    ROS_WARN_STREAM("Ready");
    ros::spin();
}
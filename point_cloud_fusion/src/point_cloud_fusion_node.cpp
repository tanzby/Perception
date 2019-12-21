#include "point_cloud_fusion/point_cloud_fusion.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_fusion_node");
    ros::NodeHandle nh("~");

    /// read parameters
    std::string output_frame_id;
    std::string output_topic;
    std::vector<std::string> input_topics;
    nh.param("output_frame_id", output_frame_id, std::string("base_link"));
    nh.getParam("input_topics", input_topics);
    nh.param<std::string>("output_topic", output_topic, std::string("/concat_points"));

    point_cloud_fusion::PointCloudFusion app(output_frame_id, output_topic, input_topics);

    ROS_WARN_STREAM("Ready");
    ros::spin();
    return 0;
}
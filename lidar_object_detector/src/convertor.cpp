#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <perception_msgs/DetectedObjectArray.h>
#include "lidar_object_detector/typedef.hpp"
#include "lidar_object_detector/ukf.hpp"

ros::Subscriber detected_object_subscriber;
ros::Publisher  jsk_object_publisher;
ros::Publisher  jsk_polygon_publisher;
ros::Publisher  info_msg_publisher;

void detected_object_callback(const perception_msgs::DetectedObjectArrayConstPtr& msgs)
{
    jsk_recognition_msgs::BoundingBoxArray  object_array;
    jsk_recognition_msgs::PolygonArray  polygon_array;
    visualization_msgs::MarkerArray marker_array;

    int count_id = 0;
    for(const auto& object: msgs->objects)
    {
        jsk_recognition_msgs::BoundingBox box;

        box.header = msgs->header;
        box.pose = object.pose;
        box.dimensions = object.dimensions;
        box.label = object.id;
        object_array.boxes.emplace_back(box);

        // fill information
        visualization_msgs::Marker arrow;
        arrow.id = count_id++;
        arrow.header = msgs->header;
        arrow.pose = object.pose;
        arrow.pose.position.z = object.pose.position.z + object.dimensions.z;
        arrow.ns = "arrow";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.lifetime = ros::Duration(0.1);
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.scale.x = 2.0;arrow.scale.y = 0.2;arrow.scale.z = 0.2;
        arrow.color.a = 1.0;arrow.color.r = 1.0;arrow.color.g = 1.0;arrow.color.b = 1.0;

        visualization_msgs::Marker text_info;
        text_info.id = count_id++;
        text_info.header = msgs->header;
        text_info.pose = object.pose;
        text_info.pose.position.z = object.pose.position.z + object.dimensions.z;
        text_info.ns = "text_info";
        text_info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_info.lifetime = ros::Duration(0.1);
        text_info.action = visualization_msgs::Marker::ADD;
        text_info.scale.z = 1.0;
        text_info.color.a = 1.0;text_info.color.r = 1.0;text_info.color.g = 1.0;text_info.color.b = 1.0;

        std::string motion_model_str ;
        switch (object.behavior_state)
        {
            case lidar_object_detector::MotionModel::CTRV: motion_model_str="CTRV"; break;
            case lidar_object_detector::MotionModel::RM: motion_model_str="RM"; break;
            case lidar_object_detector::MotionModel::CV: motion_model_str="CV"; break;
        }
        char _buf[128];
        sprintf(_buf,"Object[%d] Model: %s \n (%3.2f, %3.2f, %3.1f)",
                object.id, motion_model_str.c_str(), object.pose.position.x, object.pose.position.x,
                lidar_object_detector::getYawFromOrient(object.pose.orientation) * 180.0/M_PI);
        text_info.text = _buf;

        polygon_array.labels.emplace_back(object.id);
        polygon_array.polygons.emplace_back(object.convex_hull);
        marker_array.markers.emplace_back(text_info);
        marker_array.markers.emplace_back(arrow);
    }
    object_array.header = msgs->header;
    polygon_array.header = msgs->header;
    jsk_object_publisher.publish(object_array);
    jsk_polygon_publisher.publish(polygon_array);
    info_msg_publisher.publish(marker_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_object_converter_node");
    ros::NodeHandle private_nh("~"), nh;

    detected_object_subscriber = private_nh.subscribe("/tracked_detected_objects", 1,  detected_object_callback);
    jsk_polygon_publisher = nh.advertise<jsk_recognition_msgs::PolygonArray>("/detected_object_convex", 1);
    jsk_object_publisher  = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/visual_detected_object", 1);
    info_msg_publisher = nh.advertise<visualization_msgs::MarkerArray>("/visual_detected_info", 1);
    ROS_WARN_STREAM("Ready");

    ros::spin();
    ros::shutdown();
    return 0;
}
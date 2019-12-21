#include <utility>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


namespace point_cloud_fusion
{
    class PointCloudFusion
    {
    public:

        PointCloudFusion(std::string output_frame_id, std::string output_topic, std::vector<std::string> input_topics);

    private:
        typedef pcl::PointXYZI PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef sensor_msgs::PointCloud2 PointCloudMsgT;
        typedef message_filters::sync_policies::ApproximateTime<PointCloudT, PointCloudT, PointCloudT,
                                                                PointCloudT, PointCloudT, PointCloudT,
                                                                PointCloudT, PointCloudT>
                                                                SyncPolicyT;

        ros::NodeHandle node_handle_, private_node_handle_;
        std::unique_ptr<message_filters::Subscriber<PointCloudT>> cloud_subscribers_[8];
        std::unique_ptr<message_filters::Synchronizer<SyncPolicyT>>  cloud_synchronizer_;
        ros::Publisher cloud_publisher_;
        tf::TransformListener tf_listener_;

        std::vector<std::string> input_topics_;
        std::string output_frame_id_;

        void pointcloud_callback(const PointCloudT::ConstPtr &msg1, const PointCloudT::ConstPtr &msg2,
                                 const PointCloudT::ConstPtr &msg3, const PointCloudT::ConstPtr &msg4,
                                 const PointCloudT::ConstPtr &msg5, const PointCloudT::ConstPtr &msg6,
                                 const PointCloudT::ConstPtr &msg7, const PointCloudT::ConstPtr &msg8);
    };
}



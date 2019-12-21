#include "point_cloud_fusion/point_cloud_fusion.hpp"

namespace point_cloud_fusion
{
    PointCloudFusion::PointCloudFusion(std::string output_frame_id, std::string output_topic, std::vector<std::string> input_topics):
            node_handle_(), private_node_handle_("~"), tf_listener_(), output_frame_id_(output_frame_id), input_topics_(input_topics)
    {
        if (input_topics_.size() < 2 || 8 < input_topics_.size())
        {
            ROS_ERROR("The size of input_topics must be between 2 and 8");
            ros::shutdown();
        }
        for (size_t i = 0; i < 8; ++i)
        {
            if (i < input_topics_.size())
            {
                cloud_subscribers_[i] =
                        std::make_unique<message_filters::Subscriber<PointCloudT>>(node_handle_, input_topics_[i], 1);
            }
            else
            {
                cloud_subscribers_[i] =
                        std::make_unique<message_filters::Subscriber<PointCloudT>>(node_handle_, input_topics_[0], 1);
            }
        }
        cloud_synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
                SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
                *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);

        cloud_synchronizer_->registerCallback(
                boost::bind(&PointCloudFusion::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>(output_topic, 1);
    }

    void PointCloudFusion::pointcloud_callback(const PointCloudT::ConstPtr &msg1, const PointCloudT::ConstPtr &msg2,
                                               const PointCloudT::ConstPtr &msg3, const PointCloudT::ConstPtr &msg4,
                                               const PointCloudT::ConstPtr &msg5, const PointCloudT::ConstPtr &msg6,
                                               const PointCloudT::ConstPtr &msg7, const PointCloudT::ConstPtr &msg8)
    {
        assert(2 <= input_topics_.size() && input_topics_.size() <= 8);

        PointCloudT::ConstPtr msgs[8] = { msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8 };
        PointCloudT::Ptr cloud_sources[8];
        PointCloudT::Ptr cloud_concatenated(new PointCloudT);

        // transform points
        try
        {
            for (size_t i = 0; i < input_topics_.size(); ++i)
            {
                cloud_sources[i] = PointCloudT().makeShared();
                pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *msgs[i], msgs[i]->header.frame_id,
                                             *cloud_sources[i], tf_listener_);
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // merge points
        for (size_t i = 0; i < input_topics_.size(); ++i)
        {
            *cloud_concatenated += *cloud_sources[i];
        }

        // publish points
        cloud_concatenated->header = msgs[0]->header;
        cloud_concatenated->header.frame_id = output_frame_id_;
        cloud_publisher_.publish(cloud_concatenated);
    }
}

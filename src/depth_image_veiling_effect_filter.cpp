
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"

namespace depth_image_veiling_effect_filter
{

DepthImageVeilingEffectFilter::DepthImageVeilingEffectFilter(double threshold)
{
    m_threshold = threshold;
}

sensor_msgs::ImagePtr DepthImageVeilingEffectFilter::process(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug)
{
    // todo: check debug
    return debug; // todo
}

void DepthImageVeilingEffectFilter::setThreshold(double threshold)
{
    m_threshold = threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

DepthImageVeilingEffectFilterNode::DepthImageVeilingEffectFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it(pnh), dynrec_server(pnh)
{
    pub = it.advertiseCamera("/camera/aligned_depth_to_color_filtered/image_raw", 1);
    pub_debug = it.advertiseCamera("/camera/aligned_depth_to_color_filtered/image_raw_debug", 1);
    sub = it.subscribeCamera("/camera/aligned_depth_to_color/image_raw", 1, &DepthImageVeilingEffectFilterNode::imageCallback, this);
    dynrec_server.setCallback(boost::bind(&DepthImageVeilingEffectFilterNode::reconfigureCallback, this, _1, _2));

    while (ros::ok())
    {
        ros::spin();
    }
}

void DepthImageVeilingEffectFilterNode::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info)
{
    ROS_INFO_ONCE("First image arrived to DepthImageVeilingEffectFilter");

    sensor_msgs::ImagePtr filtered_msg;

    if (pub_debug.getNumSubscribers() > 0)
    {
        ROS_WARN_THROTTLE(1.0, "Publishing debug images may increase CPU usage");
        sensor_msgs::ImagePtr debug_msg = boost::make_shared<sensor_msgs::Image>();
        filtered_msg = filter.process(msg, info, debug_msg);

        pub.publish(msg, info);
        pub_debug.publish(msg, info); // TODO: pub_debug.publish(debug_msg, info);
    }
    else
    {
        sensor_msgs::ImagePtr filtered_msg = filter.process(msg, info);
        pub.publish(msg, info);
    }
}

void DepthImageVeilingEffectFilterNode::reconfigureCallback(DepthImageVeilingEffectFilterConfig &config, uint32_t level)
{
    ROS_INFO("DepthImageVeilingEffectFilter threshold changed to %f", config.threshold);
    filter.setThreshold(config.threshold);
}


} // namespace depth_image_veiling_effect_filter
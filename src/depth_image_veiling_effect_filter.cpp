
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"

namespace depth_image_veiling_effect_filter
{

DepthImageVeilingEffectFilter::DepthImageVeilingEffectFilter()
{

}

void DepthImageVeilingEffectFilter::process()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////

DepthImageVeilingEffectFilterNode::DepthImageVeilingEffectFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it(pnh), dynrec_server(pnh)
{
    sub = it.subscribe("/test_data/depth", 2, &DepthImageVeilingEffectFilterNode::imageCallback, this);
    dynrec_server.setCallback(boost::bind(&DepthImageVeilingEffectFilterNode::reconfigureCallback, this, _1, _2));

    while (ros::ok())
    {
        ros::spin();
    }
}


void DepthImageVeilingEffectFilterNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("imageCallback");
}

void DepthImageVeilingEffectFilterNode::reconfigureCallback(DepthImageVeilingEffectFilterConfig &config, uint32_t level)
{
    ROS_INFO("reconfigureCallback %f", config.threshold);
}


} // namespace depth_image_veiling_effect_filter
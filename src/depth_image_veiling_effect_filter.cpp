
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

DepthImageVeilingEffectFilterNode::DepthImageVeilingEffectFilterNode(ros::NodeHandle &nhp) : it(nhp)
{
    it.subscribe("in_image_base_topic", 2, &DepthImageVeilingEffectFilterNode::imageCallback, this);
}


void DepthImageVeilingEffectFilterNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("imageCallback");
}


} // namespace depth_image_veiling_effect_filter
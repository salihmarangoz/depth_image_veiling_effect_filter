
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_image_veiling_effect_filter");
    ros::NodeHandle nh("/");
    ros::NodeHandle priv_nh("~");
    depth_image_veiling_effect_filter::DepthImageVeilingEffectFilterNode filter_node(nh, priv_nh);
    ros::spin();
    return 0;
}
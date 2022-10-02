
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_image_veiling_effect_filter");
    ros::NodeHandle nhp("~");
    depth_image_veiling_effect_filter::DepthImageVeilingEffectFilterNode filter_node(nhp);
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}
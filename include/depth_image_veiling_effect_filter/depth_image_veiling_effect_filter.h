#ifndef __DEPTH_IMAGE_VEILING_EFFECT_FILTER__
#define __DEPTH_IMAGE_VEILING_EFFECT_FILTER__

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace depth_image_veiling_effect_filter
{


class DepthImageVeilingEffectFilter
{
public:
    DepthImageVeilingEffectFilter();
    void process();
};


class DepthImageVeilingEffectFilterNode
{
public:
    DepthImageVeilingEffectFilterNode(ros::NodeHandle &nhp);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    image_transport::ImageTransport it;
    image_transport::Subscriber sub;

};

} // namespace depth_image_veiling_effect_filter

#endif // __DEPTH_IMAGE_VEILING_EFFECT_FILTER__
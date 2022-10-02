#ifndef __DEPTH_IMAGE_VEILING_EFFECT_FILTER__
#define __DEPTH_IMAGE_VEILING_EFFECT_FILTER__

#include <depth_image_veiling_effect_filter/DepthImageVeilingEffectFilterConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>


namespace depth_image_veiling_effect_filter
{


class DepthImageVeilingEffectFilter
{
public:
    DepthImageVeilingEffectFilter(double threshold=80.0);
    sensor_msgs::ImagePtr process(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug=nullptr);
    template<typename T> sensor_msgs::ImagePtr process_(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug=nullptr);

    void setThreshold(double threshold);
private:
    double threshold_;
};


class DepthImageVeilingEffectFilterNode
{
public:
    DepthImageVeilingEffectFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);
    void reconfigureCallback(DepthImageVeilingEffectFilterConfig &config, uint32_t level);

    image_transport::ImageTransport it;
    image_transport::CameraPublisher pub;
    image_transport::CameraPublisher pub_debug;
    image_transport::CameraSubscriber sub;
    dynamic_reconfigure::Server<DepthImageVeilingEffectFilterConfig> dynrec_server;
    //DepthImageVeilingEffectFilterConfig config;
    DepthImageVeilingEffectFilter filter;
};

} // namespace depth_image_veiling_effect_filter

#endif // __DEPTH_IMAGE_VEILING_EFFECT_FILTER__
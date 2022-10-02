
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

namespace depth_image_veiling_effect_filter
{

DepthImageVeilingEffectFilter::DepthImageVeilingEffectFilter(double threshold)
{
    setThreshold(threshold_);
}

void DepthImageVeilingEffectFilter::setThreshold(double threshold)
{
    // TODO: boundary check!
    threshold_ = threshold;
}

sensor_msgs::ImagePtr DepthImageVeilingEffectFilter::process(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug)
{
    // TODO: check only pinhole camera model is supported

    if (input->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        return process_<uint16_t>(input, info, debug);
    }
    else if (input->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        return process_<float>(input, info, debug);
    }
    else
    {
        ROS_ERROR_THROTTLE(1.0, "Depth image has unsupported encoding [%s]", input->encoding.c_str());
        return nullptr;
    }
}

template<typename T>
sensor_msgs::ImagePtr DepthImageVeilingEffectFilter::process_(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug)
{
    // Prepare output image
    sensor_msgs::ImagePtr output = boost::make_shared<sensor_msgs::Image>();
    output->header = input->header;
    output->height = input->height;
    output->width = input->width;
    output->encoding = input->encoding;
    output->is_bigendian = input->is_bigendian;
    output->step = input->step;
    output->data.resize(output->height * output->step);
    depth_image_proc::DepthTraits<T>::initializeBuffer(output->data);

    // Prepare debug image
    if (debug!=nullptr)
    {
        debug->header = input->header;
        debug->height = input->height;
        debug->width = input->width;
        debug->encoding = input->encoding;
        debug->is_bigendian = input->is_bigendian;
        debug->step = input->step;
        debug->data.resize(debug->height * debug->step);
        depth_image_proc::DepthTraits<T>::initializeBuffer(debug->data);
    }

    // Load camera model and extract all the parameters we need
    image_geometry::PinholeCameraModel depth_model;
    depth_model.fromCameraInfo(info);
    double inv_depth_fx = 1.0 / depth_model.fx();
    double inv_depth_fy = 1.0 / depth_model.fy();
    double depth_cx = depth_model.cx(), depth_cy = depth_model.cy();
    double depth_Tx = depth_model.Tx(), depth_Ty = depth_model.Ty();

    // Cast data pointers
    const T* input_data = reinterpret_cast<const T*>(&input->data[0]);
    T* output_data = reinterpret_cast<T*>(&output->data[0]);
    T* debug_data = nullptr;
    if (debug!=nullptr)
    {
        debug_data = reinterpret_cast<T*>(&debug->data[0]);
    }

    // Apply the filter
    for (unsigned v = 0; v < input->height-1; ++v)
    {
        for (unsigned u = 0; u < input->width-1; ++u)
        {
            const T& raw_input_depth = input_data[v*input->width + u];
            const T& raw_input_next1_depth = input_data[v*input->width + u + 1];
            const T& raw_input_next2_depth = input_data[(v+1)*input->width + u];
            T& raw_output_depth = output_data[v*input->width + u]; // input->width == output->width
            

            if (!depth_image_proc::DepthTraits<T>::valid(raw_input_depth))
            {
                continue;
            }

            double depth = depth_image_proc::DepthTraits<T>::toMeters(raw_input_depth);
            double next1_depth = depth_image_proc::DepthTraits<T>::toMeters(raw_input_next1_depth);
            double next2_depth = depth_image_proc::DepthTraits<T>::toMeters(raw_input_next2_depth);

            if (std::abs(depth-next1_depth) > threshold_/100.0 || std::abs(depth-next2_depth) > threshold_/100.0)
            {
                if (debug!=nullptr)
                {
                    T& raw_debug_depth = debug_data[v*input->width + u]; // input->width == output->width
                    raw_debug_depth = depth_image_proc::DepthTraits<T>::fromMeters(depth);
                }
            }
            else
            {
                raw_output_depth = depth_image_proc::DepthTraits<T>::fromMeters(depth);
            }

        }
    }
    
    return output;
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

    if (pub.getNumSubscribers() == 0 && pub_debug.getNumSubscribers() == 0) return; // dont waste resources in no one is subscribed to any of our topics

    if (pub_debug.getNumSubscribers() > 0)
    {
        ROS_WARN_THROTTLE(1.0, "Publishing debug images may increase CPU usage");
        sensor_msgs::ImagePtr debug_msg = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::ImagePtr filtered_msg = filter.process(msg, info, debug_msg);
        pub.publish(filtered_msg, info);
        pub_debug.publish(debug_msg, info);
    }
    else
    {
        sensor_msgs::ImagePtr filtered_msg = filter.process(msg, info);
        pub.publish(filtered_msg, info);
    }
}

void DepthImageVeilingEffectFilterNode::reconfigureCallback(DepthImageVeilingEffectFilterConfig &config, uint32_t level)
{
    ROS_INFO("DepthImageVeilingEffectFilter threshold changed to %f", config.threshold);
    filter.setThreshold(config.threshold);
}


} // namespace depth_image_veiling_effect_filter
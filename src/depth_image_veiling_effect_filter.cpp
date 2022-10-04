
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Dense>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>

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
    unsigned width = input->width;
    unsigned height = input->height;

    // Cast data pointers
    const T* input_data = reinterpret_cast<const T*>(&input->data[0]);
    T* output_data = reinterpret_cast<T*>(&output->data[0]);
    T* debug_data = nullptr;
    if (debug!=nullptr)
    {
        debug_data = reinterpret_cast<T*>(&debug->data[0]);
    }


    // Filter using pcl::RangeImagePlanar::getImpactAngleImageBasedOnLocalNormals()
    pcl::RangeImagePlanar rip;
    rip.setDepthImage(input_data, width, height, depth_cx, depth_cy, depth_model.fx(), depth_model.fy());
    float* angles = rip.getImpactAngleImageBasedOnLocalNormals(1);
    for (int i=0; i<width*height; i++)
    {
        if (angles[i] > threshold_)
        {
            output_data[i] = input_data[i];
        }
        else
        {
            debug_data[i] = input_data[i];
        }
        
    }
    delete angles;

    // Filter using pcl::RangeImagePlanar::getSurfaceChangeImage() // (not implemented!)

    /*
    pcl::RangeImagePlanar rip;
    rip.setDepthImage(input_data, width, height, depth_cx, depth_cy, depth_model.fx(), depth_model.fy());
    pcl::RangeImageBorderExtractor ribe;
    ribe.setRangeImage(&rip);
    float* angles = ribe.getSurfaceChangeScores();
    
    for (int i=0; i<width*height; i++)
    {
        if (angles[i] > threshold_)
        {
            output_data[i] = input_data[i];
        }
        else
        {
            debug_data[i] = input_data[i];
        }
        //output_data[i] = angles[i]*1000;
    }
    printf("%d ", angles);
    //delete angles;
    */


    /*
    // convert to xyz
    std::vector<Eigen::Vector3d> input_xyz;
    input_xyz.reserve(height * width);
    Eigen::Vector3d bad_point(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());

    for (unsigned v = 0; v < height; ++v)
    {
        for (unsigned u = 0; u < width; ++u)
        {
            const T& raw_input_depth = input_data[v*width + u];
            if (!depth_image_proc::DepthTraits<T>::valid(raw_input_depth))
            {
                input_xyz.push_back(bad_point);
            }
            else
            {
                double depth = depth_image_proc::DepthTraits<T>::toMeters(raw_input_depth);
                Eigen::Vector3d point(((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                                      ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                                      depth);
                input_xyz.push_back(point);
            }
        }
    }


    // Apply the filter
    int u_plus[] = {0, -1, -1, -1};
    int v_plus[] = {1, 1, 0, -1};
    for (unsigned v = 1; v < height-1; ++v)
    {
        for (unsigned u = 1; u < width-1; ++u)
        {
            double min_angle = 999;
            for (int i=0; i<4; i++)
            {
                Eigen::Vector3d &p0 = input_xyz[(v + v_plus[i])*width + u + u_plus[i]];
                Eigen::Vector3d &p1 = input_xyz[(v - v_plus[i])*width + u - u_plus[i]];

                if (std::isnan(p0(0)) || std::isnan(p1(0)))
                {
                    min_angle = 999;
                    break;
                }

                double angle = std::abs(std::acos(p0.dot(p0-p1)/p0.norm()/(p0-p1).norm()));
                if (std::isnan(angle)) min_angle = 0.0; // todo
                if (angle < min_angle) min_angle = angle;
            }

            if (min_angle < threshold_)
            {
                if (debug!=nullptr)
                {
                    debug_data[v*width + u] = input_data[(v)*width + u];
                }

                // todo
                for (int i=-1; i<1; i++)
                {
                    for (int j=-1; j<1; j++)
                    {
                        output_data[(v + i)*width + u + j] = 0; // todo for float
                    }
                }
            }
            else
            {
                output_data[v*width + u] = input_data[v*width + u];
            }
        }
    }
    */
    
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
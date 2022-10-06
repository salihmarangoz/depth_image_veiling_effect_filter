
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Dense>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <sensor_msgs/PointCloud.h>

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

void DepthImageVeilingEffectFilter::setK(int k)
{
    k_ = k;
}

void DepthImageVeilingEffectFilter::setMethod(int method)
{
    method_ = method;
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

    //---------------------------------------------------------------------------------------------------------------
    // Filter using pcl::RangeImagePlanar::getImpactAngleImageBasedOnLocalNormals()
    //---------------------------------------------------------------------------------------------------------------
    if (method_ == 0)
    {
        pcl::RangeImagePlanar rip;
        rip.setDepthImage(input_data, width, height, depth_cx, depth_cy, depth_model.fx(), depth_model.fy());
        float* angles = rip.getImpactAngleImageBasedOnLocalNormals(k_);
        for (int i=0; i<width*height; i++)
        {
            if (angles[i] > threshold_) // possibly handles nan values
            {
                output_data[i] = input_data[i];
            }
            else
            {
                if (debug!=nullptr)
                    debug_data[i] = input_data[i];
            }
            
        }
        delete[] angles;
    }

    //---------------------------------------------------------------------------------------------------------------
    // Filter using custom method
    //---------------------------------------------------------------------------------------------------------------

    if (method_ == 1)
    {
        // convert to xyz
        std::vector<Eigen::Vector3d> input_xyz;
        std::vector<double> depth_wrt_cam;
        input_xyz.reserve(height * width);
        depth_wrt_cam.reserve(height * width);
        Eigen::Vector3d bad_point(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());

        sensor_msgs::PointCloud::Ptr pc = boost::make_shared<sensor_msgs::PointCloud>();
        pc->header = input->header;

        for (unsigned v = 0; v < height; ++v)
        {
            for (unsigned u = 0; u < width; ++u)
            {
                const T& raw_input_depth = input_data[v*width + u];
                if (!depth_image_proc::DepthTraits<T>::valid(raw_input_depth))
                {
                    input_xyz.push_back(bad_point);
                    /*
                    geometry_msgs::Point32 p;
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                    pc->points.push_back(p);
                    */
                }
                else
                {
                    double depth = depth_image_proc::DepthTraits<T>::toMeters(raw_input_depth);
                    Eigen::Vector3d point(((u - depth_cx)*depth - depth_Tx) * inv_depth_fx,
                                          ((v - depth_cy)*depth - depth_Ty) * inv_depth_fy,
                                          depth);
                    input_xyz.push_back(point);
                    depth_wrt_cam.push_back(point.norm());

                    /*
                    geometry_msgs::Point32 p;
                    p.x = point.x();
                    p.y = point.y();
                    p.z = point.z();
                    pc->points.push_back(p);
                    */
                }
            }
        }

        sensor_msgs::ChannelFloat32 ch;
        ch.name = "angle";
        ch.values.resize(width*height, 0.0);

        // start time
        auto start_time = std::chrono::high_resolution_clock::now();

        // Apply the filter
        //int u_plus[] = {0, -1, -1, -1};
        //int v_plus[] = {1, 1, 0, -1};
        for (unsigned v = k_; v < height-k_; ++v)
        {
            for (unsigned u = k_; u < width-k_; ++u)
            {
                Eigen::Vector3d &p0 = input_xyz[(v)*width + u]; // todo?
                Eigen::Vector3d p0_n = p0.normalized();
                bool remove_point = false;
                for (int i=-k_; i<k_; i++)
                {
                    for (int j=-k_; j<k_; j++)
                    {
                        if (!depth_image_proc::DepthTraits<T>::valid(input_data[(v + i)*width + u + j]))
                        {
                            remove_point = true;
                            break;
                        }

                        if (i==0 && j==0) continue;

                        Eigen::Vector3d &p1 = input_xyz[(v + i)*width + u + j]; 
                        Eigen::Vector3d diff = p0-p1;
                        diff.normalize();

                        double angle = std::abs(std::acos(p0_n.dot(diff)));

                        if (angle < threshold_)
                        {
                            remove_point = true;
                            break;
                        }
                    }
                    if (remove_point) break;
                }

                //ch.values.at(v*width + u) = min_angle;

                if (remove_point)
                {
                    if (debug!=nullptr)
                    {
                        debug_data[v*width + u] = input_data[(v)*width + u];
                    }

                    // todo
                    for (int i=-k_; i<k_; i++)
                    {
                        for (int j=-k_; j<k_; j++)
                        {
                            output_data[(v + i)*width + u + j] = 0; // todo for float
                            if (debug!=nullptr)
                                debug_data[(v + i)*width + u + j] = input_data[(v)*width + u]; // todo for float
                        }
                    }
                }
                else
                {
                    output_data[v*width + u] = input_data[v*width + u];
                }
            }
        }

        //pc->channels.push_back(ch);
        //pub_pc.publish(pc);

        // end time
        auto end_time = std::chrono::high_resolution_clock::now();
        printf("custom method took %d ms.\n", std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count());
    }

    //---------------------------------------------------------------------------------------------------------------
    // Filter using pcl::RangeImageBorderExtractor
    // Needs compiling PCL:
    // Download PCL 1.10.0, comment lines 152 and 153 in range_image_border_extractor.cpp, compile, and install
    //---------------------------------------------------------------------------------------------------------------
    /*
    pcl::RangeImagePlanar rip;
    rip.setDepthImage(input_data, width, height, depth_cx, depth_cy, depth_model.fx(), depth_model.fy());
    pcl::RangeImageBorderExtractor ribe;
    ribe.setRangeImage(&rip);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_descriptions.points.reserve(width*height);
    ribe.compute (border_descriptions);

    for (int y=0; y< height; ++y)
    {
        for (int x=0; x< width; ++x)
        {
            if (border_descriptions[y*width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                if (debug!=nullptr)
                    debug_data[y*width + x] = input_data[y*width + x];
            else
                output_data[y*width + x] = input_data[y*width + x];
        }
    }
    */
    
    //---------------------------------------------------------------------------------------------------------------
    // Filter using pcl::RangeImagePlanar::getSurfaceChangeImage() 
    //---------------------------------------------------------------------------------------------------------------

    // (method not implemented. weird?)

    return output;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

DepthImageVeilingEffectFilterNode::DepthImageVeilingEffectFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it(pnh), dynrec_server(pnh)
{
    pub = it.advertiseCamera("/camera/aligned_depth_to_color_filtered/image_raw", 1, true); // TODO: latch only for debug
    pub_debug = it.advertiseCamera("/camera/aligned_depth_to_color_filtered/image_raw_debug", 1, true); // TODO: latch only for debug
    sub = it.subscribeCamera("/camera/aligned_depth_to_color/image_raw", 1, &DepthImageVeilingEffectFilterNode::imageCallback, this);
    filter.pub_pc = pnh.advertise<sensor_msgs::PointCloud>("points", 1, true);
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
    filter.setMethod(config.method);
    filter.setK(config.k);
}


} // namespace depth_image_veiling_effect_filter
#include <ros/ros.h>
#include "depth_image_veiling_effect_filter/depth_image_veiling_effect_filter.h"
#include <depth_image_proc/depth_traits.h>
#include <depth_image_proc/depth_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <Eigen/Dense>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace depth_image_veiling_effect_filter
{

DepthImageVeilingEffectFilter::DepthImageVeilingEffectFilter(){}


// NOTE: range_max works differently compared to depth_image_proc!
// This function removes points with higher range value. depth_image_proc::convert replaces invalid points with range_max
template<typename T>
pcl::PointCloud<pcl::PointXYZ>::Ptr DepthImageVeilingEffectFilter::convertAndFilter(const sensor_msgs::ImagePtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& camera_info, double range_max)
{
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info);

    unsigned int height = depth_msg->height;
    unsigned int width = depth_msg->width;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_msg->points.resize(height * width);
    cloud_msg->width = width;
    cloud_msg->height = height;
    cloud_msg->is_dense = false;

    // Use correct principal point from calibration
    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin();
    T* depth_row = reinterpret_cast<T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);
    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)cloud_msg->width; ++u)
        {
            pcl::PointXYZ& pt = *pt_iter++;
            T depth = depth_row[u];

            // Missing points denoted by NaNs
            if (!depth_image_proc::DepthTraits<T>::valid(depth))
            {
                pt.x = pt.y = pt.z = bad_point;
                depth_row[u] = bad_point;
                continue;
            }

            double depth_meters = depth_image_proc::DepthTraits<T>::toMeters(depth);

            if (range_max < depth_meters)
            {
                pt.x = pt.y = pt.z = bad_point;
                depth_row[u] = bad_point;
                continue;
            }

            // Fill in XYZ
            pt.x = (u - center_x) * depth * constant_x;
            pt.y = (v - center_y) * depth * constant_y;
            pt.z = depth_meters;
        }
    }

    return cloud_msg;
}

sensor_msgs::ImagePtr DepthImageVeilingEffectFilter::process(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    sensor_msgs::ImagePtr out = nullptr;

    // TODO: check only pinhole camera model is supported

    if (input->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        out = process_<uint16_t>(input, info, debug);
    }
    else if (input->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        out = process_<float>(input, info, debug);
    }
    else
    {
        ROS_ERROR_THROTTLE(1.0, "Depth image has unsupported encoding [%s]", input->encoding.c_str());
    }

    if (out != nullptr)
    {
        auto end_time = std::chrono::high_resolution_clock::now();
        printf("Filtering took %ld ms.\n", std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count());
    }
    
    return out;
}

template<typename T>
sensor_msgs::ImagePtr DepthImageVeilingEffectFilter::process_(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug)
{
    // Copy input image
    sensor_msgs::ImagePtr filtered_img = boost::make_shared<sensor_msgs::Image>();
    (*filtered_img) = (*input);

    // Load camera model and extract all the parameters we need
    image_geometry::PinholeCameraModel depth_model;
    depth_model.fromCameraInfo(info);
    double inv_depth_fx = 1.0 / depth_model.fx();
    double inv_depth_fy = 1.0 / depth_model.fy();
    double depth_cx = depth_model.cx(), depth_cy = depth_model.cy();
    double depth_Tx = depth_model.Tx(), depth_Ty = depth_model.Ty();
    unsigned width = input->width;
    unsigned height = input->height;

    // Cast data pointers to depth images
    const T* input_data = reinterpret_cast<const T*>(&input->data[0]);
    T* filtered_data = reinterpret_cast<T*>(&filtered_img->data[0]);

    //==================================================================================
    //==================== DEPTH TO POINTCLOUD + CONDITIONAL FILTER ====================
    //==================================================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc = convertAndFilter<T>(filtered_img, info, p_max_range); // this command filters input img as well!

    //==================================================================================
    //==================== SURFACE NORMAL BASED FILTERING ==============================
    //==================================================================================

    if (p_normal_based_filtering_method == 0)
    {
        //------------------------------------------------------------------------------
        // Disabled filtering
        //------------------------------------------------------------------------------

        // todo
    }
    else if (p_normal_based_filtering_method == 1)
    {
        //------------------------------------------------------------------------------
        // Filter using pcl::RangeImagePlanar::getImpactAngleImageBasedOnLocalNormals
        //------------------------------------------------------------------------------

        pcl::RangeImagePlanar rip;
        rip.setDepthImage(filtered_data, width, height, depth_cx, depth_cy, depth_model.fx(), depth_model.fy());
        float* angles = rip.getImpactAngleImageBasedOnLocalNormals(p_rip_radius);
        for (int i=0; i<width*height; i++)
        {
            if (angles[i] < p_rip_threshold)
            {
                filtered_data[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }
        delete[] angles;
    }
    else if (p_normal_based_filtering_method == 2)
    {
        //------------------------------------------------------------------------------
        // Filter using pcl::IntegralImageNormalEstimation + pcl::ShadowPoints
        //------------------------------------------------------------------------------

        // surface normal estimation
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod( (pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod) p_sp_normal_estimation_method);
        ne.setMaxDepthChangeFactor(p_sp_max_depth_change_factor);
        ne.setNormalSmoothingSize(p_sp_normal_smoothing_size);
        ne.setDepthDependentSmoothing(p_sp_use_depth_dependent_smoothing);
        ne.useSensorOriginAsViewPoint();
        ne.setInputCloud(filtered_pc);
        ne.compute(*normals);

        // filter points
        pcl::Indices indices;
        pcl::ShadowPoints<pcl::PointXYZ, pcl::Normal> sp(true);
        sp.setInputCloud(filtered_pc);
        sp.setNormals(normals);
        sp.setThreshold(p_sp_threshold);
        sp.setNegative(true);
        sp.filter(indices);

        for (int i=0; i<indices.size(); i++)
        {
            filtered_pc->at(i) = pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
            filtered_data[indices[i]] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    //==================================================================================
    //==================== OUTLIER / STATISTICAL FILTERING =============================
    //==================================================================================

    
    if (p_statistical_filtering_method == 0)
    {
        //---------------------------------------------------------------------------------------------------------------
        // Disabled filtering
        //---------------------------------------------------------------------------------------------------------------
    }
    if (p_statistical_filtering_method == 1)
    {
        //---------------------------------------------------------------------------------------------------------------
        // Filter using pcl::StatisticalOutlierRemoval
        //---------------------------------------------------------------------------------------------------------------

        // filter points
        pcl::Indices indices;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
        sor.setInputCloud(filtered_pc);
        sor.setMeanK(p_sor_mean_k);
        sor.setStddevMulThresh(p_sor_stddevmulthresh);
        sor.setNegative(true);
        sor.filter(indices);

        for (int i=0; i<indices.size(); i++)
        {
            //filtered_pc->at(i) = pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
            filtered_data[indices[i]] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // NOTE: IF YOU WOULD LIKE TO ADD ONE MORE STEP TO THE PIPELINE UNCOMMENT LINES STARTING WITH "filtered_pc->at(i)" IN THIS SECTION

    //==================================================================================

    // Prepare the debug image
    if (debug!=nullptr)
    {
        (*debug) = (*input); // copy from input
        T* debug_data = reinterpret_cast<T*>(&debug->data[0]);

        for (int i=0; i<height*width; i++)
        {
            if (depth_image_proc::DepthTraits<T>::valid(filtered_data[i]))
            {
                debug_data[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    return filtered_img;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

DepthImageVeilingEffectFilterNode::DepthImageVeilingEffectFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it(pnh), dynrec_server(pnh)
{
    pub = it.advertiseCamera("/camera/aligned_depth_to_color_filtered/image_raw", 1, true); // TODO: latch only for debug
    pub_debug = it.advertiseCamera("/camera/aligned_depth_to_color_filtered/image_raw_debug", 1, true); // TODO: latch only for debug
    sub = it.subscribeCamera("/camera/aligned_depth_to_color/image_raw", 1, &DepthImageVeilingEffectFilterNode::imageCallback, this);
    //filter.pub_pc = pnh.advertise<sensor_msgs::PointCloud>("points", 1, true);
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
    filter.set_max_range(config.max_range);
    //----------------------------
    filter.set_normal_based_filtering_method(config.normal_based_filtering_method);
    filter.set_rip_radius(config.rip_radius);
    filter.set_rip_threshold(config.rip_threshold);
    filter.set_sp_normal_estimation_method(config.sp_normal_estimation_method);
    filter.set_sp_normal_smoothing_size(config.sp_normal_smoothing_size);
    filter.set_sp_max_depth_change_factor(config.sp_max_depth_change_factor);
    filter.set_sp_use_depth_dependent_smoothing(config.sp_use_depth_dependent_smoothing);
    filter.set_sp_threshold(config.sp_threshold);
    //----------------------------
    filter.set_statistical_filtering_method(config.statistical_filtering_method);
    filter.set_sor_mean_k(config.sor_mean_k);
    filter.set_sor_stddevmulthresh(config.sor_stddevmulthresh);

    
}


} // namespace depth_image_veiling_effect_filter
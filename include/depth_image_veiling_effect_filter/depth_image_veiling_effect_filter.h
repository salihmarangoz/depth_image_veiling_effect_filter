#ifndef __DEPTH_IMAGE_VEILING_EFFECT_FILTER__
#define __DEPTH_IMAGE_VEILING_EFFECT_FILTER__

#include <depth_image_veiling_effect_filter/DepthImageVeilingEffectFilterConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>


namespace depth_image_veiling_effect_filter
{


class DepthImageVeilingEffectFilter
{
public:
    DepthImageVeilingEffectFilter();
    sensor_msgs::ImagePtr process(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug=nullptr);
    template<typename T> sensor_msgs::ImagePtr process_(const sensor_msgs::ImageConstPtr& input, const sensor_msgs::CameraInfoConstPtr& info, sensor_msgs::ImagePtr debug=nullptr);
    template<typename T> pcl::PointCloud<pcl::PointXYZ>::Ptr convertAndFilter(const sensor_msgs::ImagePtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& camera_info, double range_max=HUGE_VAL);

    void set_normal_based_filtering_method(int new_value){p_normal_based_filtering_method = new_value;}
    void set_max_range(double new_value){p_max_range = new_value;}
    void set_rip_radius(int new_value){p_rip_radius = new_value;}
    void set_rip_threshold(double new_value){p_rip_threshold = new_value;}
    void set_sp_normal_estimation_method(int new_value){p_sp_normal_estimation_method = new_value;}
    void set_sp_normal_smoothing_size(double new_value){p_sp_normal_smoothing_size = new_value;}
    void set_sp_max_depth_change_factor(double new_value){p_sp_max_depth_change_factor = new_value;}
    void set_sp_use_depth_dependent_smoothing(bool new_value){p_sp_use_depth_dependent_smoothing = new_value;}
    void set_sp_threshold(double new_value){p_sp_threshold = new_value;}
    void set_statistical_filtering_method(int new_value){p_statistical_filtering_method = new_value;}
    void set_sor_mean_k(int new_value){p_sor_mean_k = new_value;}
    void set_sor_stddevmulthresh(double new_value){p_sor_stddevmulthresh = new_value;}

private:
    //-------------------------------------------
    int      p_normal_based_filtering_method = 0;
    double   p_max_range = HUGE_VAL;
    //-------------------------------------------
    int      p_rip_radius = 5;
    double   p_rip_threshold = 0.15;
    //-------------------------------------------
    int      p_sp_normal_estimation_method = 0;
    double   p_sp_max_depth_change_factor = 0.1;
    double   p_sp_normal_smoothing_size = 10.0;
    bool     p_sp_use_depth_dependent_smoothing = true;
    double   p_sp_threshold = 0.15;
    //-------------------------------------------
    int      p_statistical_filtering_method = 0;
    //-------------------------------------------
    int      p_sor_mean_k = 50;
    double   p_sor_stddevmulthresh = 1.0;



public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
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
public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace depth_image_veiling_effect_filter

#endif // __DEPTH_IMAGE_VEILING_EFFECT_FILTER__
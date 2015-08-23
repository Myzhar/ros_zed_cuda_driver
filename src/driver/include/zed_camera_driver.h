#ifndef ZED_CAMERA_DRIVER_H
#define ZED_CAMERA_DRIVER_H

//ZED include
#include <zed/Camera.hpp>
#include "cuda.h"
#include "cuda_runtime.h"


#include <ros/ros.h>
#include <csignal>
#include <list>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

#include <pthread.h>

using namespace std;
using namespace sl;

class ZedDriver
{
public:
    typedef union _RGBA
    {
        struct _color
        {
            uint8_t b;
            uint8_t g;
            uint8_t r;
            uint8_t a;
        } color;

        float val;
    } RGBA;

    ZedDriver();
    ~ZedDriver();

    bool init();
    void releaseCamera();


    void  start();

    /// Function used to start a thread inside the class
    static void* callRunFunction(void *arg) { return ((ZedDriver*)arg)->run(); }

private:
    void loadParams();

    void initCamInfo(zed::StereoParameters *params);

    // >>>>> Ctrl+C handler
    /*! Ctrl+C handler
     */
    static void sighandler(int signo)
    {
        ZedDriver::_stopping = (signo == SIGINT);
    }
    // <<<<< Ctrl+C handler

    void* run();

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _nhPriv;
    ros::Publisher _vertex_pub;
    ros::Publisher _vertex_reg_pub;

    sensor_msgs::CameraInfo _left_cam_info_msg;
    sensor_msgs::CameraInfo _right_cam_info_msg;
    sensor_msgs::CameraInfo _depth_cam_info_msg;

    // >>>>> Image transportation RGB
    image_transport::ImageTransport _rgb_left_ImgTr;
    image_transport::CameraPublisher _rgb_left_pub;
    image_transport::ImageTransport _rgb_right_ImgTr;
    image_transport::CameraPublisher _rgb_right_pub;
    // <<<<< Image transportation RGB

    // >>>>> Image transportation Depth/Confidence
    ros::Publisher _disparity_pub;
    image_transport::ImageTransport _depth_ImgTr;
    image_transport::ImageTransport _confidence_ImgTr;
    image_transport::CameraPublisher _depth_pub;
    image_transport::CameraPublisher _norm_confidence_pub;
    // <<<<< Image transportation Depth/Confidence

    bool _initialized;
    bool _streaming;
    bool _error;

    static bool _stopping;

    // >>>>> node params
    zed::ZEDResolution_mode _resol;     ///< Camera resolution
    double _fps;                        ///< Acquisition FPS
    bool _publish_tf;                   ///< Publish TF if true
    bool _enable_rgb;                   ///< Publish RGB stream if true
    //bool _enable_depth_confidence;      ///< Publish Depth and Confidence stream if true
    bool _enable_depth;                 ///< Publish raw floating point Depth Map
    bool _enable_norm_confidence;       ///< Publish normalized Confidence Map
    bool _enable_disp;                  ///< Publish Disparity Map
    int  _conf_thresh;                  ///< Confidence threshold [DYN]
    bool _enable_ptcloud;               ///< Publish 3d pointcloud if true.
    bool _enable_registered;            ///< Publish 3d registered pointcloud if true and @ref _enable_rgb is true and @ref _enable_ptcloud is true
    // <<<<< node params

    pthread_t _threadId;
    zed::Camera* _zed_camera;
};

#endif

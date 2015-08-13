#ifndef ZED_CAMERA_DRIVER_H
#define ZED_CAMERA_DRIVER_H

//ZED include
#include <zed/Camera.hpp>


#include <ros/ros.h>
#include <csignal>
#include <list>
#include <image_transport/image_transport.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

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
    void release();


    void  start();

    /// Function used to start a thread inside the class
    static void* callRunFunction(void *arg) { return ((ZedDriver*)arg)->run(); }

private:
    void loadParams();

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
    ros::Publisher _vertex_pub;
    ros::Publisher _vertex_reg_pub;

    // >>>>> Image transportation RGB
    image_transport::ImageTransport _rgb_left_ImgTr;
    image_transport::CameraPublisher _rgb_left_pub;
    image_transport::ImageTransport _rgb_right_ImgTr;
    image_transport::CameraPublisher _rgb_right_pub;
    // <<<<< Image transportation RGB

    // >>>>> Image transportation Depth/Confidence
    image_transport::ImageTransport _depth_ImgTr;
    image_transport::CameraPublisher _depth_pub;
    image_transport::ImageTransport _confidence_ImgTr;
    image_transport::CameraPublisher _confidence_pub;
    // <<<<< Image transportation Depth/Confidence

    bool _initialized;
    bool _streaming;
    bool _error;

    static bool _stopping;

    // >>>>> node params
    zed::ZEDResolution_mode _resol; ///< Camera resolution
    bool _publish_tf;                   ///< Publish TF if true
    bool _enable_rgb;                   ///< Publish RGB stream if true
    bool _enable_depth_confidence;      ///< Publish Depth and Confidence stream if true
    int  _conf_thresh;                  ///< Confidence threshold [DYN]
    bool _enable_ptcloud;               ///< Publish 3d pointcloud if true.
    bool _enable_registered;            ///< Publish 3d registered pointcloud if true and @ref _enable_rgb is true and @ref _enable_ptcloud is true
    // <<<<< node params

    pthread_t _threadId;
    zed::Camera* _zed_camera;
};

#endif

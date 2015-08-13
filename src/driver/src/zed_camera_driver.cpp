#include "zed_camera_driver.h"

//ZED include
#include <zed/Camera.hpp>

#include <string>

bool ZedDriver::_stopping = false;

using namespace std;

ZedDriver::ZedDriver()
    : _initialized(false)
    , _streaming(false)
    , _error(false)
    , _rgb_left_ImgTr(_nh)
    , _depth_ImgTr(_nh)
    , _rgb_right_ImgTr(_nh)
    , _confidence_ImgTr(_nh)
    , _zed_camera(NULL)
{
    struct sigaction sigAct;
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = ZedDriver::sighandler;
    sigaction(SIGINT, &sigAct, 0);

    loadParams();

    if( _enable_ptcloud )
        _vertex_pub = _nh.advertise<sensor_msgs::PointCloud2>("vertex_data", 1, false);

    if( _enable_rgb )
    {
        _rgb_left_pub = _rgb_left_ImgTr.advertiseCamera("rgb_left_image", 1, false);
        _rgb_right_pub = _rgb_right_ImgTr.advertiseCamera("rgb_right_image", 1, false);
    }

    if( _enable_depth_confidence )
    {
        _depth_pub = _depth_ImgTr.advertiseCamera("depth_image", 1,false);
        _confidence_pub = _confidence_ImgTr.advertiseCamera("confidence_image", 1, false);
    }

    if( _enable_ptcloud && _enable_rgb && _enable_registered )
        _vertex_reg_pub = _nh.advertise<sensor_msgs::PointCloud2>("vertex_rgb_data", 1, false);

}

ZedDriver::~ZedDriver()
{
    release();
}

bool ZedDriver::init()
{
    if( _zed_camera )
        release();

    // TODO set FPS from params!
    _zed_camera = new zed::Camera( _resol, 0.0f );

    zed::MODE mode = _enable_depth_confidence==true?zed::PERFORMANCE:zed::NONE;

    zed::ERRCODE err = _zed_camera->init( mode, 0, true );


    if( err != zed::SUCCESS )
    {
        delete _zed_camera;

        // ERRCODE display
        ROS_ERROR_STREAM( "ZED Init Err: " << errcode2str(err) );
        return false;
    }

    ROS_INFO_STREAM( "ZED Serial Number : " << _zed_camera->getZEDSerial() );

    return true;
}

void ZedDriver::release()
{
    // TODO deinizializzazione

    delete _zed_camera;
    _zed_camera = NULL;
}

void* ZedDriver::run()
{
    _stopping = false;

    zed::SENSING_MODE dm_type = zed::RAW;

    zed::Mat depth, imLeft, imRight, disparity, confidence;

    while( 1 )
    {
        if( _stopping )
        {
            break;
        }

        ros::spinOnce(); // Processing ROS messages

        // ZED Grabbing
        if( !_zed_camera->grab(dm_type) )
            continue;

        if( _enable_depth_confidence )
        {
            // TODO verify the buffer overwriting as reported on DOCS
            // file:///usr/local/zed/doc/API/classsl_1_1zed_1_1Camera.html#a7ae4783e231502e7681890636e24e49c
            depth = _zed_camera->retrieveMeasure(zed::MEASURE::DEPTH);
            disparity = _zed_camera->retrieveMeasure(zed::MEASURE::DISPARITY);
            confidence = _zed_camera->retrieveMeasure(zed::MEASURE::CONFIDENCE);
        }

        if( _enable_rgb )
        {
            // TODO verify the buffer overwriting as reported on DOCS
            // file:///usr/local/zed/doc/API/classsl_1_1zed_1_1Camera.html#a7ae4783e231502e7681890636e24e49c
            imLeft = _zed_camera->retrieveImage(zed::SIDE::LEFT);
            imRight = _zed_camera->retrieveImage(zed::SIDE::RIGHT);
        }
    }

    ROS_INFO_STREAM( "ZED camera stopped... ");

    release();

    ROS_INFO_STREAM( "Stopping node... ");

    ros::shutdown();

    ROS_INFO_STREAM( "... done.");

    return 0;
}

#define PAR_RESOL                   "zed_camera/resolution"
#define PAR_PUB_TF                  "zed_camera/publish_tf"
#define PAR_ENABLE_RGB              "zed_camera/enable_rgb"
#define PAR_ENABLE_PTCLOUD          "zed_camera/enable_ptcloud"
#define PAR_ENABLE_REGISTERED       "zed_camera/enable_ptcloud_reg"
#define PAR_ENABLE_DEPTH_CONFIDENCE "zed_camera/enable_depth_confidence"
#define PAR_CONF_THRESH             "zed_camera/confidence_thresh"

void ZedDriver::loadParams()
{
    if( _nh.hasParam( PAR_RESOL ) )
    {
        string resolStr;
        _nh.getParam( PAR_RESOL, resolStr );

        if( resolStr.compare( "VGA")==0 )
        {
            _resol = zed::VGA;
        }
        else if( resolStr.compare( "HD2K")==0 )
        {
            _resol = zed::HD2K;
        }
        else if( resolStr.compare( "HD1080")==0 )
        {
            _resol = zed::HD1080;
        }
        else if( resolStr.compare( "HD720")==0 )
        {
            _resol = zed::HD720;
        }
        else
        {
            ROS_WARN_STREAM( "Resolution not correct, using default VGA");
            _resol = zed::VGA;
        }
    }
    else
    {
        _resol = zed::VGA;
        _nh.setParam( PAR_PUB_TF, "VGA" );
    }

    if( _nh.hasParam( PAR_ENABLE_RGB ) )
    {
        _nh.getParam( PAR_ENABLE_RGB, _enable_rgb );
    }
    else
    {
        _enable_rgb = true;
        _nh.setParam( PAR_ENABLE_RGB, _enable_rgb );
    }

    if( _nh.hasParam( PAR_ENABLE_PTCLOUD ) )
    {
        _nh.getParam( PAR_ENABLE_PTCLOUD, _enable_ptcloud );
    }
    else
    {
        _enable_ptcloud = true;
        _nh.setParam( PAR_ENABLE_RGB, _enable_ptcloud );
    }

    if( _nh.hasParam( PAR_ENABLE_REGISTERED ) )
    {
        _nh.getParam( PAR_ENABLE_REGISTERED, _enable_registered );
    }
    else
    {
        _enable_registered = true;
        _nh.setParam( PAR_ENABLE_REGISTERED, _enable_registered );
    }

    if( _nh.hasParam( PAR_ENABLE_DEPTH_CONFIDENCE ) )
    {
        _nh.getParam( PAR_ENABLE_DEPTH_CONFIDENCE, _enable_depth_confidence );
    }
    else
    {
        _enable_depth_confidence = true;
        _nh.setParam( PAR_ENABLE_DEPTH_CONFIDENCE, _enable_depth_confidence );
    }

    if( _nh.hasParam( PAR_CONF_THRESH ) )
    {
        _nh.getParam( PAR_CONF_THRESH, _conf_thresh );
    }
    else
    {
        _conf_thresh = 60;
        _nh.setParam( PAR_CONF_THRESH, _conf_thresh );
    }
}

void  ZedDriver::start(void)
{
    int       result;
    result = pthread_create(&_threadId, 0, ZedDriver::callRunFunction, this);
    if (result == 0)
        pthread_detach(_threadId);
}

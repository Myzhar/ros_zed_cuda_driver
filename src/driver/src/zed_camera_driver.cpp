#include "zed_camera_driver.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <stereo_msgs/DisparityImage.h>

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
    //TODO , _depth_ImgTr(_nh)
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
        _rgb_left_ImgTr.
        _rgb_left_pub = _rgb_left_ImgTr.advertiseCamera("stereo/left/image_rect", 1, false);
        _rgb_right_pub = _rgb_right_ImgTr.advertiseCamera("stereo/right/image_rect", 1, false);
    }

    if( _enable_depth_confidence )
    {
        //TODO _depth_pub = _depth_ImgTr.advertiseCamera("depth_image", 1,false);
        _disparity_pub = _nh.advertise<stereo_msgs::DisparityImage>("stereo/disparity", 1 );

        _confidence_pub = _confidence_ImgTr.advertiseCamera("stereo/confidence", 1, false);
    }

    if( _enable_ptcloud && _enable_rgb && _enable_registered )
        _vertex_reg_pub = _nh.advertise<sensor_msgs::PointCloud2>("stereo/registered_cloud", 1, false);

}

ZedDriver::~ZedDriver()
{
    releaseCamera();
}

bool ZedDriver::init()
{
    releaseCamera();

    // TODO set FPS from params!
    _zed_camera = new zed::Camera( _resol, 0.0f );

    zed::MODE mode = _enable_depth_confidence==true?zed::PERFORMANCE:zed::NONE;

    zed::ERRCODE err = _zed_camera->init( mode, 0, true );


    if( err != zed::SUCCESS )
    {
        releaseCamera();

        // ERRCODE display
        ROS_ERROR_STREAM( "ZED Init Err: " << errcode2str(err) );
        return false;
    }

    ROS_INFO_STREAM( "ZED Serial Number : " << _zed_camera->getZEDSerial() );

    return true;
}

void ZedDriver::releaseCamera()
{
    if( !_stopping )
        _stopping=true;

    if(_zed_camera)
        delete _zed_camera;
    _zed_camera = NULL;
}

void ZedDriver::initCamInfo( zed::StereoParameters* params )
{
    _left_cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    _right_cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    _left_cam_info_msg.D.resize(5);
    _left_cam_info_msg.D[0] = params->LeftCam.disto[0];
    _left_cam_info_msg.D[1] = params->LeftCam.disto[1];
    _left_cam_info_msg.D[2] = params->LeftCam.disto[2];
    _left_cam_info_msg.D[3] = params->LeftCam.disto[3];
    _left_cam_info_msg.D[4] = params->LeftCam.disto[4];
    _left_cam_info_msg.K.fill( 0.0 );
    _left_cam_info_msg.K[0] = params->LeftCam.fx;
    _left_cam_info_msg.K[2] = params->LeftCam.cx;
    _left_cam_info_msg.K[4] = params->LeftCam.fy;
    _left_cam_info_msg.K[5] = params->LeftCam.cy;
    _left_cam_info_msg.K[8] = 1.0;

    _left_cam_info_msg.R.fill( 0.0 );

    _left_cam_info_msg.P.fill( 0.0 );
    _left_cam_info_msg.P[0] = params->LeftCam.fx;
    _left_cam_info_msg.P[2] = params->LeftCam.cx;
    _left_cam_info_msg.P[5] = params->LeftCam.fy;
    _left_cam_info_msg.P[6] = params->LeftCam.cy;
    _left_cam_info_msg.P[10] = 1.0;

    _left_cam_info_msg.width = _zed_camera->getImageSize().width;
    _left_cam_info_msg.height = _zed_camera->getImageSize().height;

    _right_cam_info_msg.D.resize(5);
    _right_cam_info_msg.D[0] = params->RightCam.disto[0];
    _right_cam_info_msg.D[1] = params->RightCam.disto[1];
    _right_cam_info_msg.D[2] = params->RightCam.disto[2];
    _right_cam_info_msg.D[3] = params->RightCam.disto[3];
    _right_cam_info_msg.D[4] = params->RightCam.disto[4];
    _right_cam_info_msg.K.fill( 0.0 );
    _right_cam_info_msg.K[0] = params->RightCam.fx;
    _right_cam_info_msg.K[2] = params->RightCam.cx;
    _right_cam_info_msg.K[4] = params->RightCam.fy;
    _right_cam_info_msg.K[5] = params->RightCam.cy;
    _right_cam_info_msg.K[8] = 1.0;

    _right_cam_info_msg.R.fill( 0.0 ); // TODO use "params->convergence" data???

    _right_cam_info_msg.P.fill( 0.0 );
    _right_cam_info_msg.P[0] = params->RightCam.fx;
    _right_cam_info_msg.P[2] = params->RightCam.cx;
    _right_cam_info_msg.P[3] = -params->RightCam.fx * params->baseline; // Tx = -Fx * Baseline
    _right_cam_info_msg.P[5] = params->RightCam.fy;
    _right_cam_info_msg.P[6] = params->RightCam.cy;
    _right_cam_info_msg.P[10] = 1.0;

    _right_cam_info_msg.width = _zed_camera->getImageSize().width;
    _right_cam_info_msg.height = _zed_camera->getImageSize().height;
}

void* ZedDriver::run()
{
    if( !_zed_camera )
    {
        ROS_ERROR_STREAM( "The camera is not initialized.");
        return 0;
    }

    _stopping = false;
    int frameCnt = 0;

    zed::SENSING_MODE dm_type = zed::RAW;

    zed::Mat tmpMat;
    //zed::Mat depth, imLeft, imRight, disparity, confidence;

    // >>>>> Camera information
    zed::StereoParameters* params = _zed_camera->getParameters();
    initCamInfo(params);
    // <<<<< Camera information

    std_msgs::Header leftMsgHeader;
    std_msgs::Header rightMsgHeader;
    sensor_msgs::Image leftImgMsg;
    sensor_msgs::Image rightImgMsg;

    stereo_msgs::DisparityImage dispMsg;

    while( 1 )
    {
        if( _stopping )
        {
            break;
        }

        if( !_zed_camera )
        {
            ROS_ERROR_STREAM( "The camera is not initialized or has been stopped. Closing");
            break;
        }

        ros::spinOnce(); // Processing ROS messages

        // ZED Grabbing
        if( !_zed_camera->grab(dm_type,_enable_depth_confidence,_enable_depth_confidence) )
            continue;

        ros::Time now = ros::Time::now();
        frameCnt++;


        if( _enable_depth_confidence )
        {
            // TODO verify the buffer overwriting as reported on DOCS
            // file:///usr/local/zed/doc/API/classsl_1_1zed_1_1Camera.html#a7ae4783e231502e7681890636e24e49c
            tmpMat = _zed_camera->retrieveMeasure(zed::MEASURE::DEPTH);
            //            if( depth.height != tmpMat.height || depth.width!=tmpMat.width )
            //                depth.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( depth.data, tmpMat.data, tmpMat.getDataSize() );

            tmpMat = _zed_camera->retrieveMeasure(zed::MEASURE::DISPARITY);
            //            if( disparity.height != tmpMat.height || disparity.width!=tmpMat.width )
            //                disparity.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( disparity.data, tmpMat.data, tmpMat.getDataSize() );

            dispMsg.header.stamp = now;
            dispMsg.header.seq = frameCnt;
            dispMsg.header.frame_id = "disp";

            dispMsg.f = params->LeftCam.fx;
            dispMsg.valid_window.x_offset = 0;
            dispMsg.valid_window.y_offset = 0;
            dispMsg.valid_window.width = tmpMat.width;
            dispMsg.valid_window.height = tmpMat.height;
            dispMsg.min_disparity = 0.0f;
            dispMsg.max_disparity = 256.0f;
            
            dispMsg.image.header = dispMsg.header;
            dispMsg.image.width = tmpMat.width;
            dispMsg.image.height = tmpMat.height;

            int pixSize;
            if(tmpMat.data_type == zed::DATA_TYPE::FLOAT)
            {
                dispMsg.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                pixSize = sizeof(float);
            }
            else if(tmpMat.data_type == zed::DATA_TYPE::UCHAR)
            {
                dispMsg.image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
                pixSize = sizeof(uint8_t);
            }


            dispMsg.image.step = tmpMat.width * pixSize * tmpMat.channels;

            int imgSize = dispMsg.image.height * dispMsg.image.step;
            dispMsg.image.data.resize( imgSize );

            memcpy( (float*)(&dispMsg.image.data[0]), (float*)(&tmpMat.data[0]), imgSize );

            if(_disparity_pub.getNumSubscribers()>0)
                _disparity_pub.publish( dispMsg );

            tmpMat = _zed_camera->retrieveMeasure(zed::MEASURE::CONFIDENCE);
            //            if( confidence.height != tmpMat.height || confidence.width!=tmpMat.width )
            //                confidence.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( confidence.data, tmpMat.data, tmpMat.getDataSize() );
        }

        if( _enable_rgb )
        {
            // TODO verify the buffer overwriting as reported on DOCS
            // file:///usr/local/zed/doc/API/classsl_1_1zed_1_1Camera.html#a7ae4783e231502e7681890636e24e49c

            // >>>>> Left Image
            tmpMat = _zed_camera->retrieveImage(zed::SIDE::LEFT);
            //            if( imLeft.height != tmpMat.height || imLeft.width!=tmpMat.width )
            //                imLeft.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( imLeft.data, tmpMat.data, tmpMat.getDataSize() );

            leftMsgHeader.stamp = now;
            leftMsgHeader.seq = frameCnt;
            leftMsgHeader.frame_id = "left";

            leftImgMsg.width = tmpMat.width;
            leftImgMsg.height = tmpMat.height;
            if(tmpMat.channels==3)
                leftImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
            else if(tmpMat.channels==4)
                leftImgMsg.encoding = sensor_msgs::image_encodings::BGRA8;

            leftImgMsg.step = tmpMat.width * sizeof(uint8_t) * tmpMat.channels;

            int imgSize = leftImgMsg.height * leftImgMsg.step;
            leftImgMsg.data.resize( imgSize );

            memcpy( (uint8_t*)(&leftImgMsg.data[0]), (uint8_t*)(&tmpMat.data[0]), imgSize );

            _left_cam_info_msg.header = leftMsgHeader;
            // <<<<< Left Image

            // >>>>> Right Image
            tmpMat = _zed_camera->retrieveImage(zed::SIDE::RIGHT);
            //            if( imRight.height != tmpMat.height || imRight.width!=tmpMat.width )
            //                imRight.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( imRight.data, tmpMat.data, tmpMat.getDataSize() );

            rightMsgHeader.stamp = now;
            rightMsgHeader.seq = frameCnt;
            rightMsgHeader.frame_id = "right";

            rightImgMsg.width = tmpMat.width;
            rightImgMsg.height = tmpMat.height;
            if(tmpMat.channels==3)
                rightImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
            else if(tmpMat.channels==4)
                rightImgMsg.encoding = sensor_msgs::image_encodings::BGRA8;

            rightImgMsg.step = tmpMat.width * sizeof(uint8_t) * tmpMat.channels;

            imgSize = rightImgMsg.height * rightImgMsg.step;
            rightImgMsg.data.resize( imgSize );

            memcpy( (uint8_t*)(&rightImgMsg.data[0]), (uint8_t*)(&tmpMat.data[0]), imgSize );

            _right_cam_info_msg.header = rightMsgHeader;
            // <<<<< Right Image

            if( _rgb_left_pub.getNumSubscribers()>0 )
                _rgb_left_pub.publish( leftImgMsg, _left_cam_info_msg );


            if( _rgb_right_pub.getNumSubscribers()>0 )
                _rgb_right_pub.publish( rightImgMsg, _right_cam_info_msg );

        }

        // TODO Create the pointcloud
    }

    //depth.deallocate();
    //imLeft.deallocate();
    //imRight.deallocate();
    //disparity.deallocate();
    //confidence.deallocate();

    ROS_INFO_STREAM( "ZED camera stopped... ");

    releaseCamera();

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

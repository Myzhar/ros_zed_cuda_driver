#include "zed_camera_driver.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <stereo_msgs/DisparityImage.h>

#include <string>

bool ZedDriver::_stopping = false;

using namespace std;

ZedDriver::ZedDriver()
    : _nhPriv("~")
    , _initialized(false)
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
        _rgb_left_pub = _rgb_left_ImgTr.advertiseCamera("stereo/left/image_rect", 1, false);
        _rgb_right_pub = _rgb_right_ImgTr.advertiseCamera("stereo/right/image_rect", 1, false);
    }

    if( _enable_depth )
        _depth_pub = _depth_ImgTr.advertiseCamera("stereo/depth_norm", 1,false);

    if( _enable_disp )
        _disparity_pub = _nh.advertise<stereo_msgs::DisparityImage>("stereo/disparity", 1 );

    if( _enable_norm_confidence )
        _norm_confidence_pub = _confidence_ImgTr.advertiseCamera("stereo/confidence_norm", 1, false);


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

    _zed_camera = new zed::Camera( _resol, (float)_fps );

    zed::MODE mode = (_enable_norm_confidence||_enable_depth||_enable_disp||_enable_ptcloud||_enable_registered)?(zed::PERFORMANCE):(zed::NONE);

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
    _left_cam_info_msg.D[0] = 0.0; //params->LeftCam.disto[0]; Images are rectified!!!
    _left_cam_info_msg.D[1] = 0.0; //params->LeftCam.disto[1]; Images are rectified!!!
    _left_cam_info_msg.D[2] = 0.0; //params->LeftCam.disto[2]; Images are rectified!!!
    _left_cam_info_msg.D[3] = 0.0; //params->LeftCam.disto[3]; Images are rectified!!!
    _left_cam_info_msg.D[4] = 0.0; //params->LeftCam.disto[4]; Images are rectified!!!
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
    _right_cam_info_msg.D[0] = 0.0; //params->RightCam.disto[0]; Images are rectified!!!
    _right_cam_info_msg.D[1] = 0.0; //params->RightCam.disto[1]; Images are rectified!!!
    _right_cam_info_msg.D[2] = 0.0; //params->RightCam.disto[2]; Images are rectified!!!
    _right_cam_info_msg.D[3] = 0.0; //params->RightCam.disto[3]; Images are rectified!!!
    _right_cam_info_msg.D[4] = 0.0; //params->RightCam.disto[4]; Images are rectified!!!
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

    _depth_cam_info_msg.D.resize(5);
    _depth_cam_info_msg.D[0] = 0.0; //params->LeftCam.disto[0]; Images are rectified!!!
    _depth_cam_info_msg.D[1] = 0.0; //params->LeftCam.disto[1]; Images are rectified!!!
    _depth_cam_info_msg.D[2] = 0.0; //params->LeftCam.disto[2]; Images are rectified!!!
    _depth_cam_info_msg.D[3] = 0.0; //params->LeftCam.disto[3]; Images are rectified!!!
    _depth_cam_info_msg.D[4] = 0.0; //params->LeftCam.disto[4]; Images are rectified!!!
    _depth_cam_info_msg.K.fill( 0.0 );
    _depth_cam_info_msg.K[0] = params->LeftCam.fx;
    _depth_cam_info_msg.K[2] = params->LeftCam.cx;
    _depth_cam_info_msg.K[4] = params->LeftCam.fy;
    _depth_cam_info_msg.K[5] = params->LeftCam.cy;
    _depth_cam_info_msg.K[8] = 1.0;

    _depth_cam_info_msg.R.fill( 0.0 );

    _depth_cam_info_msg.P.fill( 0.0 );
    _depth_cam_info_msg.P[0] = params->LeftCam.fx;
    _depth_cam_info_msg.P[2] = params->LeftCam.cx;
    _depth_cam_info_msg.P[5] = params->LeftCam.fy;
    _depth_cam_info_msg.P[6] = params->LeftCam.cy;
    _depth_cam_info_msg.P[10] = 1.0;

    _depth_cam_info_msg.width = _zed_camera->getImageSize().width;
    _depth_cam_info_msg.height = _zed_camera->getImageSize().height;
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
    std_msgs::Header depthMsgHeader;
    sensor_msgs::Image leftImgMsg;
    sensor_msgs::Image rightImgMsg;
    sensor_msgs::Image depthImgMsg;

    stereo_msgs::DisparityImage dispMsg;

    while( 1/*ros::ok()*/ )
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

        // >>>>> Acquiring
        ros::Time acqStart = ros::Time::now();

        bool enableMeasure = _enable_norm_confidence||_enable_depth||_enable_disp||_enable_ptcloud||_enable_registered;

        _zed_camera->setConfidenceThreshold( _conf_thresh );

        // ZED Grabbing
        if( !_zed_camera->grab(dm_type, enableMeasure, enableMeasure) )
            continue;

        ROS_INFO_STREAM( "Grabbing: " << (ros::Time::now() - acqStart)*1000.0 << " msec" );

        // <<<<< Acquiring

        ros::Time msg_timeStamp = ros::Time::now();
        frameCnt++;

        // >>>>> Depth Map
        if( _enable_depth )
        {
            ros::Time depthStart = ros::Time::now();

            //tmpMat = _zed_camera->retrieveMeasure_gpu( zed::MEASURE::DEPTH );
            tmpMat = _zed_camera->retrieveMeasure( zed::MEASURE::DEPTH );

            depthMsgHeader.stamp = msg_timeStamp;
            depthMsgHeader.seq = frameCnt;
            depthMsgHeader.frame_id = "depth";

            depthImgMsg.width = tmpMat.width;
            depthImgMsg.height = tmpMat.height;

            int pixSize;
            if(tmpMat.data_type == zed::DATA_TYPE::FLOAT)
            {
                depthImgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                pixSize = sizeof(float);
            }
            else if(tmpMat.data_type == zed::DATA_TYPE::UCHAR)
            {
                depthImgMsg.encoding = sensor_msgs::image_encodings::RGBA8;
                pixSize = sizeof(uint8_t);
            }

            depthImgMsg.step = tmpMat.width * pixSize * tmpMat.channels;

            int imgSize = depthImgMsg.height * depthImgMsg.step;
            depthImgMsg.data.resize( imgSize );

            //cudaMemcpy( (float*)(&depthImgMsg.data[0]), (float*)(&tmpMat.data[0]), imgSize, cudaMemcpyDeviceToHost );
            //cudaDeviceSynchronize();
            memcpy( (float*)(&depthImgMsg.data[0]), (float*)(&tmpMat.data[0]), imgSize );

            _depth_cam_info_msg.header = depthMsgHeader;

            if(_depth_pub.getNumSubscribers()>0)
                _depth_pub.publish( depthImgMsg, _depth_cam_info_msg );

            ROS_INFO_STREAM( "Depth: " << (ros::Time::now() - depthStart)*1000.0 << " msec" );
        }
        // <<<<< Depth Map

        // >>>>> Disparity Map
        if(_enable_disp)
        {
            ros::Time dispStart = ros::Time::now();

            //tmpMat = _zed_camera->retrieveMeasure_gpu(zed::MEASURE::DISPARITY);
            tmpMat = _zed_camera->retrieveMeasure(zed::MEASURE::DISPARITY);
            //            if( disparity.height != tmpMat.height || disparity.width!=tmpMat.width )
            //                disparity.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( disparity.data, tmpMat.data, tmpMat.getDataSize() );

            dispMsg.header.stamp = msg_timeStamp;
            dispMsg.header.seq = frameCnt;
            dispMsg.header.frame_id = "disp";

            dispMsg.f = params->LeftCam.fx;
            dispMsg.valid_window.x_offset = 0;
            dispMsg.valid_window.y_offset = 0;
            dispMsg.valid_window.width = tmpMat.width;
            dispMsg.valid_window.height = tmpMat.height;
            dispMsg.min_disparity = -tmpMat.width/8;    // According to StereoLabs information
            dispMsg.max_disparity = 0.0f;               // According to StereoLabs information

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

            //cudaMemcpy((float*)(&dispMsg.image.data[0]), (float*)(&tmpMat.data[0]), imgSize, cudaMemcpyDeviceToHost);
            //cudaDeviceSynchronize();
            memcpy( (float*)(&dispMsg.image.data[0]), (float*)(&tmpMat.data[0]), imgSize );

            if(_disparity_pub.getNumSubscribers()>0)
                _disparity_pub.publish( dispMsg );

            ROS_INFO_STREAM( "Disparity: " << (ros::Time::now() - dispStart)*1000.0 << " msec" );
        }
        // <<<<< Disparity Map normalized

        if( _enable_norm_confidence )
        {
            ros::Time confStart = ros::Time::now();

            //tmpMat = _zed_camera->retrieveMeasure_gpu(zed::MEASURE::CONFIDENCE);
            //            if( confidence.height != tmpMat.height || confidence.width!=tmpMat.width )
            //                confidence.allocate_cpu( tmpMat.width, tmpMat.height, tmpMat.channels, tmpMat.data_type );
            //            memcpy( confidence.data, tmpMat.data, tmpMat.getDataSize() );

            ROS_INFO_STREAM( "Confidence: " << (ros::Time::now() - confStart)*1000.0 << " msec" );
        }

        if( _enable_rgb )
        {
            ros::Time rgbStart = ros::Time::now();

            // >>>>> Left Image
            //tmpMat = _zed_camera->retrieveImage_gpu(zed::SIDE::LEFT);
            tmpMat = _zed_camera->retrieveImage(zed::SIDE::LEFT);

            leftMsgHeader.stamp = msg_timeStamp;
            leftMsgHeader.seq = frameCnt;
            leftMsgHeader.frame_id = "left";

            leftImgMsg.width = tmpMat.width;
            leftImgMsg.height = tmpMat.height;
            if(tmpMat.channels==1)
                leftImgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            else if(tmpMat.channels==3)
                leftImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
            else if(tmpMat.channels==4)
                leftImgMsg.encoding = sensor_msgs::image_encodings::BGRA8;

            leftImgMsg.step = tmpMat.width * sizeof(uint8_t) * tmpMat.channels;

            int imgSize = leftImgMsg.height * leftImgMsg.step;
            leftImgMsg.data.resize( imgSize );

            //cudaMemcpy( (uint8_t*)(&leftImgMsg.data[0]), (uint8_t*)(&tmpMat.data[0]), imgSize, cudaMemcpyDeviceToHost );
            //cudaDeviceSynchronize();
            memcpy( (uint8_t*)(&leftImgMsg.data[0]), (uint8_t*)(&tmpMat.data[0]), imgSize );

            _left_cam_info_msg.header = leftMsgHeader;
            // <<<<< Left Image

            // >>>>> Right Image
            //tmpMat = _zed_camera->retrieveImage_gpu(zed::SIDE::RIGHT);
            tmpMat = _zed_camera->retrieveImage(zed::SIDE::RIGHT);

            rightMsgHeader.stamp = msg_timeStamp;
            rightMsgHeader.seq = frameCnt;
            rightMsgHeader.frame_id = "right";

            rightImgMsg.width = tmpMat.width;
            rightImgMsg.height = tmpMat.height;
            if(tmpMat.channels==1)
                rightImgMsg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            else if(tmpMat.channels==3)
                rightImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
            else if(tmpMat.channels==4)
                rightImgMsg.encoding = sensor_msgs::image_encodings::BGRA8;

            rightImgMsg.step = tmpMat.width * sizeof(uint8_t) * tmpMat.channels;

            imgSize = rightImgMsg.height * rightImgMsg.step;
            rightImgMsg.data.resize( imgSize );

            //cudaMemcpy( (uint8_t*)(&rightImgMsg.data[0]), (uint8_t*)(&tmpMat.data[0]), imgSize, cudaMemcpyDeviceToHost );
            //cudaDeviceSynchronize();
            memcpy( (uint8_t*)(&rightImgMsg.data[0]), (uint8_t*)(&tmpMat.data[0]), imgSize );

            _right_cam_info_msg.header = rightMsgHeader;
            // <<<<< Right Image

            if( _rgb_left_pub.getNumSubscribers()>0 )
                _rgb_left_pub.publish( leftImgMsg, _left_cam_info_msg );


            if( _rgb_right_pub.getNumSubscribers()>0 )
                _rgb_right_pub.publish( rightImgMsg, _right_cam_info_msg );

            ROS_INFO_STREAM( "RGB: " << (ros::Time::now() - rgbStart)*1000.0 << " msec" );
        }

        // TODO Create the pointcloud

        ros::spinOnce(); // Processing ROS messages
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

#define PAR_RESOL                   "resolution"
#define PAR_FPS                     "fps"
#define PAR_PUB_TF                  "publish_tf"
#define PAR_ENABLE_RGB              "enable_rgb"
#define PAR_ENABLE_PTCLOUD          "enable_ptcloud"
#define PAR_ENABLE_REGISTERED       "enable_rgb_ptcloud"
#define PAR_enable_depth            "enable_depth"
#define PAR_ENABLE_NORM_DISP        "enable_norm_disparity"
#define PAR_ENABLE_NORM_CONF        "enable_norm_confidence"
#define PAR_CONF_THRESH             "confidence_thresh"

void ZedDriver::loadParams()
{
    if( _nhPriv.hasParam( PAR_RESOL ) )
    {
        string resolStr;
        _nhPriv.getParam( PAR_RESOL, resolStr );

        if( resolStr.compare( "VGA")==0 )
        {
            _resol = zed::VGA;
            ROS_INFO_STREAM( "Resolution: " << "VGA" );
        }
        else if( resolStr.compare( "HD2K")==0 )
        {
            _resol = zed::HD2K;
            ROS_INFO_STREAM( "Resolution: " << "HD2K" );
        }
        else if( resolStr.compare( "HD1080")==0 )
        {
            _resol = zed::HD1080;
            ROS_INFO_STREAM( "Resolution: " << "HD1080" );
        }
        else if( resolStr.compare( "HD720")==0 )
        {
            _resol = zed::HD720;
            ROS_INFO_STREAM( "Resolution: " << "HD720" );
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
        _nhPriv.setParam( PAR_PUB_TF, "VGA" );

        ROS_INFO_STREAM( "Resolution: " << "VGA" );
    }

    if( _nhPriv.hasParam( PAR_FPS ) )
    {
        _nhPriv.getParam( PAR_FPS, _fps );
    }
    else
    {
        _fps = 15.0f;
        _nhPriv.setParam( PAR_FPS, _fps );
    }

    ROS_INFO_STREAM( "FPS: " << _fps );

    if( _nhPriv.hasParam( PAR_ENABLE_RGB ) )
    {
        _nhPriv.getParam( PAR_ENABLE_RGB, _enable_rgb );
    }
    else
    {
        _enable_rgb = true;
        _nhPriv.setParam( PAR_ENABLE_RGB, _enable_rgb );
    }

    ROS_INFO_STREAM( "RGB: " << (_enable_rgb?"Enabled":"Disabled") );

    if( _nhPriv.hasParam( PAR_ENABLE_PTCLOUD ) )
    {
        _nhPriv.getParam( PAR_ENABLE_PTCLOUD, _enable_ptcloud );
    }
    else
    {
        _enable_ptcloud = false;
        _nhPriv.setParam( PAR_ENABLE_RGB, _enable_ptcloud );
    }

    ROS_INFO_STREAM( "Pointcloud: " << (_enable_ptcloud?"Enabled":"Disabled") );

    if( _nhPriv.hasParam( PAR_ENABLE_REGISTERED ) )
    {
        _nhPriv.getParam( PAR_ENABLE_REGISTERED, _enable_registered );
    }
    else
    {
        _enable_registered = false;
        _nhPriv.setParam( PAR_ENABLE_REGISTERED, _enable_registered );
    }

    ROS_INFO_STREAM( "Registered Pointcloud: " << (_enable_registered?"Enabled":"Disabled") );

    if( _nhPriv.hasParam( PAR_enable_depth ) )
    {
        _nhPriv.getParam( PAR_enable_depth, _enable_depth );
    }
    else
    {
        _enable_depth = false;
        _nhPriv.setParam( PAR_enable_depth, _enable_depth );
    }

    ROS_INFO_STREAM( "Normalized Depth Map: " << (_enable_depth?"Enabled":"Disabled") );

    if( _nhPriv.hasParam( PAR_ENABLE_NORM_DISP ) )
    {
        _nhPriv.getParam( PAR_ENABLE_NORM_DISP, _enable_disp );
    }
    else
    {
        _enable_disp = false;
        _nhPriv.setParam( PAR_ENABLE_NORM_DISP, _enable_disp );
    }

    ROS_INFO_STREAM( "Normalized Disparity Map: " << (_enable_disp?"Enabled":"Disabled") );


    if( _nhPriv.hasParam( PAR_ENABLE_NORM_CONF ) )
    {
        _nhPriv.getParam( PAR_ENABLE_NORM_CONF, _enable_norm_confidence );
    }
    else
    {
        _enable_norm_confidence = false;
        _nhPriv.setParam( PAR_ENABLE_NORM_CONF, _enable_norm_confidence );
    }

    ROS_INFO_STREAM( "Normalized Confidence Map: " << (_enable_norm_confidence?"Enabled":"Disabled") );

    if( _nhPriv.hasParam( PAR_CONF_THRESH ) )
    {
        _nhPriv.getParam( PAR_CONF_THRESH, _conf_thresh );
    }
    else
    {
        _conf_thresh = 60;
        _nhPriv.setParam( PAR_CONF_THRESH, _conf_thresh );
    }

    ROS_INFO_STREAM( "Disparity Confidence Threshold: " << _conf_thresh );

}

void  ZedDriver::start(void)
{
    int       result;
    result = pthread_create(&_threadId, 0, ZedDriver::callRunFunction, this);
    if (result == 0)
        pthread_detach(_threadId);
}

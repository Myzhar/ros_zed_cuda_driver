#include <stdlib.h>
#include <ros/ros.h>
#include "zed_camera_driver.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_camera_node");

    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("       StereoLabs ZED node         \r");
    ROS_INFO_STREAM("-----------------------------------\r");

    ZedDriver cameraDriver;

    if(cameraDriver.init() )
        cameraDriver.start();
    else
        return(EXIT_FAILURE);

    ros::spin();

    ROS_INFO_STREAM( "... shutting down complete." );

    return(EXIT_SUCCESS);
}

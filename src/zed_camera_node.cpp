#include <stdlib.h>
#include <ros/ros.h>
#include "zed_camera_driver.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_camera_node");

    ROS_INFO_STREAM("-----------------------------------");
    ROS_INFO_STREAM("       StereoLabs ZED node         ");
    ROS_INFO_STREAM("-----------------------------------");

    ZedDriver cameraDriver;

    bool ready = cameraDriver.init();

    if( ready )
        cameraDriver.start();
    else
        return(EXIT_FAILURE);

    while( ros::ok() )
    {
        ros::Duration(1).sleep();

        ros::spinOnce();
    }

    ROS_INFO_STREAM( "... shutting down complete." );

    return(EXIT_SUCCESS);
}

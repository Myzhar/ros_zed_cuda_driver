cmake_minimum_required(VERSION 2.8.7)

project(ros_zed_camera)

###############################################################################
## Note by Myzhar                                                            ##
## ===============                                                           ##
##                                                                           ##
## You need to install the official StereoLabs ZED SDK from the website:     ##
## https://www.stereolabs.com/developers/                                    ##
##                                                                           ##
## For ARM based system you must request the SDK to StereoLabs Support       ##
##                                                                           ##
###############################################################################

###############################################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
###############################################################################

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    image_transport
)

###############################################################################
# INCLUDES

set(zed_INC_DIRS
    src/
    src/driver/include/
    /usr/local/zed/include
)

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${zed_INC_DIRS}
)
###############################################################################

###############################################################################
# SOURCES
set(zed_SRC
    src/zed_camera_node.cpp
    src/driver/src/zed_camera_driver.cpp
)
###############################################################################

###############################################################################
# EXECUTABLE
add_executable(
  zed_camera_node
  ${zed_SRC}
)

find_library(zed_LIBRARY sl_calibration sl_depthcore sl_zed cudpp cudpp_hash /usr/local/zed/lib)

target_link_libraries(
        zed_camera_node
        ${catkin_LIBRARIES} ${zed_LIBRARY}
    )
###############################################################################


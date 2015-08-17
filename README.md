# ros-zed-cuda-driver
Driver for the ZED Stereo Camera by StereoLabs. The driver is based on ZED SDK, so needs CUDA to run

The StereoLabs SDK performs measurament calculation directly after image acquisition using CUDA elaboration power, so you don't need to make your own registration and depth map estimation. You can use directly "raw" depth map, "pointcloud" and "registered pointcloud" for your navigation algorithms.

If you system is not Nvidia CUDA powered, you can get registered images and calculate disparity and depth maps using the "standard OpenCV method" after calibration.

## Instructions
Execute the node running 
```
roslaunch ros_zed_cuda_driver zed.launch
```

Modify the file "launch/zed.launch" accoding to your configuration


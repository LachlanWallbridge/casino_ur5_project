#!/bin/bash
 
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p enable_color:=true \
  -p rgb_camera.color_profile:=1920x1080x6 \
  -p enable_depth:=true \
  -p depth_module.depth_profile:=848x480x6 \
  -p enable_sync:=true \
  -p align_depth.enable:=true\
  -p pointcloud.enable:=true 
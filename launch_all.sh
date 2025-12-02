#!/bin/bash

# ============================================================
#   MASTER LAUNCH SCRIPT FOR THE ENTIRE MTRN4231 ROBOT STACK
# ============================================================

WS=~/MTRN4231_Project
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

echo "=============================================="
echo "Building workspace..."
echo "=============================================="

cd $WS
colcon build --symlink-install

echo "Sourcing workspace..."
source $ROS_SETUP
source $WS_SETUP

sleep 1

# ------------------------------------------------------------
# Function to open a sourced terminal
# ------------------------------------------------------------
open_term() {
    gnome-terminal --title="$1" -- bash -c "
        echo 'Launching $1...';
        source $ROS_SETUP;
        source $WS_SETUP;
        ${@:2};
        exec bash"
}

# ------------------------------------------------------------
# 1. RealSense Camera
# ------------------------------------------------------------
open_term "RealSense Camera" \
    "ros2 run realsense2_camera realsense2_camera_node \
        --ros-args \
        -p enable_color:=true \
        -p rgb_camera.color_profile:=1920x1080x6 \
        -p enable_depth:=true \
        -p depth_module.depth_profile:=848x480x6 \
        -p enable_sync:=true \
        -p align_depth.enable:=true \
        -p pointcloud.enable:=true"

sleep 5

# ------------------------------------------------------------
# 2. UR Driver Server
# ------------------------------------------------------------
open_term "UR Driver Server" \
    "ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur5e robot_ip:=192.168.0.100 \
        use_fake_hardware:=false launch_rviz:=false"

sleep 10

# ------------------------------------------------------------
# 3. MoveIt Server
# ------------------------------------------------------------
open_term "MoveIt Server" \
    'ros2 launch ur_moveit_config ur_moveit.launch.py \
        robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true'

sleep 4

# ------------------------------------------------------------
# 4. Gripper Visualiser
# ------------------------------------------------------------
open_term "Gripper Visualiser" \
    "ros2 launch linear_gripper_visualiser auxiliary.launch.py"

sleep 3

# ------------------------------------------------------------
# 5. Perception Stack
# ------------------------------------------------------------
open_term "Perception" \
    "ros2 launch perception_cv perception.launch.py"

sleep 3

# ------------------------------------------------------------
# 6. Rosbridge Server (WebSocket)
# ------------------------------------------------------------
open_term "Rosbridge" \
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

sleep 3

# ------------------------------------------------------------
# 7. Brain Node
# ------------------------------------------------------------
open_term "Brain Node" \
    "ros2 launch brain brain.launch.py"

sleep 3

# ------------------------------------------------------------
# 8. Backend Server
# ------------------------------------------------------------
open_term "Backend" \
    "cd $WS/backend && uvicorn api:app --reload --port 8000"

sleep 2

# ------------------------------------------------------------
# 9. Frontend UI
# ------------------------------------------------------------
open_term "Frontend" \
    "cd $WS/frontend && npm start"

echo "=============================================="
echo " ALL SYSTEMS LAUNCHED SUCCESSFULLY "
echo "=============================================="

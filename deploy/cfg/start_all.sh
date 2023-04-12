#!/bin/bash

xfce4-terminal  -T master -e "bash -iC 'echo start!; $SHELL'" \
        --tab   -T realsense_camera -e "bash -ic 'sleep 5; ros2 run realsense2_camera realsense2_camera_node; $SHELL'" \
        #--tab   -T mavros -e "bash -ic 'sleep 5; roslaunch mavros px4.launch; $SHELL'" \
        --tab   -T vins_fusion -e "bash -ic 'sleep 35; cd /catkin_ws/src/navigation/vins_fusion/vins/config/PX4_realsenseD435i; ros2 run vins vins_node px4_realsense_stereo_imu_config.yaml; $SHELL'" \
        #--tab   -T vins_rviz -e "bash -ic 'sleep 15; roslaunch vins vins_rviz.launch; $SHELL'" \
        #--tab   -T happymoon_quad_control -e "bash -ic 'sleep 10; roslaunch happymoon_quad_control happymoon_control.launch; $SHELL'" \
#!/bin/bash

xfce4-terminal  -T master -e "bash -iC 'echo start!; $SHELL'" \
        --tab   -T ros_core -e "bash -ic 'roscore; $SHELL'" \
        --tab   -T realsense_camera -e "bash -ic 'sleep 5; roslaunch realsense2_camera rs_camera.launch; $SHELL'" \
        --tab   -T mavros -e "bash -ic 'sleep 5; roslaunch mavros px4.launch; $SHELL'" \
        --tab   -T vins_fusion -e "bash -ic 'sleep 35; roslaunch vins vins_fusion.launch; $SHELL'" \
        --tab   -T vins_rviz -e "bash -ic 'sleep 15; roslaunch vins vins_rviz.launch; $SHELL'" \
        #--tab   -T happymoon_quad_control -e "bash -ic 'sleep 10; roslaunch happymoon_quad_control happymoon_control.launch; $SHELL'" \
#!/bin/bash

xfce4-terminal  -T master -e "bash -iC 'echo start!; $SHELL'" \
        --tab   -T ros_core -e "bash -ic 'roscore; $SHELL'" \
        --tab   -T realsense_camera -e "bash -ic 'sleep 3; roslaunch realsense2_camera realsense_camera.launch; $SHELL'" \
        --tab   -T djiros -e "bash -ic 'sleep 25; roslaunch djiros djiros.launch; $SHELL'" \
        --tab   -T nlink_parser -e "bash -ic 'sleep 10; roslaunch nlink_parser tofsense.launch; $SHELL'" \
        --tab   -T happymoon_quad_control -e "bash -ic 'sleep 10; roslaunch happymoon_quad_control happymoon_control.launch; $SHELL'" \
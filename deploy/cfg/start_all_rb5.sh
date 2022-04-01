#!/bin/bash

roslaunch realsense2_camera realsense_camera.launch &

sleep 50

echo "realsense_camera starting success!"

roslaunch dji_sdk sdk.launch &

sleep 30

echo "dji_sdk starting success!"

roslaunch vins vins_fusion.launch &

sleep 15

echo "vins starting success!"

# roslaunch happymoon_quad_control happymoon_control.launch &

# sleep 5

# echo "happymoon_quad_control starting success!"




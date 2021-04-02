#!/bin/bash

roslaunch realsense2_camera realsense_camera.launch &

sleep 25

echo "realsense_camera starting success!"

roslaunch djiros djiros.launch &

sleep 15

echo "djiros starting success!"

roslaunch nlink_parser tofsense.launch &

sleep 10

echo "tofsense starting success!"

roslaunch happymoon_quad_control happymoon_control.launch &

sleep 5

echo "happymoon_quad_control starting success!"




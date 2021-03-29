#!/bin/bash
roslaunch realsense2_camera realsense_camera.launch &

sleep 25

echo "realsense_camera starting success!"

roslaunch djiros djiros.launch &

sleep 10

echo "djiros starting success!"


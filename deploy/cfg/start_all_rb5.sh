#!/bin/bash

roslaunch realsense2_camera realsense_camera.launch &

sleep 40

echo "realsense_camera starting success!"

cd /home/rb5_quad/catkin_ws/src/quad_ros/driver/px4_driver/launch/

roslaunch px4.launch &

sleep 15

echo "px4 starting success!"

rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 &

sleep 5

roslaunch vins vins_fusion.launch &

sleep 15

echo "vins starting success!"

# roslaunch happymoon_quad_control happymoon_control.launch &

# sleep 5

# echo "happymoon_quad_control starting success!"




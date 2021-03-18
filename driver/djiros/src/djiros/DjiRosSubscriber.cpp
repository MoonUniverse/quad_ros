//
// Created by ltb on 6/8/17.
//

#include <djiros/DjiRos.h>

bool DjiRos::initSubscriber(ros::NodeHandle &nh) {
  ctrl_sub = nh.subscribe<sensor_msgs::Joy>(
      "ctrl", 10, boost::bind(&DjiRos::control_callback, this, _1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  //    gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
  //            "dji_sdk/gimbal_angle_cmd", 10,
  //            &DJISDKNode::gimbalAngleCtrlCallback, this);
  //    gimbal_speed_cmd_subscriber =
  //    nh.subscribe<geometry_msgs::Vector3Stamped>(
  //            "dji_sdk/gimbal_speed_cmd", 10,
  //            &DJISDKNode::gimbalSpeedCtrlCallback, this);

  return true;
};

void DjiRos::control_callback(const sensor_msgs::JoyConstPtr &pMsg) {
  if (pMsg->header.frame_id.compare("FRD") == 0) {
    last_ctrl_stamp = ros::Time::now();
    uint8_t flag = 0b00101010;

    DJI::OSDK::Control::CtrlData ctrl_data(flag, pMsg->axes[0], pMsg->axes[1],
                                           pMsg->axes[2], pMsg->axes[3]);

    vehicle->control->flightCtrl(ctrl_data);
  } else {
    ROS_ERROR("[djiros] input joy_msg.frame_id should be FRD!!!");
  }
}
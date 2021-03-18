#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include "control_data_arbiter.h"

namespace happymoon_control {

ControlDataArbiter::ControlDataArbiter() {
  for (int i = 0; i < MAX_CTRL_NUM; ++i) {
    ctrl_array[i].priority = i;
    ctrl_array[i].is_valid = false;
    ctrl_array[i].djictrl.pitch = 0.0;
    ctrl_array[i].djictrl.roll = 0.0;
    ctrl_array[i].djictrl.yawrate = 0.0;
    ctrl_array[i].djictrl.thrust = 0.0;
    ctrl_array[i].is_active = false;
    ctrl_array[i].timeout = 0.5;
    ctrl_array[i].last_rec_time = ros::Time::now();
  }
}

bool ControlDataArbiter::setCtrlByPriority(uint32_t priority,
                                           djiFlightControl *ctrl) {
  if ((priority >= MAX_CTRL_NUM) || (NULL == ctrl)) {
    ROS_INFO_NAMED("CtrlArbiter::setCtrlByPriority",
                   "Input Parameters invalid");
    return false;
  }
  ctrl_array[priority].djictrl.pitch = ctrl->pitch;
  ctrl_array[priority].djictrl.roll = ctrl->roll;
  ctrl_array[priority].djictrl.yawrate = ctrl->yawrate;
  ctrl_array[priority].djictrl.thrust = ctrl->thrust;

  ctrl_array[priority].last_rec_time = ros::Time::now();

  return true;
}

bool ControlDataArbiter::getHighestPriorityCtrl(uint32_t *priority,
                                                djiFlightControl *ctrl) {
  for (int idx = 0; idx < MAX_CTRL_NUM; ++idx) {
    ros::Time current_time = ros::Time::now();

    if (((current_time.toSec() - ctrl_array[idx].last_rec_time.toSec()) <
         ctrl_array[idx].timeout) &&
        (true == ctrl_array[idx].is_active)) {
      ctrl_array[idx].is_valid = true;
    } else {
      ctrl_array[idx].is_valid = false;
    }
  }

  for (int i = 0; i < MAX_CTRL_NUM; ++i) {
    if (ctrl_array[i].is_valid == false) {
      continue;
    }

    *priority = i;
    ctrl->pitch = ctrl_array[i].djictrl.pitch;
    ctrl->roll = ctrl_array[i].djictrl.roll;
    ctrl->yawrate = ctrl_array[i].djictrl.yawrate;
    ctrl->thrust = ctrl_array[i].djictrl.thrust;

    return true;
  }

  return false;
}

bool ControlDataArbiter::setActiveFlagByPriority(uint32_t priority,
                                                 bool is_active) {
  if (priority >= MAX_CTRL_NUM) {
    ROS_INFO_NAMED("ControlDataArbiter::setValidFlagByPriority",
                   "Input Parameters invalid");
    return false;
  }

  ctrl_array[priority].is_active = is_active;

  return true;
}
} // namespace happymoon_control

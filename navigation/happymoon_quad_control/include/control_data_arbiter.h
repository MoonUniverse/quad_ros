#ifndef CONTROL_DATA_ARBITER_H
#define CONTROL_DATA_ARBITER_H
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#define MAX_CTRL_NUM (5)
namespace happymoon_control {

typedef struct {
  float pitch;
  float roll;
  float yawrate;
  float thrust;
} djiFlightControl;

typedef struct {
  djiFlightControl djictrl;
  uint32_t priority;
  bool is_active;
  bool is_valid;
  ros::Time last_rec_time;
  double timeout;
} ctrlElement;

class ControlDataArbiter {
public:
  ControlDataArbiter();

  bool setCtrlByPriority(uint32_t priority, djiFlightControl *ctrl);

  bool getHighestPriorityCtrl(uint32_t *priority, djiFlightControl *ctrl);

  bool setActiveFlagByPriority(uint32_t priority, bool is_active);

private:
  ctrlElement ctrl_array[MAX_CTRL_NUM];
};
}; // namespace happymoon_control

#endif

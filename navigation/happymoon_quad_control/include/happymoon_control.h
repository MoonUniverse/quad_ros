#ifndef HAPPYMOON_CONTROL_H
#define HAPPYMOON_CONTROL_H

#include <atomic>
#include <list>
#include <mutex>
#include <thread>

// ROS includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <happymoon_quad_control/TofsenseFrame0.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ManualControl.h>

#include <eigen3/Eigen/Dense>

#include "control_data_arbiter.h"
#include "geometry_eigen_conversions.h"
#include "math_common.h"

using namespace std;

struct QuadStateEstimateData {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d roll_pitch_yaw;
  Eigen::Vector3d bodyrates; // Body rates are represented in body coordinates
};

struct QuadStateReferenceData {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d roll_pitch_yaw;
  double heading;
  double heading_rate;
  double heading_acceleration;
};

struct HappymoonReference {
  Eigen::Vector3d position;
  double heading;
  double heading_rate;
};

typedef enum {
  JOY_STICK_PRIO_IDX = 0,
  NAV_PRIO_IDX,
} enPriorityIdx;

struct PositionControllerParams {
  double kpxy; // [1/s^2]
  double kdxy; // [1/s]

  double kpz; // [1/s^2]
  double kdz; // [1/s]

  double krp;  // [1/s]
  double kyaw; // [1/s]

  double refVelXYKp;
  double refVelZKp;
  double refVelHeadingKp;
  double refVelRateheadingKp;

  double ref_vxy_error_max; //[m/s]
  double ref_vz_error_max;  //[m/s]
  double pxy_error_max;     // [m]
  double vxy_error_max;     // [m/s]
  double pz_error_max;      // [m]
  double vz_error_max;      // [m/s]
  double yaw_error_max;     // [rad]

  // Whether or not to compensate for aerodynamic effects
  double k_drag_x; // x-direction rotor drag coefficient
  double k_drag_y; // y-direction rotor drag coefficient
  double k_drag_z; // z-direction rotor drag coefficient
  // thrust correction coefficient due to body horizontal velocity
  double k_thrust_horz;
};

struct ControlCommand {
  ros::Time timestamp;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyangle;
  Eigen::Vector3d bodyrates;             // Body rates in body frame
  Eigen::Vector3d angular_accelerations; // Angular accelerations in body frame
  double collective_thrust;
};
namespace happymoon_control {
class HappyMoonControl {
public:
  HappyMoonControl();
  ~HappyMoonControl();

private:
  boost::shared_ptr<happymoon_control::ControlDataArbiter> ctrl_arbiter_ptr_;
  void joyStickCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void serverCmdCallback(const std_msgs::String::ConstPtr &msg);
  void stateEstimateCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void
  tofSenseCallback(const happymoon_quad_control::TofsenseFrame0::ConstPtr &msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);
  Eigen::Vector3d geometryToEigen(const geometry_msgs::Point &vec_ros);
  QuadStateReferenceData
  QuadReferenceState(HappymoonReference ref_msg, QuadStateEstimateData est_msg,
                     const PositionControllerParams &config);
  QuadStateEstimateData QuadStateEstimate(const nav_msgs::Odometry &msg);
  void ControlRun(const QuadStateEstimateData &state_estimate,
                  const QuadStateReferenceData &state_reference,
                  const PositionControllerParams &config);
  Eigen::Vector3d
  computePIDErrorAcc(const QuadStateEstimateData &state_estimate,
                     const QuadStateReferenceData &reference_state,
                     const PositionControllerParams &config);
  double computeDesiredCollectiveMassNormalizedThrust(
      const Eigen::Quaterniond &attitude_estimate,
      const Eigen::Vector3d &desired_acc,
      const PositionControllerParams &config);

  Eigen::Vector3d
  computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype,
                         const Eigen::Vector3d &x_C, const Eigen::Vector3d &y_C,
                         const Eigen::Quaterniond &attitude_estimate);

  Eigen::Quaterniond
  computeDesiredAttitude(const Eigen::Vector3d &desired_acceleration,
                         const double reference_heading,
                         const Eigen::Quaterniond &attitude_estimate);

  void setZeroCtrl(djiFlightControl *ctrl_msg);

  bool almostZero(const double value);
  bool almostZeroThrust(const double thrust_value);

  void state_cb(const mavros_msgs::State::ConstPtr &msg);

  void runBehavior(void);

  ros::Publisher ctrlAngleThrust;
  ros::Publisher ctrlAnglePX4;
  ros::Publisher ctrlManualControlPX4;
  ros::Publisher ctrlExpectAngleThrustPX4;

  ros::Subscriber joy_cmd_sub;
  ros::Subscriber imu_data_sub;
  ros::Subscriber server_cmd_sub;
  ros::Subscriber vision_odom_sub;
  ros::Subscriber tofsense_dis_sub;
  ros::Subscriber state_sub;

  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

  mutable std::mutex main_mutex_;
  std::thread *run_behavior_thread_;

  ros::Time timestamp;

  HappymoonReference happymoon_reference;
  PositionControllerParams happymoon_config;
  ControlCommand happymoon_control_command;

  GeometryEigenConversions geometryToEigen_;
  MathCommon mathcommon_;
  PositionControllerParams happymoonconfig_;

  sensor_msgs::Joy ctrlDjiFlightData;
  sensor_msgs::Imu imu_data;
  bool stop_quad;
  float height_dis;

  // Constants
  const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
  static constexpr double kMinNormalizedCollectiveThrust_ = 8.0;
  static constexpr double kMaxNormalizedCollectiveThrust_ = 15.0;
  static constexpr double kAlmostZeroValueThreshold_ = 0.001;
  static constexpr double kAlmostZeroThrustThreshold_ = 0.01;

  // mavros msg
  mavros_msgs::State current_state;
  mavros_msgs::SetMode set_mode;
  mavros_msgs::CommandBool arm_cmd;
};

} // namespace happymoon_control

#endif

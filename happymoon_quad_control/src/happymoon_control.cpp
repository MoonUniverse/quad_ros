#include "happymoon_control.h"

namespace happymoon_control
{

HappyMoonControl::HappyMoonControl()
{
    ros::NodeHandle nh("~");
    nh.param("kpxy", happymoon_config.kpxy, 0.0);
    nh.param("kdxy", happymoon_config.kdxy, 0.0);
    nh.param("kpz", happymoon_config.kpz, 0.0);
    nh.param("kdz", happymoon_config.kdz, 0.0);
    nh.param("krp", happymoon_config.krp, 0.0);
    nh.param("kyaw", happymoon_config.kyaw, 0.0);

    nh.param("refVelXYKp", happymoon_config.refVelXYKp, 0.0);
    nh.param("refVelZKp", happymoon_config.refVelZKp, 0.0);
    nh.param("refVelHeadingKp", happymoon_config.refVelHeadingKp, 0.0);
    nh.param("refVelRateheadingKp", happymoon_config.refVelRateheadingKp, 0.0);

    nh.param("pxy_error_max", happymoon_config.pxy_error_max, 0.0);
    nh.param("vxy_error_max", happymoon_config.vxy_error_max, 0.0);
    nh.param("pz_error_max", happymoon_config.pz_error_max, 0.0);
    nh.param("vz_error_max", happymoon_config.vz_error_max, 0.0);
    nh.param("yaw_error_max", happymoon_config.yaw_error_max, 0.0);

    // Publish the control signal
    ctrlAngleThrust = nh.advertise<sensor_msgs::Joy>("/djiros/ctrl", 10);
    // Subcribe the control signal
    joy_cmd = nh.subscribe<sensor_msgs::Joy>("/joy", 10, boost::bind(&HappyMoonControl::joyStickCallback, this, _1));
    // Subcribe the reference signal 
    // reference_state = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("");
    // Subcribe the VIO nav msg
    vision_odom = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 10,
                                                    boost::bind(&HappyMoonControl::stateEstimateCallback, this, _1),
                                                        ros::VoidConstPtr(),
                                                          ros::TransportHints().tcpNoDelay());
}

void HappyMoonControl::joyStickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy == nullptr){
        return;
    }
    sensor_msgs::Joy ctrlAngleThrustData;
    ctrlAngleThrustData.header.stamp = ros::Time::now();
    ctrlAngleThrustData.header.frame_id = std::string("FRD");
    ctrlAngleThrustData.axes.push_back(-joy->axes[3] * 10);//roll
    ctrlAngleThrustData.axes.push_back(joy->axes[4] * 10); //pitch
    ctrlAngleThrustData.axes.push_back((joy->axes[1] + 1.0)/2);      //thrust
    ctrlAngleThrustData.axes.push_back(joy->axes[0]);   //yawRate

    ctrlAngleThrust.publish(ctrlAngleThrustData);
    ROS_INFO("roll:%f,pitch:%f,THRUST:%f,YawRate:%f",-joy->axes[3] * 10,joy->axes[4] * 10,(joy->axes[1] + 1.0)/2,joy->axes[0]);

}

void HappyMoonControl::stateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg == nullptr) {
        return;
    }    
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    QuadStateEstimateData happymoon_state_estimate;
    QuadStateReferenceData happymoon_state_reference;
    happymoon_state_estimate = QuadStateEstimate(*msg);
    happymoon_state_reference = QuadReferenceState(happymoon_reference,happymoon_state_estimate);
    ControlRun(happymoon_state_estimate,happymoon_state_reference,happymoon_config);
}

QuadStateReferenceData HappyMoonControl::QuadReferenceState(HappymoonReference ref_msg,QuadStateEstimateData est_msg){
  QuadStateReferenceData happymoon_reference_state;
  happymoon_reference_state.position.x() =  ref_msg.position.x();
  happymoon_reference_state.position.y() =  ref_msg.position.y();
  happymoon_reference_state.position.z() =  ref_msg.position.z();
  happymoon_reference_state.heading = ref_msg.heading;
  happymoon_reference_state.velocity.x() =  happymoon_config.refVelXYKp * (ref_msg.position.x() - est_msg.position.x());
  happymoon_reference_state.velocity.y() =  happymoon_config.refVelXYKp * (ref_msg.position.y() - est_msg.position.y());
  happymoon_reference_state.velocity.z() =  happymoon_config.refVelZKp * (ref_msg.position.z() - est_msg.position.z());

  return happymoon_reference_state;
}

QuadStateEstimateData HappyMoonControl::QuadStateEstimate(
     const nav_msgs::Odometry& state_estimate_msg)
{  
  QuadStateEstimateData happymoon_state_estimate;  
  timestamp = state_estimate_msg.header.stamp;
  happymoon_state_estimate.position = geometryToEigen_.geometryToEigen(state_estimate_msg.pose.pose.position);
  happymoon_state_estimate.velocity = geometryToEigen_.geometryToEigen(state_estimate_msg.twist.twist.linear);
  happymoon_state_estimate.orientation = geometryToEigen_.geometryToEigen(state_estimate_msg.pose.pose.orientation);
  happymoon_state_estimate.roll_pitch_yaw = mathcommon_.quaternionToEulerAnglesZYX(happymoon_state_estimate.orientation);
  happymoon_state_estimate.bodyrates = geometryToEigen_.geometryToEigen(state_estimate_msg.twist.twist.angular);

  return happymoon_state_estimate;
}


void HappyMoonControl::ControlRun(const QuadStateEstimateData& state_estimate,
                                    const QuadStateReferenceData& state_reference,
                                        const PositionControllerParams& config){

    ControlCommand command;
    // Compute desired control commands                                        
    const Eigen::Vector3d pid_error_accelerations =
      computePIDErrorAcc(state_estimate, state_reference, config);
    const Eigen::Vector3d desired_acceleration = pid_error_accelerations - kGravity_;
    command.collective_thrust = computeDesiredCollectiveMassNormalizedThrust(
      state_estimate.orientation, desired_acceleration, config);
    const Eigen::Quaterniond desired_attitude =
      computeDesiredAttitude(desired_acceleration, state_reference.heading,
                             state_estimate.orientation);
    const Eigen::Vector3d desired_r_p_y = mathcommon_.quaternionToEulerAnglesZYX(desired_attitude);

    sensor_msgs::Joy controlAngleThrust;
    controlAngleThrust.axes.push_back(desired_r_p_y.x());//roll
    controlAngleThrust.axes.push_back(desired_r_p_y.y());//pitch
    controlAngleThrust.axes.push_back(command.collective_thrust);//thrust
    controlAngleThrust.axes.push_back(config.kyaw * desired_r_p_y.z());//yawRate

    ctrlAngleThrust.publish(controlAngleThrust);
}


Eigen::Vector3d HappyMoonControl::computePIDErrorAcc(
                const QuadStateEstimateData& state_estimate,
                    const QuadStateReferenceData& reference_state,
                        const PositionControllerParams& config){
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error =
      reference_state.position.x() - state_estimate.position.x();
  mathcommon_.limit(&x_pos_error, -config.pxy_error_max,
                          config.pxy_error_max);

  double x_vel_error =
      reference_state.velocity.x() - state_estimate.velocity.x();
  mathcommon_.limit(&x_vel_error, -config.vxy_error_max,
                          config.vxy_error_max);

  acc_error.x() = config.kpxy * x_pos_error + config.kdxy * x_vel_error;

  // y acceleration
  double y_pos_error =
      reference_state.position.y() - state_estimate.position.y();
  mathcommon_.limit(&y_pos_error, -config.pxy_error_max,
                          config.pxy_error_max);

  double y_vel_error =
      reference_state.velocity.y() - state_estimate.velocity.y();
  mathcommon_.limit(&y_vel_error, -config.vxy_error_max,
                          config.vxy_error_max);

  acc_error.y() = config.kpxy * y_pos_error + config.kdxy * y_vel_error;

  // z acceleration
  double z_pos_error =
      reference_state.position.z() - state_estimate.position.z();
  mathcommon_.limit(&z_pos_error, -config.pz_error_max,
                          config.pz_error_max);

  double z_vel_error =
      reference_state.velocity.z() - state_estimate.velocity.z();
  mathcommon_.limit(&z_vel_error, -config.vz_error_max,
                          config.vz_error_max);

  acc_error.z() = config.kpz * z_pos_error + config.kdz * z_vel_error;

  return acc_error;
}

double HappyMoonControl::computeDesiredCollectiveMassNormalizedThrust(
    const Eigen::Quaterniond& attitude_estimate,
    const Eigen::Vector3d& desired_acc,
    const PositionControllerParams& config){
  const Eigen::Vector3d body_z_axis =
      attitude_estimate * Eigen::Vector3d::UnitZ();

  double normalized_thrust = desired_acc.dot(body_z_axis);
  if (normalized_thrust < kMinNormalizedCollectiveThrust_) {
    normalized_thrust = kMinNormalizedCollectiveThrust_;
  }
  return normalized_thrust;
}

Eigen::Quaterniond HappyMoonControl::computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) {
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  Eigen::Vector3d z_B;
  if (almostZero(desired_acceleration.norm())) {
    // In case of free fall we keep the thrust direction to be the estimated one
    // This only works assuming that we are in this condition for a very short
    // time (otherwise attitude drifts)
    z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
  } else {
    z_B = desired_acceleration.normalized();
  }

  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}

Eigen::Vector3d HappyMoonControl::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) {
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm())) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm())) {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  //  if (x_B.dot(x_C) < 0.0)
  //  {
  //    x_B = -x_B;
  //  }

  return x_B;
}

bool HappyMoonControl::almostZero(const double value) {
  return fabs(value) < kAlmostZeroValueThreshold_;
}

bool HappyMoonControl::almostZeroThrust(const double thrust_value) {
  return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
}


HappyMoonControl::~HappyMoonControl()
{
}

}


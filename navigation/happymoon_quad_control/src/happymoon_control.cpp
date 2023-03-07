#include "happymoon_control.h"

namespace happymoon_control
{

  HappyMoonControl::HappyMoonControl() : stop_quad(false)
  {
    ros::NodeHandle nh("~");
    nh.param("kpxy", happymoon_config.kpxy, 10.0);
    nh.param("kdxy", happymoon_config.kdxy, 4.0);
    nh.param("kpz", happymoon_config.kpz, 15.0);
    nh.param("kdz", happymoon_config.kdz, 6.0);
    nh.param("krp", happymoon_config.krp, 12.0);
    nh.param("kyaw", happymoon_config.kyaw, 5.0);

    nh.param("refVelXYKp", happymoon_config.refVelXYKp, 0.5);
    nh.param("refVelZKp", happymoon_config.refVelZKp, 0.5);
    nh.param("refVelHeadingKp", happymoon_config.refVelHeadingKp, 0.5);
    nh.param("refVelRateheadingKp", happymoon_config.refVelRateheadingKp, 0.5);

    nh.param("pxy_error_max", happymoon_config.pxy_error_max, 0.6);
    nh.param("vxy_error_max", happymoon_config.vxy_error_max, 1.0);
    nh.param("pz_error_max", happymoon_config.pz_error_max, 0.3);
    nh.param("vz_error_max", happymoon_config.vz_error_max, 0.75);
    nh.param("yaw_error_max", happymoon_config.yaw_error_max, 0.7);

    nh.param("ref_pos_x", happymoon_reference.position.x(), 0.0);
    nh.param("ref_pos_y", happymoon_reference.position.y(), 0.0);
    nh.param("ref_pos_z", happymoon_reference.position.z(), 1.0);
    nh.param("ref_head", happymoon_reference.heading, 0.0);
    nh.param("ref_head_rate", happymoon_reference.heading_rate, 0.0);

    ctrl_arbiter_ptr_.reset(new ControlDataArbiter());
    // Publish the control signal
    ctrlAngleThrust = nh.advertise<sensor_msgs::Joy>("/djiros/ctrl", 1);
    ctrlAnglePX4 = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
    ctrlManualControlPX4 = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/send", 1);
    ctrlExpectAngleThrustPX4 = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

    // Subcribe the control signal
    joy_cmd_sub = nh.subscribe<sensor_msgs::Joy>(
        "/joy", 10, boost::bind(&HappyMoonControl::joyStickCallback, this, _1));
    // Subcribe the reference signal
    server_cmd_sub = nh.subscribe<std_msgs::String>(
        "/happymoon/server_cmd", 1,
        boost::bind(&HappyMoonControl::serverCmdCallback, this, _1));
    // VIO nav msg sub
    vision_odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 1,
        boost::bind(&HappyMoonControl::stateEstimateCallback, this, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    // TofSense msg sub
    tofsense_dis_sub = nh.subscribe<happymoon_quad_control::TofsenseFrame0>(
        "/nlink_tofsense_frame0", 1,
        boost::bind(&HappyMoonControl::tofSenseCallback, this, _1));
    // imu msg sub
    imu_data_sub = nh.subscribe<sensor_msgs::Imu>(
        "/mavros/imu/data_raw", 1,
        boost::bind(&HappyMoonControl::ImuCallback, this, _1));
    // PX4 state sub
    state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 1, 
        boost::bind(&HappyMoonControl::state_cb, this, _1));

    //service
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");   

    // thread
    run_behavior_thread_ =
        new std::thread(std::bind(&HappyMoonControl::runBehavior, this));
  }

  void HappyMoonControl::runBehavior(void)
  {
    ros::NodeHandle n;
    ros::Rate rate(50.0);

    while (n.ok())
    {

      


      ros::spinOnce();
      rate.sleep();
    }
  }

  
  void HappyMoonControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
      current_state = *msg;
      // std::cout << current_state << std::endl;
  }

  void HappyMoonControl::tofSenseCallback(
      const happymoon_quad_control::TofsenseFrame0::ConstPtr &msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    height_dis = msg->dis;
  }

  void HappyMoonControl::ImuCallback(
      const sensor_msgs::Imu::ConstPtr &imu_msg)
  {
    if (imu_msg == nullptr)
    {
      return;
    }
    imu_data.header.frame_id = imu_msg->header.frame_id;
    imu_data.orientation = imu_msg->orientation;
    imu_data.angular_velocity = imu_msg->angular_velocity;
    imu_data.linear_acceleration = imu_msg->linear_acceleration;
  }

  void HappyMoonControl::joyStickCallback(const sensor_msgs::Joy::ConstPtr &joy)
  {
    if(joy == nullptr) return;

    if (1 == joy->buttons[0])
    {
      arm_cmd.request.value = true;
      arming_client.call(arm_cmd);
      if( arm_cmd.response.success)
      {
        ROS_INFO("Vehicle armed");
      }
    }
    
    if(1 == joy->buttons[1])
    {
      set_mode.request.custom_mode = "STABILIZED";
      set_mode_client.call(set_mode);
      if(set_mode.response.mode_sent)
      {
        ROS_INFO("mode sent successful");
      }
    }

    if(1 == joy->buttons[2])
    {
      arm_cmd.request.value = false;
      arming_client.call(arm_cmd);
      if( arm_cmd.response.success)
      {
        ROS_INFO("Vehicle unarmed");
      }
    }
  }

  void HappyMoonControl::serverCmdCallback(
      const std_msgs::String::ConstPtr &msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    std_msgs::String cmd_str = *msg;
    if (cmd_str.data.find("StartMove:") != std::string::npos)
    {
      std::string param_str;
      param_str = cmd_str.data.substr(10, cmd_str.data.length() - 10);
      std::vector<std::string> params;
      std::istringstream param_stream(param_str);
      std::string param;
      while (getline(param_stream, param, ','))
      {
        params.push_back(param);
      }
      if (params.size() == 5)
      {
        ROS_INFO("pos_x: %s, pos_y: %s, pos_z: %s, head: %s, head_rate: %s",
                 params[0].c_str(), params[1].c_str(), params[2].c_str(),
                 params[3].c_str(), params[4].c_str());
        happymoon_reference.position.x() = atof(params[0].c_str());
        happymoon_reference.position.y() = atof(params[1].c_str());
        happymoon_reference.position.z() = atof(params[2].c_str());
        happymoon_reference.heading = atof(params[3].c_str());
        happymoon_reference.heading_rate = atof(params[4].c_str());
      }
      else
      {
        ROS_ERROR("Param error!");
        return;
      }
    }
  }

  void HappyMoonControl::stateEstimateCallback(
      const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    QuadStateEstimateData happymoon_state_estimate;
    QuadStateReferenceData happymoon_state_reference;
    happymoon_state_estimate = QuadStateEstimate(*msg);
    happymoon_state_reference =
        QuadReferenceState(happymoon_reference, happymoon_state_estimate);
    ControlRun(happymoon_state_estimate, happymoon_state_reference,
               happymoon_config);
  }

  QuadStateReferenceData
  HappyMoonControl::QuadReferenceState(HappymoonReference ref_msg,
                                       QuadStateEstimateData est_msg)
  {
    QuadStateReferenceData happymoon_reference_state;
    happymoon_reference_state.position.x() = ref_msg.position.x();
    happymoon_reference_state.position.y() = ref_msg.position.y();
    happymoon_reference_state.position.z() = ref_msg.position.z();
    happymoon_reference_state.heading = ref_msg.heading;
    happymoon_reference_state.velocity.x() =
        happymoon_config.refVelXYKp *
        (ref_msg.position.x() - est_msg.position.x());
    happymoon_reference_state.velocity.y() =
        happymoon_config.refVelXYKp *
        (ref_msg.position.y() - est_msg.position.y());
    happymoon_reference_state.velocity.z() =
        happymoon_config.refVelZKp *
        (ref_msg.position.z() - est_msg.position.z());

    return happymoon_reference_state;
  }

  QuadStateEstimateData HappyMoonControl::QuadStateEstimate(
      const nav_msgs::Odometry &state_estimate_msg)
  {
    QuadStateEstimateData happymoon_state_estimate;
    timestamp = state_estimate_msg.header.stamp;
    happymoon_state_estimate.position =
        geometryToEigen_.geometryToEigen(state_estimate_msg.pose.pose.position);
    happymoon_state_estimate.velocity =
        geometryToEigen_.geometryToEigen(state_estimate_msg.twist.twist.linear);
    happymoon_state_estimate.orientation = geometryToEigen_.geometryToEigen(
        state_estimate_msg.pose.pose.orientation);
    happymoon_state_estimate.roll_pitch_yaw =
        mathcommon_.quaternionToEulerAnglesZYX(
            happymoon_state_estimate.orientation);
    happymoon_state_estimate.bodyrates =
        geometryToEigen_.geometryToEigen(state_estimate_msg.twist.twist.angular);

    return happymoon_state_estimate;
  }

  void HappyMoonControl::ControlRun(const QuadStateEstimateData &state_estimate,
                                    const QuadStateReferenceData &state_reference,
                                    const PositionControllerParams &config)
  {
    ControlCommand command;
    // Compute desired control commands
    const Eigen::Vector3d pid_error_accelerations =
        computePIDErrorAcc(state_estimate, state_reference, config);
    const Eigen::Vector3d desired_acceleration =
        pid_error_accelerations - kGravity_;
    command.collective_thrust = computeDesiredCollectiveMassNormalizedThrust(
        state_estimate.orientation, desired_acceleration, config);
    const Eigen::Quaterniond desired_attitude =
        computeDesiredAttitude(desired_acceleration, state_reference.heading,
                               state_estimate.orientation);

    const Eigen::Vector3d desired_r_p_y =
        mathcommon_.quaternionToEulerAnglesZYX(desired_attitude);
    geometry_msgs::Vector3 desired_r_p_y_rate;
    desired_r_p_y_rate.x = 0.5 * desired_r_p_y.x();
    desired_r_p_y_rate.y = 0.5 * desired_r_p_y.y();
    desired_r_p_y_rate.z = 0.2 * desired_r_p_y.z();

    mavros_msgs::AttitudeTarget expect_px;

    if(current_state.armed && current_state.mode == "OFFBOARD"){
      expect_px.header.frame_id = "base_link";
      expect_px.header.stamp = ros::Time::now();
      expect_px.type_mask = 7;
      expect_px.orientation = geometryToEigen_.eigenToGeometry(desired_attitude);
      expect_px.body_rate = desired_r_p_y_rate;
      expect_px.thrust = command.collective_thrust / 10;
    }else{
      expect_px.header.frame_id = "base_link";
      expect_px.header.stamp = ros::Time::now();
      expect_px.orientation = geometryToEigen_.eigenToGeometry(desired_attitude);
      expect_px.body_rate = desired_r_p_y_rate;
      expect_px.thrust = 0;
    }
    ctrlExpectAngleThrustPX4.publish(expect_px);
  }

  Eigen::Vector3d HappyMoonControl::computePIDErrorAcc(
      const QuadStateEstimateData &state_estimate,
      const QuadStateReferenceData &reference_state,
      const PositionControllerParams &config)
  {
    // Compute the desired accelerations due to control errors in world frame
    // with a PID controller
    Eigen::Vector3d acc_error;

    // x acceleration
    double x_pos_error =
        reference_state.position.x() - state_estimate.position.x();
    mathcommon_.limit(&x_pos_error, -config.pxy_error_max, config.pxy_error_max);

    double x_vel_error =
        reference_state.velocity.x() - state_estimate.velocity.x();
    mathcommon_.limit(&x_vel_error, -config.vxy_error_max, config.vxy_error_max);

    acc_error.x() = config.kpxy * x_pos_error + config.kdxy * x_vel_error;

    // y acceleration
    double y_pos_error =
        reference_state.position.y() - state_estimate.position.y();
    mathcommon_.limit(&y_pos_error, -config.pxy_error_max, config.pxy_error_max);

    double y_vel_error =
        reference_state.velocity.y() - state_estimate.velocity.y();
    mathcommon_.limit(&y_vel_error, -config.vxy_error_max, config.vxy_error_max);

    acc_error.y() = config.kpxy * y_pos_error + config.kdxy * y_vel_error;

    // z acceleration
    double z_pos_error =
        reference_state.position.z() - state_estimate.position.z();
    mathcommon_.limit(&z_pos_error, -config.pz_error_max, config.pz_error_max);

    double z_vel_error =
        reference_state.velocity.z() - state_estimate.velocity.z();
    mathcommon_.limit(&z_vel_error, -config.vz_error_max, config.vz_error_max);

    acc_error.z() = config.kpz * z_pos_error + config.kdz * z_vel_error;

    return acc_error;
  }

  double HappyMoonControl::computeDesiredCollectiveMassNormalizedThrust(
      const Eigen::Quaterniond &attitude_estimate,
      const Eigen::Vector3d &desired_acc,
      const PositionControllerParams &config)
  {
    const Eigen::Vector3d body_z_axis =
        attitude_estimate * Eigen::Vector3d::UnitZ();

    double normalized_thrust = desired_acc.dot(body_z_axis);
    if (normalized_thrust < kMinNormalizedCollectiveThrust_)
    {
      normalized_thrust = kMinNormalizedCollectiveThrust_;
    }
    return normalized_thrust;
  }

  Eigen::Quaterniond HappyMoonControl::computeDesiredAttitude(
      const Eigen::Vector3d &desired_acceleration, const double reference_heading,
      const Eigen::Quaterniond &attitude_estimate)
  {
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
        Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

    // Compute desired orientation
    const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

    Eigen::Vector3d z_B;
    if (almostZero(desired_acceleration.norm()))
    {
      // In case of free fall we keep the thrust direction to be the estimated one
      // This only works assuming that we are in this condition for a very short
      // time (otherwise attitude drifts)
      z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
    }
    else
    {
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
      const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
      const Eigen::Vector3d &y_C, const Eigen::Quaterniond &attitude_estimate)
  {
    Eigen::Vector3d x_B = x_B_prototype;

    if (almostZero(x_B.norm()))
    {
      // if cross(y_C, z_B) == 0, they are collinear =>
      // every x_B lies automatically in the x_C - z_C plane

      // Project estimated body x-axis into the x_C - z_C plane
      const Eigen::Vector3d x_B_estimated =
          attitude_estimate * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d x_B_projected =
          x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
      if (almostZero(x_B_projected.norm()))
      {
        // Not too much intelligent stuff we can do in this case but it should
        // basically never occur
        x_B = x_C;
      }
      else
      {
        x_B = x_B_projected.normalized();
      }
    }
    else
    {
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

  void HappyMoonControl::setZeroCtrl(djiFlightControl *ctrl_msg)
  {
    if (ctrl_msg == nullptr)
    {
      return;
    }
    ctrl_msg->pitch = 0;
    ctrl_msg->roll = 0;
    ctrl_msg->thrust = 0;
    ctrl_msg->yawrate = 0;
  }

  bool HappyMoonControl::almostZero(const double value)
  {
    return fabs(value) < kAlmostZeroValueThreshold_;
  }

  bool HappyMoonControl::almostZeroThrust(const double thrust_value)
  {
    return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
  }

  HappyMoonControl::~HappyMoonControl() {}

} // namespace happymoon_control

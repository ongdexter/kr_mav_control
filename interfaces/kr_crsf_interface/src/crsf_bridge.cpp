#include <kr_crsf_interface/crsf_bridge.h>

#include <Eigen/Dense>
#include <kr_crsf_interface/crsf_channel_mapping.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace crsf_bridge {

CrsfBridge::CrsfBridge(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      logger_(node->get_logger()),
      stop_watchdog_thread_(false),
      time_last_rc_msg_received_(),
      time_last_crsf_msg_sent_(node_->now()),
      time_last_battery_voltage_received_(node_->now()),
      time_last_active_control_command_received_(),
      bridge_state_(BridgeState::KILL),
      control_mode_(ControlMode::NONE),
      arming_counter_(0),
      battery_voltage_(0.0),
      bridge_armed_(false),
      rc_was_disarmed_once_(false),
      destructor_invoked_(false) {
  if (!loadParameters()) {
    RCLCPP_ERROR(logger_, "Could not load parameters.");
    rclcpp::shutdown();
    return;
  }

  if (!setUpCrsfSerialPort(port_name_, enable_receiving_crsf_messages_, node->get_clock())) {
    rclcpp::shutdown();
    return;
  }

  try {
    watchdog_thread_ = std::thread(&CrsfBridge::watchdogThread, this);
  } catch (...) {
    RCLCPP_ERROR(
        logger_,
        "Could not successfully start watchdog thread. Exiting CrsfBridge.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(logger_, "CrsfBridge initialized");
}

CrsfBridge::~CrsfBridge() {
  destructor_invoked_ = true;

  if (enable_receiving_crsf_messages_) {
    stopReceiverThread();
  }

  stop_watchdog_thread_ = true;
  watchdog_thread_.join();
  setBridgeState(BridgeState::OFF);

  // Send disarming CRSF message for safety
  CrsfMsg shut_down_message;
  shut_down_message.setArmStateDisarmed();
  rclcpp::Rate loop_rate(110.0);
  for (int i = 0; i < kSmoothingFailRepetitions_; i++) {
    transmitSerialCrsfMessage(shut_down_message);
    loop_rate.sleep();
  }

  disconnectSerialPort();
}

void CrsfBridge::watchdogThread() {
  rclcpp::Rate watchdog_rate(110.0);
  while (rclcpp::ok() && !stop_watchdog_thread_) {
    watchdog_rate.sleep();
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    const rclcpp::Time time_now = node_->now();
    if (bridge_state_ == BridgeState::RC_FLIGHT &&
        (time_now - time_last_rc_msg_received_).seconds() > rc_timeout_) {
      RCLCPP_WARN(
          logger_,
          "Remote control was active but no message from it was received "
          "within timeout (%f s).",
          rc_timeout_);
      setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
    }
    if (bridge_state_ == BridgeState::KILL) {
      CrsfMsg kill_msg;
      kill_msg.setArmStateDisarmed();
      sendCrsfMessageToSerialPort(kill_msg);
      if (bridge_armed_) {
        disarmBridge();
      }
    }
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
    if ((time_now - time_last_battery_voltage_received_).seconds() >
        kBatteryVoltageTimeout_) {
      battery_voltage_ = 0.0;
      if (perform_thrust_voltage_compensation_) {
        RCLCPP_WARN_THROTTLE(
            logger_, *node_->get_clock(), 1000,
            "Can not perform battery voltage compensation because there "
            "is no recent battery voltage measurement");
      }
    }
  }
}

void CrsfBridge::handleReceivedCrsfMessage(const CrsfMsg &received_crsf_msg)
{
  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);
    time_last_rc_msg_received_ = node_->now();
    if (received_crsf_msg.isKillSwitch())
    {
      if (bridge_state_ != BridgeState::KILL)
      {
        setBridgeState(BridgeState::KILL);
        RCLCPP_INFO(logger_, "Kill switch ON");
      }
    }
    else
    {
      if (bridge_state_ == BridgeState::KILL)
      {
        setBridgeState(BridgeState::OFF);
        RCLCPP_WARN(logger_, "Kill switch OFF");
      }
    }
    if (bridge_state_ == BridgeState::KILL)
    {
      return;
    }
    if (received_crsf_msg.isArmed()) 
    {
      if (!rc_was_disarmed_once_) {
        RCLCPP_WARN_THROTTLE(
            logger_, *node_->get_clock(), 1.0,
            "RC needs to be disarmed once before it can take over control");
        return;
      }
      if (bridge_state_ != BridgeState::RC_FLIGHT) {
        setBridgeState(BridgeState::RC_FLIGHT);
        RCLCPP_INFO(logger_, "Control authority taken over by remote control.");
      }
      sendCrsfMessageToSerialPort(received_crsf_msg);
      control_mode_ = received_crsf_msg.getControlMode();
    }
  }
}

void CrsfBridge::controlCommandCallback(
    const kr_mav_msgs::msg::SO3Command::ConstPtr &msg,
    const Eigen::Quaterniond &odom_q) 
{
  std::lock_guard<std::mutex> main_lock(main_mutex_);
  if (destructor_invoked_)
  {
    return;
  }
  time_last_active_control_command_received_ = node_->now();
  if (!bridge_armed_ || bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
  {    
    if (!bridge_armed_ && msg->aux.enable_motors &&
        bridge_state_ != BridgeState::RC_FLIGHT)
    {
      RCLCPP_WARN(
          logger_,
          "Received active control command but crsf bridge is not armed. "
          "Please arm the bridge before sending control commands.");
    }
    return;
  }
  CrsfMsg crsf_msg_to_send;
  {
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
    crsf_msg_to_send = generateCrsfMessageFromSO3Command(msg, odom_q);
  }
  if (!msg->aux.enable_motors) 
  {
    if (bridge_state_ == BridgeState::ARMING ||
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT)
    {
      RCLCPP_INFO(logger_, "Control command received, setting bridge state to OFF");
      setBridgeState(BridgeState::OFF);
    }
    crsf_msg_to_send.setArmStateDisarmed();
  }
  crsf_msg_to_send.limitAllChannelsFeasible();
  sendCrsfMessageToSerialPort(crsf_msg_to_send);
}

void CrsfBridge::sendCrsfMessageToSerialPort(const CrsfMsg& crsf_msg)
{
  CrsfMsg crsf_message_to_send = crsf_msg;

  switch (bridge_state_) {
    case BridgeState::OFF:
      // Disarm vehicle
      crsf_message_to_send.setArmStateDisarmed();
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple
      // messages before arming, we repeat the messages with minimum throttle
      // and arming command multiple times. 5 times seems to work robustly.
      if (arming_counter_ >= kSmoothingFailRepetitions_) {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      } else {
        // Set thrust to minimum command to make sure FC arms
        crsf_message_to_send.setThrottleCommand(CrsfMsg::kMinCmd);
        crsf_message_to_send.setArmStateArmed();
        arming_counter_++;
      }
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      crsf_message_to_send.setArmStateArmed();
      if (use_body_rates_)
      {
        crsf_message_to_send.setControlModeBodyRates();
      } else
      {
        crsf_message_to_send.setControlModeAttitude();
      }
      break;

    case BridgeState::RC_FLIGHT:
      // Passing RC command straight through
      crsf_message_to_send.setControlModeAttitude();
      break;

    case BridgeState::KILL:
      // Disarm vehicle
      crsf_message_to_send.setArmStateDisarmed();
      break;

    default:
      // Disarm the vehicle because this code must be terribly wrong
      crsf_message_to_send.setArmStateDisarmed();
      RCLCPP_WARN(
          logger_, "Bridge is in unknown state, vehicle will be disarmed");
      break;
  }

  if ((node_->now() - time_last_crsf_msg_sent_).seconds() <= 0.006) {
    // An SBUS message takes 3ms to be transmitted by the serial port so let's
    // not stress it too much. This should only happen in case of switching
    // between control commands and rc commands
    if (bridge_state_ == BridgeState::ARMING) {
      // In case of arming we want to send kSmoothingFailRepetitions_ messages
      // with minimum throttle to the flight controller. Since this check
      // prevents the message from being sent out we reduce the counter that
      // was incremented above assuming the message would actually be sent.
      arming_counter_--;
    }
    return;
  }

  crsf_message_to_send.timestamp = node_->now();
  transmitSerialCrsfMessage(crsf_message_to_send);
  time_last_crsf_msg_sent_ = node_->now();
}

static std::pair<double, double> solve_quadratic(double a, double b, double c)
{
  const double term1 = -b, term2 = std::sqrt(b * b - 4 * a * c);
  return std::make_pair((term1 + term2) / (2 * a), (term1 - term2) / (2 * a));
}

double CrsfBridge::thrust_model_kartik(double thrust) const
{
  int num_props = 4;
  double avg_thrust = std::max(0.0, thrust) / num_props;

  // Scale thrust to individual rotor velocities (RPM)
  auto rpm_solutions =
      solve_quadratic(thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_,
                      thrust_vs_rpm_cof_c_ - avg_thrust);
  const double omega_avg = std::max(rpm_solutions.first, rpm_solutions.second);

  // Scaling from rotor velocity (RPM) to att_throttle for betaflight
  double throttle =
      (omega_avg - rpm_vs_throttle_linear_coeff_b_) / rpm_vs_throttle_linear_coeff_a_;
  return throttle;
}

CrsfMsg CrsfBridge::generateCrsfMessageFromSO3Command(const kr_mav_msgs::msg::SO3Command::ConstPtr& msg, const Eigen::Quaterniond& odom_q) const
{
  CrsfMsg crsf_msg;

  // set crsf_msg to not killed
  crsf_msg.channels[crsf_bridge::channel_mapping::kKillSwitch] = 1792;

  crsf_msg.setArmStateArmed();

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  // const Eigen::Vector3d ang_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  // convert to tf::Quaternion
  // tf::Quaternion imu_tf = tf::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  // tf::Quaternion odom_tf = tf::Quaternion(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  const Eigen::Matrix3d R_cur(odom_q);

  double thrust = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  double throttle = 0.0;
  if (thrust > 1e-5)
  {
    throttle = thrust_model_kartik(thrust);
  }

  // remap throttle (1000 to 2000) to crsf (kMinCmd to kMaxCmd)
  uint16_t throttle_cmd = round(((throttle - 1000) / 1000) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
  RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 1.0,
      "AUTONOMOUS MODE: thrust: %f throttle: %f throttle_cmd: %d",
      thrust, throttle, throttle_cmd);
  crsf_msg.setThrottleCommand(throttle_cmd);

  // convert quaternion to euler
  Eigen::Vector3d desired_euler_angles = q_des.toRotationMatrix().eulerAngles(0, 1, 2);

  // sanity check if larger than 3 rad, set to 0
  for (int i = 0; i < 3; i++) {
    if (desired_euler_angles(i) > 3.0 || desired_euler_angles(i) < -3.0)
    {
      desired_euler_angles(i) = 0;
    }
  }

  // parse commands
  uint16_t roll_cmd, pitch_cmd, yaw_cmd;

  if (use_body_rates_)
  {
    // get body rates
    double roll_rate = msg->angular_velocity.x;
    double pitch_rate = msg->angular_velocity.y;

    roll_rate = std::max(-max_roll_rate_, std::min(max_roll_rate_, roll_rate));
    pitch_rate = std::max(-max_pitch_rate_, std::min(max_pitch_rate_, pitch_rate));

    // additional safety check
    roll_rate = std::max(-400.0, std::min(400.0, roll_rate));
    pitch_rate = std::max(-400.0, std::min(400.0, pitch_rate));

    roll_cmd = round((roll_rate + max_roll_rate_) / (2 * max_roll_rate_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    pitch_cmd = round((pitch_rate + max_pitch_rate_) / (2 * max_pitch_rate_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    
    RCLCPP_INFO_THROTTLE(
        logger_, *node_->get_clock(), 1.0,
        "AUTONOMOUS MODE: roll_rate: %f pitch_rate: %f roll_cmd: %d pitch_cmd: %d",
        roll_rate, pitch_rate, roll_cmd, pitch_cmd);
  }
  else
  {
    // get body angles
    double yaw, roll, pitch;
    // use tf2 to convert quaternion to euler
    tf2::Matrix3x3(tf2::Quaternion(q_des.x(), q_des.y(), q_des.z(), q_des.w())).getRPY(roll, pitch, yaw);
    // convert to degrees
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    
    // clip max roll
    roll_deg = std::max(-max_roll_angle_, std::min(max_roll_angle_, roll_deg));
    // clip max pitch
    pitch_deg = std::max(-max_pitch_angle_, std::min(max_pitch_angle_, pitch_deg));    

    roll_cmd = round((roll_deg + max_roll_angle_) / (2 * max_roll_angle_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    pitch_cmd = round((pitch_deg + max_pitch_angle_) / (2 * max_pitch_angle_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
    
    RCLCPP_INFO_THROTTLE(
        logger_, *node_->get_clock(), 1.0,
        "AUTONOMOUS MODE: roll_deg: %f pitch_deg: %f roll_cmd: %d pitch_cmd: %d",
        roll_deg, pitch_deg, roll_cmd, pitch_cmd);
  }

  // always use yaw rate
  double yaw_rate = -msg->angular_velocity.z; // flip sign since FC yaw is inverted (z-down)
  yaw_rate = yaw_rate * 180.0 / M_PI; // convert to degrees
  yaw_rate = std::max(-400.0, std::min(400.0, yaw_rate)); // clip max yaw rate
  yaw_cmd = round((yaw_rate + max_yaw_rate_) / (2 * max_yaw_rate_) * (CrsfMsg::kMaxCmd - CrsfMsg::kMinCmd) + CrsfMsg::kMinCmd);
  
  RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 1.0,
      "AUTONOMOUS MODE: yaw_rate: %f yaw_cmd: %d",
      yaw_rate, yaw_cmd);

  crsf_msg.setRollCommand(roll_cmd);
  crsf_msg.setPitchCommand(pitch_cmd);
  crsf_msg.setYawCommand(yaw_cmd);

//   // publish values as odom message
//   nav_msgs::Odometry sbus_cmd;
//   sbus_cmd.header.stamp = ros::Time::now();
//   // convert to float
//   sbus_cmd.pose.pose.position.x = static_cast<float>(roll_cmd);
//   sbus_cmd.pose.pose.position.y = static_cast<float>(pitch_cmd);
//   sbus_cmd.pose.pose.position.z = static_cast<float>(yaw_cmd);

//   // publish
//   sbus_cmd_pub_.publish(sbus_cmd);

//   ROS_INFO("Thrust: %f", thrust_mapping_.inverseThrustMapping(
//         thrust * mass_, battery_voltage_));
  // ROS_INFO("Roll: %f", round((desired_euler_angles(0) / max_roll_angle_) *
  //                 (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
  //             SBusMsg::kMeanCmd));
  // ROS_INFO("Pitch: %f", round((desired_euler_angles(1) / max_pitch_angle_) *
  //                 (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
  //             SBusMsg::kMeanCmd));
  // ROS_INFO("Yaw: %f", round((msg->angular_velocity.z / max_yaw_rate_) *
  //                 (SBusMsg::kMaxCmd - SBusMsg::kMeanCmd) +
  //             SBusMsg::kMeanCmd));
  // ROS_INFO("Desired Euler Angles: %f, %f, %f", desired_euler_angles(0), desired_euler_angles(1), desired_euler_angles(2));
  // ROS_INFO("Current Euler Angles: %f, %f, %f", odom_q.toRotationMatrix().eulerAngles(0, 1, 2)(0), odom_q.toRotationMatrix().eulerAngles(0, 1, 2)(1), odom_q.toRotationMatrix().eulerAngles(0, 1, 2)(2));
  return crsf_msg;
}

void CrsfBridge::setBridgeState(const BridgeState& desired_bridge_state) {
  switch (desired_bridge_state) {
    case BridgeState::OFF:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::ARMING:
      bridge_state_ = desired_bridge_state;
      arming_counter_ = 0;
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::RC_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::KILL:
      bridge_state_ = desired_bridge_state;
      break;

    default:
      RCLCPP_WARN(
          logger_, "Wanted to switch to unknown bridge state, setting to OFF");
      bridge_state_ = BridgeState::OFF;
  }
}

void CrsfBridge::armBridge() 
{
  std::lock_guard<std::mutex> main_lock(arm_mutex_);
  bridge_armed_ = true;
}

void CrsfBridge::disarmBridge()
{
  std::lock_guard<std::mutex> main_lock(arm_mutex_);
  bridge_armed_ = false;
}

bool CrsfBridge::isBridgeArmed() const
{
  return bridge_armed_;
}

bool CrsfBridge::loadParameters()
{
  node_->declare_parameter("port_name", std::string("/dev/ttyUSB0"));
  node_->declare_parameter("enable_receiving_crsf_messages", true);
  node_->declare_parameter("control_command_timeout", 0.5);
  node_->declare_parameter("rc_timeout", 0.5);
  node_->declare_parameter("mass", 1.0);
  node_->declare_parameter("disable_thrust_mapping", false);
  node_->declare_parameter("max_roll_rate", 400.0);
  node_->declare_parameter("max_pitch_rate", 400.0);
  node_->declare_parameter("max_yaw_rate", 400.0);
  node_->declare_parameter("max_roll_angle", 0.5);
  node_->declare_parameter("max_pitch_angle", 0.5);
  node_->declare_parameter("alpha_vbat_filter", 0.1);
  node_->declare_parameter("perform_thrust_voltage_compensation", true);
  node_->declare_parameter("n_lipo_cells", 3);
  node_->declare_parameter("thrust_vs_rpm_cof_a_", 0.0001);
  node_->declare_parameter("thrust_vs_rpm_cof_b_", 0.0);
  node_->declare_parameter("thrust_vs_rpm_cof_c_", 0.0);
  node_->declare_parameter("rpm_vs_throttle_linear_coeff_a_", 0.0001);
  node_->declare_parameter("rpm_vs_throttle_linear_coeff_b_", 0.0);
  node_->declare_parameter("rpm_vs_throttle_quadratic_coeff_a_", 0.0001);
  node_->declare_parameter("rpm_vs_throttle_quadratic_coeff_b_", 0.0);
  node_->declare_parameter("rpm_vs_throttle_quadratic_coeff_c_", 0.0);
  node_->declare_parameter("use_body_rates", false);
  return true;
}

}  // namespace crsf_bridge

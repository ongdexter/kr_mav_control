#include <kr_betaflight_interface/sbus/sbus_channel_mapping.h>
#include "kr_betaflight_interface/sbus/sbus_bridge.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

namespace sbus_bridge
{

SBusBridge::SBusBridge(const rclcpp::Node::SharedPtr &node)
    : node_(node),
      logger_(node->get_logger()),
      stop_watchdog_thread_(false),
      time_last_rc_msg_received_(),
      time_last_sbus_msg_sent_(node_->now()),
      time_last_battery_voltage_received_(node_->now()),
      time_last_active_control_command_received_(),
      bridge_state_(BridgeState::KILL),
      control_mode_(ControlMode::NONE),
      arming_counter_(0),
      battery_voltage_(0.0),
      bridge_armed_(false),
      rc_was_disarmed_once_(false),
      destructor_invoked_(false)
{
  serial_port_ = std::make_unique<SBusSerialPort>();
  if(!loadParameters())
  {
    RCLCPP_ERROR(logger_, "Could not load parameters.");
    rclcpp::shutdown();
    return;
  }

  if(!serial_port_->setUpSBusSerialPort(port_name_, node->get_clock()))
  {
    rclcpp::shutdown();
    return;
  }

  // Start watchdog thread
  try
  {
    watchdog_thread_ = std::thread(&SBusBridge::watchdogThread, this);
  }
  catch(...)
  {
    RCLCPP_ERROR(logger_, "Could not successfully start watchdog thread. Exiting SBusBridge.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(logger_, "SBusBridge initialized");
}

SBusBridge::~SBusBridge()
{
  destructor_invoked_ = true;
  serial_port_->stopReceiverThread();

  // Stop watchdog thread
  stop_watchdog_thread_ = true;
  // Wait for watchdog thread to finish
  watchdog_thread_.join();

  // Now only one thread (the ROS thread) is remaining

  setBridgeState(BridgeState::OFF);

  // Send disarming SBus message for safety
  // We repeat it to prevent any possible smoothing of commands on the flight
  // controller to interfere with this
  SBusMsg shut_down_message;
  shut_down_message.setArmStateDisarmed();
  rclcpp::Rate loop_rate(110.0);
  for(int i = 0; i < kSmoothingFailRepetitions_; i++)
  {
    serial_port_->transmitSerialSBusMessage(shut_down_message);
    loop_rate.sleep();
  }

  // Close serial port
  serial_port_->disconnectSerialPort();
}

void SBusBridge::watchdogThread()
{
  rclcpp::Rate watchdog_rate(110.0);
  while(rclcpp::ok() && !stop_watchdog_thread_)
  {
    watchdog_rate.sleep();

    std::lock_guard<std::mutex> main_lock(main_mutex_);

    const rclcpp::Time time_now = node_->now();

    if(bridge_state_ == BridgeState::RC_FLIGHT && (time_now - time_last_rc_msg_received_).seconds() > rc_timeout_)
    {
      // If the last received RC command was armed but was received longer than
      // rc_timeout ago we switch the bridge state to AUTONOMOUS_FLIGHT.
      // In case there are no valid control commands the bridge state is set to
      // OFF in the next check below
      RCLCPP_WARN(logger_,
                  "Remote control was active but no message from it was received "
                  "within timeout (%f s).",
                  rc_timeout_);
      setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
    }

    // if (bridge_state_ == BridgeState::ARMING ||
    //     bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT) {
    //   if ((time_now - time_last_active_control_command_received_).seconds() >
    //       control_command_timeout_) {
    //     // When switching the bridge state to off, our watchdog ensures that a
    //     // disarming off message is repeated.
    //     RCLCPP_INFO(node_->get_logger(), "No active control command received, setting bridge state to OFF");
    //     setBridgeState(BridgeState::OFF);
    //     // Note: Control could theoretically still be taken over by RC but if
    //     // this happened in flight it might require super human reaction since
    //     // in this case the quad can not be armed with non zero throttle by
    //     // the remote.
    //   }
    // }

    if(bridge_state_ == BridgeState::KILL)
    {
      // Send off message that disarms the vehicle
      // We repeat it to prevent any weird behavior that occurs if the flight
      // controller is not receiving commands for a while
      SBusMsg kill_msg;
      kill_msg.setArmStateDisarmed();
      sendSBusMessageToSerialPort(kill_msg);
      // disarm bridge if needed
      if(bridge_armed_)
      {
        disarmBridge();
      }
    }

    // Check battery voltage timeout
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
    if((time_now - time_last_battery_voltage_received_).seconds() > kBatteryVoltageTimeout_)
    {
      battery_voltage_ = 0.0;
      if(perform_thrust_voltage_compensation_)
      {
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                             "Can not perform battery voltage compensation because there "
                             "is no recent battery voltage measurement");
      }
    }

    // Mutexes are unlocked because they go out of scope here
  }
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg &received_sbus_msg)
{
  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);

    time_last_rc_msg_received_ = node_->now();

    if(received_sbus_msg.isKillSwitch())
    {
      if(bridge_state_ != BridgeState::KILL)
      {
        setBridgeState(BridgeState::KILL);
        RCLCPP_INFO(logger_, "Kill switch ON");
      }
    }
    else
    {
      if(bridge_state_ == BridgeState::KILL)
      {
        setBridgeState(BridgeState::OFF);
        RCLCPP_WARN(logger_, "Kill switch OFF");
      }
    }

    if(bridge_state_ == BridgeState::KILL)
    {
      // send disarm to FC to kill motors
      //   sendSBusMessageToSerialPort(received_sbus_msg);

      return;
    }

    if(received_sbus_msg.isArmed())
    {
      if(!rc_was_disarmed_once_)
      {
        // This flag prevents that the vehicle can be armed if the RC is armed
        // on startup of the bridge
        RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1.0,
                             "RC needs to be disarmed once before it can take over control");
        return;
      }

      // Immediately go into RC_FLIGHT state since RC always has priority
      if(bridge_state_ != BridgeState::RC_FLIGHT)
      {
        setBridgeState(BridgeState::RC_FLIGHT);
        RCLCPP_INFO(logger_, "Control authority taken over by remote control.");
      }
      sendSBusMessageToSerialPort(received_sbus_msg);
      control_mode_ = received_sbus_msg.getControlMode();
    }
    else if(bridge_state_ == BridgeState::RC_FLIGHT)
    {
      // If the bridge was in state RC_FLIGHT and the RC is disarmed we set the
      // state to AUTONOMOUS_FLIGHT
      // In case there are valid control commands, the bridge will stay in
      // AUTONOMOUS_FLIGHT, otherwise the watchdog will set the state to OFF
      RCLCPP_INFO(logger_, "Control authority returned by remote control.");
      if(bridge_armed_)
      {
        RCLCPP_INFO(logger_, "Bridge armed, setting bridge state to AUTONOMOUS_FLIGHT");
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      }
      else
      {
        // When switching the bridge state to off, our watchdog ensures that a
        // disarming off message is sent
        RCLCPP_INFO(logger_, "Bridge not armed, setting bridge state to OFF");
        setBridgeState(BridgeState::OFF);
      }
    }
    else if(!rc_was_disarmed_once_)
    {
      RCLCPP_INFO(logger_, "Received disarmed RC message but RC was not disarmed once, ignoring it");
      rc_was_disarmed_once_ = true;
    }
    else if(bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
    {
      // not killed, not armed, not autonomous flight
      sendSBusMessageToSerialPort(received_sbus_msg);
    }

    // Main mutex is unlocked here because it goes out of scope
  }

  //   received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}

void SBusBridge::controlCommandCallback(const kr_mav_msgs::msg::SO3Command::ConstPtr &msg,
                                        const Eigen::Quaterniond &odom_q)
{
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if(destructor_invoked_)
  {
    // This ensures that if the destructor was invoked we do not try to write
    // to the serial port anymore because of receiving a control command
    return;
  }

  // may need more logic @TODO
  time_last_active_control_command_received_ = node_->now();

  if(!bridge_armed_ || bridge_state_ != BridgeState::AUTONOMOUS_FLIGHT)
  {
    // If bridge is not armed we do not allow control commands to be sent
    // RC has priority over control commands for autonomous flying
    if(!bridge_armed_ && msg->aux.enable_motors && bridge_state_ != BridgeState::RC_FLIGHT)
    {
      RCLCPP_WARN(logger_, "Received active control command but sbus bridge is not armed. "
                           "Please arm the bridge before sending control commands.");
    }
    return;
  }

  SBusMsg sbus_msg_to_send;
  {
    std::lock_guard<std::mutex> battery_lock(battery_voltage_mutex_);
    // Set commands
    sbus_msg_to_send = generateSBusMessageFromSO3Command(msg, odom_q);
    // Battery voltage mutex is unlocked because it goes out of scope here
  }

  // Send disarm if necessary
  if(!msg->aux.enable_motors)
  {
    if(bridge_state_ == BridgeState::ARMING || bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT)
    {
      RCLCPP_INFO(logger_, "Control command received, setting bridge state to OFF");
      setBridgeState(BridgeState::OFF);
    }
    // Make sure vehicle is disarmed to immediately switch it off
    sbus_msg_to_send.setArmStateDisarmed();
  }

  // Limit channels
  sbus_msg_to_send.limitAllChannelsFeasible();

  // Immediately send SBus message
  sendSBusMessageToSerialPort(sbus_msg_to_send);

  // Main mutex is unlocked because it goes out of scope here
}

void SBusBridge::sendSBusMessageToSerialPort(const SBusMsg &sbus_msg)
{
  // // print bridge state
  // switch (bridge_state_) {
  //     case BridgeState::OFF:
  //     ROS_INFO("Bridge state: OFF");
  //     break;
  //     case BridgeState::ARMING:
  //     ROS_INFO("Bridge state: ARMING");
  //     break;
  //     case BridgeState::AUTONOMOUS_FLIGHT:
  //     ROS_INFO("Bridge state: AUTONOMOUS_FLIGHT");
  //     break;
  //     case BridgeState::RC_FLIGHT:
  //     ROS_INFO("Bridge state: RC_FLIGHT");
  //     break;
  //     default:
  //     ROS_INFO("Bridge state: UNKNOWN");
  // }
  SBusMsg sbus_message_to_send = sbus_msg;

  switch(bridge_state_)
  {
    case BridgeState::OFF:
      // Disarm vehicle
      sbus_message_to_send.setArmStateDisarmed();
      //   ROS_INFO("Bridge state: OFF");
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple
      // messages before arming, we repeat the messages with minimum throttle
      // and arming command multiple times. 5 times seems to work robustly.
      if(arming_counter_ >= kSmoothingFailRepetitions_)
      {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      }
      else
      {
        // Set thrust to minimum command to make sure FC arms
        sbus_message_to_send.setThrottleCommand(SBusMsg::kMinCmd);
        sbus_message_to_send.setArmStateArmed();
        arming_counter_++;
      }
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      sbus_message_to_send.setArmStateArmed();
      if(use_body_rates_)
      {
        sbus_message_to_send.setControlModeBodyRates();
      }
      else
      {
        sbus_message_to_send.setControlModeAttitude();
      }
      break;

    case BridgeState::RC_FLIGHT:
      // Passing RC command straight through
      //   ROS_INFO("RC MODE: throttle_cmd %d", sbus_message_to_send.channels[sbus_bridge::channel_mapping::kThrottle]);
      sbus_message_to_send.setControlModeAttitude();
      break;

    case BridgeState::KILL:
      // Disarm vehicle
      sbus_message_to_send.setArmStateDisarmed();
      //   ROS_INFO("Bridge state: OFF");
      break;

    default:
      // Disarm the vehicle because this code must be terribly wrong
      sbus_message_to_send.setArmStateDisarmed();
      RCLCPP_WARN(logger_, "Bridge is in unknown state, vehicle will be disarmed");
      break;
  }

  if((node_->now() - time_last_sbus_msg_sent_).seconds() <= 0.006)
  {
    // An SBUS message takes 3ms to be transmitted by the serial port so let's
    // not stress it too much. This should only happen in case of switching
    // between control commands and rc commands
    if(bridge_state_ == BridgeState::ARMING)
    {
      // In case of arming we want to send kSmoothingFailRepetitions_ messages
      // with minimum throttle to the flight controller. Since this check
      // prevents the message from being sent out we reduce the counter that
      // was incremented above assuming the message would actually be sent.
      arming_counter_--;
    }
    return;
  }

  sbus_message_to_send.timestamp = node_->now();
  serial_port_->transmitSerialSBusMessage(sbus_message_to_send);
  time_last_sbus_msg_sent_ = node_->now();
}

static std::pair<double, double> solve_quadratic(double a, double b, double c)
{
  const double term1 = -b, term2 = std::sqrt(b * b - 4 * a * c);
  return std::make_pair((term1 + term2) / (2 * a), (term1 - term2) / (2 * a));
}

double SBusBridge::thrust_model_kartik(double thrust) const
{
  int num_props = 4;
  double avg_thrust = std::max(0.0, thrust) / num_props;

  // Scale thrust to individual rotor velocities (RPM)
  auto rpm_solutions = solve_quadratic(thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_, thrust_vs_rpm_cof_c_ - avg_thrust);
  const double omega_avg = std::max(rpm_solutions.first, rpm_solutions.second);

  // Scaling from rotor velocity (RPM) to att_throttle for betaflight
  double throttle = (omega_avg - rpm_vs_throttle_linear_coeff_b_) / rpm_vs_throttle_linear_coeff_a_;
  return throttle;
}

SBusMsg SBusBridge::generateSBusMessageFromSO3Command(const kr_mav_msgs::msg::SO3Command::ConstPtr &msg,
                                                      const Eigen::Quaterniond &odom_q) const
{
  SBusMsg sbus_msg;

  // set sbus_msg to not killed
  sbus_msg.channels[sbus_bridge::sbus_channel_mapping::kKillSwitch] = 1792;

  sbus_msg.setArmStateArmed();

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
  if(thrust > 1e-5)
  {
    throttle = thrust_model_kartik(thrust);
  }

  // remap throttle (1000 to 2000) to sbus (kMinCmd to kMaxCmd)
  uint16_t throttle_cmd = round(((throttle - 1000) / 1000) * (SBusMsg::kMaxCmd - SBusMsg::kMinCmd) + SBusMsg::kMinCmd);
  RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1.0, "AUTONOMOUS MODE: thrust: %f throttle: %f throttle_cmd: %d",
                       thrust, throttle, throttle_cmd);
  sbus_msg.setThrottleCommand(throttle_cmd);

  // convert quaternion to euler
  Eigen::Vector3d desired_euler_angles = q_des.toRotationMatrix().eulerAngles(0, 1, 2);

  // sanity check if larger than 3 rad, set to 0
  for(int i = 0; i < 3; i++)
  {
    if(desired_euler_angles(i) > 3.0 || desired_euler_angles(i) < -3.0)
    {
      desired_euler_angles(i) = 0;
    }
  }

  // parse commands
  uint16_t roll_cmd, pitch_cmd, yaw_cmd;

  if(use_body_rates_)
  {
    // get body rates
    double roll_rate = msg->angular_velocity.x;
    double pitch_rate = msg->angular_velocity.y;

    roll_rate = std::max(-max_roll_rate_, std::min(max_roll_rate_, roll_rate));
    pitch_rate = std::max(-max_pitch_rate_, std::min(max_pitch_rate_, pitch_rate));

    // additional safety check
    roll_rate = std::max(-400.0, std::min(400.0, roll_rate));
    pitch_rate = std::max(-400.0, std::min(400.0, pitch_rate));

    roll_cmd = round((roll_rate + max_roll_rate_) / (2 * max_roll_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMinCmd) +
                     SBusMsg::kMinCmd);
    pitch_cmd = round((pitch_rate + max_pitch_rate_) / (2 * max_pitch_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMinCmd) +
                      SBusMsg::kMinCmd);

    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
                         "AUTONOMOUS MODE: roll_rate: %f pitch_rate: %f roll_cmd: %d pitch_cmd: %d", roll_rate,
                         pitch_rate, roll_cmd, pitch_cmd);
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

    roll_cmd = round((roll_deg + max_roll_angle_) / (2 * max_roll_angle_) * (SBusMsg::kMaxCmd - SBusMsg::kMinCmd) +
                     SBusMsg::kMinCmd);
    pitch_cmd = round((pitch_deg + max_pitch_angle_) / (2 * max_pitch_angle_) * (SBusMsg::kMaxCmd - SBusMsg::kMinCmd) +
                      SBusMsg::kMinCmd);

    RCLCPP_INFO_THROTTLE(logger_, *node_->get_clock(), 1000,
                         "AUTONOMOUS MODE: roll_deg: %f pitch_deg: %f roll_cmd: %d pitch_cmd: %d", roll_deg, pitch_deg,
                         roll_cmd, pitch_cmd);
  }

  // always use yaw rate
  double yaw_rate = -msg->angular_velocity.z;              // flip sign since FC yaw is inverted (z-down)
  yaw_rate = yaw_rate * 180.0 / M_PI;                      // convert to degrees
  yaw_rate = std::max(-400.0, std::min(400.0, yaw_rate));  // clip max yaw rate
  yaw_cmd = round((yaw_rate + max_yaw_rate_) / (2 * max_yaw_rate_) * (SBusMsg::kMaxCmd - SBusMsg::kMinCmd) +
                  SBusMsg::kMinCmd);

  //   RCLCPP_INFO_THROTTLE(
  //       logger_, *node_->get_clock(), 1.0,
  //       "AUTONOMOUS MODE: yaw_rate: %f yaw_cmd: %d",
  //       yaw_rate, yaw_cmd);

  sbus_msg.setRollCommand(roll_cmd);
  sbus_msg.setPitchCommand(pitch_cmd);
  sbus_msg.setYawCommand(yaw_cmd);

  return sbus_msg;
}

void SBusBridge::setBridgeState(const BridgeState &desired_bridge_state)
{
  switch(desired_bridge_state)
  {
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
      RCLCPP_WARN(logger_, "Wanted to switch to unknown bridge state, setting to OFF");
      bridge_state_ = BridgeState::OFF;
  }
}

void SBusBridge::armBridge()
{
  std::lock_guard<std::mutex> main_lock(arm_mutex_);
  bridge_armed_ = true;
}

void SBusBridge::disarmBridge()
{
  std::lock_guard<std::mutex> main_lock(arm_mutex_);
  bridge_armed_ = false;
}

bool SBusBridge::isBridgeArmed() const
{
  return bridge_armed_;
}

bool SBusBridge::loadParameters()
{
  node_->declare_parameter("port_name", std::string("/dev/ttyUSB0"));
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

  // Get parameters
  if(!node_->get_parameter("port_name", port_name_) ||
     !node_->get_parameter("control_command_timeout", control_command_timeout_) ||
     !node_->get_parameter("rc_timeout", rc_timeout_) || !node_->get_parameter("mass", mass_) ||
     !node_->get_parameter("disable_thrust_mapping", disable_thrust_mapping_) ||
     !node_->get_parameter("max_roll_rate", max_roll_rate_) ||
     !node_->get_parameter("max_pitch_rate", max_pitch_rate_) || !node_->get_parameter("max_yaw_rate", max_yaw_rate_) ||
     !node_->get_parameter("max_roll_angle", max_roll_angle_) ||
     !node_->get_parameter("max_pitch_angle", max_pitch_angle_) ||
     !node_->get_parameter("alpha_vbat_filter", alpha_vbat_filter_) ||
     !node_->get_parameter("perform_thrust_voltage_compensation", perform_thrust_voltage_compensation_) ||
     !node_->get_parameter("n_lipo_cells", n_lipo_cells_) ||
     !node_->get_parameter("thrust_vs_rpm_cof_a_", thrust_vs_rpm_cof_a_) ||
     !node_->get_parameter("thrust_vs_rpm_cof_b_", thrust_vs_rpm_cof_b_) ||
     !node_->get_parameter("thrust_vs_rpm_cof_c_", thrust_vs_rpm_cof_c_) ||
     !node_->get_parameter("rpm_vs_throttle_linear_coeff_a_", rpm_vs_throttle_linear_coeff_a_) ||
     !node_->get_parameter("rpm_vs_throttle_linear_coeff_b_", rpm_vs_throttle_linear_coeff_b_) ||
     !node_->get_parameter("rpm_vs_throttle_quadratic_coeff_a_", rpm_vs_throttle_quadratic_coeff_a_) ||
     !node_->get_parameter("rpm_vs_throttle_quadratic_coeff_b_", rpm_vs_throttle_quadratic_coeff_b_) ||
     !node_->get_parameter("rpm_vs_throttle_quadratic_coeff_c_", rpm_vs_throttle_quadratic_coeff_c_) ||
     !node_->get_parameter("use_body_rates", use_body_rates_))
  {
    RCLCPP_ERROR(logger_, "Failed to load one or more parameters.");
    return false;
  }
  return true;
}

}  // namespace sbus_bridge

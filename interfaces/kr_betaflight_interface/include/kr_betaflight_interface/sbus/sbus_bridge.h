#pragma once

#include <kr_mav_msgs/msg/so3_command.hpp>
#include "kr_betaflight_interface/sbus/sbus_msg.h"
#include "kr_betaflight_interface/protocol_bridge_base.h"
#include "kr_betaflight_interface/sbus/sbus_serial_port.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <Eigen/Geometry>
#include <atomic>
#include <mutex>
#include <thread>

namespace sbus_bridge
{

enum class BridgeState
{
  OFF,
  ARMING,
  AUTONOMOUS_FLIGHT,
  RC_FLIGHT,
  KILL
};

class SBusBridge : public ProtocolBridgeBase
{
 public:
  SBusBridge(const rclcpp::Node::SharedPtr &node);
  void controlCommandCallback(const kr_mav_msgs::msg::SO3Command::ConstPtr &msg, const Eigen::Quaterniond &odom_q) override;
  void armBridge() override;
  void disarmBridge() override;
  bool isBridgeArmed() const override;
  const ProtocolMsgBase* getLastProtocolMsg() const override { return last_msg_.get(); }
  virtual ~SBusBridge();

 private:
  void watchdogThread();

  void handleReceivedSbusMessage(const SBusMsg &received_sbus_msg);

  SBusMsg generateSBusMessageFromSO3Command(const kr_mav_msgs::msg::SO3Command::ConstPtr &control_command,
                                            const Eigen::Quaterniond &odom_q) const;

  void sendSBusMessageToSerialPort(const SBusMsg &sbus_msg);

  void setBridgeState(const BridgeState &desired_bridge_state);

  double thrust_model_kartik(double thrust) const;

  bool loadParameters();

  // Serial port interface
  std::unique_ptr<SBusSerialPort> serial_port_;
  std::unique_ptr<SBusMsg> last_msg_;
  // rclcpp Node
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  // Mutex for:
  // - bridge_state_
  // - control_mode_
  // - bridge_armed_
  // - time_last_active_control_command_received_
  // - time_last_rc_msg_received_
  // - arming_counter_
  // - time_last_sbus_msg_sent_
  // Also "setBridgeState" and "sendSBusMessageToSerialPort" should only be
  // called when "main_mutex_" is locked
  mutable std::mutex main_mutex_;
  mutable std::mutex arm_mutex_;
  // Mutex for:
  // - battery_voltage_
  // - time_last_battery_voltage_received_
  // Also "generateSBusMessageFromControlCommand" should only be called when
  // "battery_voltage_mutex_" is locked
  mutable std::mutex battery_voltage_mutex_;

  // Subscribers
  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr control_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_sub_;
  // ROS 2: arm_bridge_sub_ removed unless you have a custom message/type for it

  // Timer
  rclcpp::TimerBase::SharedPtr low_level_feedback_pub_timer_;

  // Watchdog
  std::thread watchdog_thread_;
  std::atomic_bool stop_watchdog_thread_;
  rclcpp::Time time_last_rc_msg_received_;
  rclcpp::Time time_last_sbus_msg_sent_;
  rclcpp::Time time_last_battery_voltage_received_;
  rclcpp::Time time_last_active_control_command_received_;

  BridgeState bridge_state_;
  ControlMode control_mode_;
  int arming_counter_;
  double battery_voltage_;

  // Safety flags
  bool bridge_armed_;
  bool rc_was_disarmed_once_;

  std::atomic_bool destructor_invoked_;

  // Parameters
  std::string port_name_;
  bool enable_receiving_sbus_messages_;

  double control_command_timeout_;
  double rc_timeout_;

  double mass_;

  bool disable_thrust_mapping_;

  double max_roll_rate_;
  double max_pitch_rate_;
  double max_yaw_rate_;

  double max_roll_angle_;
  double max_pitch_angle_;

  double alpha_vbat_filter_;
  bool perform_thrust_voltage_compensation_;
  int n_lipo_cells_;

  // Constants
  static constexpr double kLowLevelFeedbackPublishFrequency_ = 50.0;

  static constexpr int kSmoothingFailRepetitions_ = 5;

  static constexpr double kBatteryLowVoltagePerCell_ = 3.6;
  static constexpr double kBatteryCriticalVoltagePerCell_ = 3.4;
  static constexpr double kBatteryInvalidVoltagePerCell_ = 3.0;
  static constexpr double kBatteryVoltageTimeout_ = 1.0;

  double thrust_vs_rpm_cof_a_;
  double thrust_vs_rpm_cof_b_;
  double thrust_vs_rpm_cof_c_;
  double rpm_vs_throttle_linear_coeff_a_;
  double rpm_vs_throttle_linear_coeff_b_;
  double rpm_vs_throttle_quadratic_coeff_a_;
  double rpm_vs_throttle_quadratic_coeff_b_;
  double rpm_vs_throttle_quadratic_coeff_c_;

  bool use_body_rates_;
};

}  // namespace sbus_bridge

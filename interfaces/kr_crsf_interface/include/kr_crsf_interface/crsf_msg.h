#pragma once

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>

namespace crsf_bridge {

enum class ControlMode { NONE, ATTITUDE, BODY_RATES };
enum class ArmState { DISARMED, ARMED };

#pragma pack(push)
#pragma pack(1)
struct CrsfMsg {
  // Constants (update for CRSF protocol specifics)
  static constexpr int kNChannels = 16; // CRSF typically supports up to 16 channels
  static constexpr uint16_t kMinCmd = 172;   // CRSF min value (update as needed)
  static constexpr uint16_t kMeanCmd = 992;  // CRSF mid value (update as needed)
  static constexpr uint16_t kMaxCmd = 1811;  // CRSF max value (update as needed)

  rclcpp::Time timestamp;

  // Normal 11 bit channels (update for CRSF specifics if needed)
  uint16_t channels[kNChannels];

  // Digital channels (if supported by CRSF)
  bool digital_channel_1;
  bool digital_channel_2;

  // Flags
  bool frame_lost;
  bool failsafe;

  CrsfMsg();
  virtual ~CrsfMsg();

  void limitAllChannelsFeasible();
  void limitCrsfChannelFeasible(const int channel_idx);

  // Setting CRSF command helpers
  void setThrottleCommand(const uint16_t throttle_cmd);
  void setRollCommand(const uint16_t roll_cmd);
  void setPitchCommand(const uint16_t pitch_cmd);
  void setYawCommand(const uint16_t yaw_cmd);
  void setControlMode(const ControlMode& control_mode);
  void setControlModeAttitude();
  void setControlModeBodyRates();
  void setArmState(const ArmState& arm_state);
  void setArmStateArmed();
  void setArmStateDisarmed();

  // CRSF message check helpers
  bool isArmed() const;
  bool isKillSwitch() const;
  ControlMode getControlMode() const;
};
#pragma pack(pop)
}  // namespace crsf_bridge

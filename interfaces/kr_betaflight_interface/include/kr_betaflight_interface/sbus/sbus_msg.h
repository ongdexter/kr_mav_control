#pragma once
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include "kr_betaflight_interface/protocol_msg_base.h"

namespace sbus_bridge {

enum class ControlMode { NONE, ATTITUDE, BODY_RATES };
enum class ArmState { DISARMED, ARMED };

#pragma pack(push)
#pragma pack(1)
struct SBusMsg : public ProtocolMsgBase {
  // Constants
  static constexpr int kNChannels = 16;
  static constexpr uint16_t kMinCmd = 174;   // corresponds to 1000 on FC
  static constexpr uint16_t kMeanCmd = 992;  // corresponds to 1500 on FC
  static constexpr uint16_t kMaxCmd = 1811;  // corresponds to 2000 on FC

  rclcpp::Time timestamp;

  // Normal 11 bit channels
  uint16_t channels[kNChannels];

  // Digital channels (ch17 and ch18)
  bool digital_channel_1;
  bool digital_channel_2;

  // Flags
  bool frame_lost;
  bool failsafe;


  SBusMsg();
  virtual ~SBusMsg();

  uint16_t getChannel(int idx) const override { return channels[idx]; }
  void setChannel(int idx, uint16_t value) override { channels[idx] = value; }

//   sbus_bridge::SbusRosMessage toRosMessage() const;

  void limitAllChannelsFeasible();
  void limitSbusChannelFeasible(const int channel_idx);

  // Setting sbus command helpers
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

  // Sbus message check helpers
  bool isArmed() const;
  bool isKillSwitch() const;
  ControlMode getControlMode() const;
};
#pragma pack(pop)

}  // namespace sbus_bridge

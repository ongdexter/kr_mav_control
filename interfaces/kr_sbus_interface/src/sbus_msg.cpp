#include "kr_sbus_interface/sbus_msg.h"

#include "kr_sbus_interface/channel_mapping.h"

namespace sbus_bridge {

SBusMsg::SBusMsg()
    : timestamp(rclcpp::Clock().now()),
      digital_channel_1(false),
      digital_channel_2(false),
      frame_lost(false),
      failsafe(false) {
  for (int i = 0; i < kNChannels; i++) {
    channels[i] = kMeanCmd;
  }
}

SBusMsg::~SBusMsg() {}

void SBusMsg::limitAllChannelsFeasible() {
  for (uint8_t i = 0; i < kNChannels; i++) {
    limitSbusChannelFeasible(i);
  }
}

void SBusMsg::limitSbusChannelFeasible(const int channel_idx) {
  if (channel_idx < 0 || channel_idx >= kNChannels) {
    return;
  }

  if (channels[channel_idx] > kMaxCmd) {
    channels[channel_idx] = kMaxCmd;
  }
  if (channels[channel_idx] < kMinCmd) {
    channels[channel_idx] = kMinCmd;
  }
}

void SBusMsg::setThrottleCommand(const uint16_t throttle_cmd) {
  channels[channel_mapping::kThrottle] = throttle_cmd;
}

void SBusMsg::setRollCommand(const uint16_t roll_cmd) {
  channels[channel_mapping::kRoll] = roll_cmd;
}

void SBusMsg::setPitchCommand(const uint16_t pitch_cmd) {
  channels[channel_mapping::kPitch] = pitch_cmd;
}

void SBusMsg::setYawCommand(const uint16_t yaw_cmd) {
  channels[channel_mapping::kYaw] = yaw_cmd;
}

void SBusMsg::setControlMode(const ControlMode& control_mode) {
  if (control_mode == ControlMode::ATTITUDE) {
    channels[channel_mapping::kControlMode] = kMinCmd;
  } else if (control_mode == ControlMode::BODY_RATES) {
    channels[channel_mapping::kControlMode] = kMaxCmd;
  }
}

void SBusMsg::setControlModeAttitude() {
  setControlMode(ControlMode::ATTITUDE);
}

void SBusMsg::setControlModeBodyRates() {
  setControlMode(ControlMode::BODY_RATES);
}

void SBusMsg::setArmState(const ArmState& arm_state) {
  if (arm_state == ArmState::ARMED) {
    channels[channel_mapping::kArming] = kMaxCmd;
  } else {
    channels[channel_mapping::kArming] = kMinCmd;
  }
}

void SBusMsg::setArmStateArmed() { setArmState(ArmState::ARMED); }

void SBusMsg::setArmStateDisarmed() {
  setArmState(ArmState::DISARMED);
  // Should not be necessary but for safety we also set the throttle command
  // to the minimum
  setThrottleCommand(kMinCmd);
}

bool SBusMsg::isArmed() const {
  if (channels[channel_mapping::kArming] <= kMeanCmd) {
    return false;
  }

  return true;
}

bool SBusMsg::isKillSwitch() const {
  if (channels[channel_mapping::kKillSwitch] > kMeanCmd) {
    return false;
  }

  return true;
}

ControlMode SBusMsg::getControlMode() const {
  if (channels[channel_mapping::kControlMode] > kMeanCmd) {
    return ControlMode::BODY_RATES;
  }

  return ControlMode::ATTITUDE;
}

}  // namespace sbus_bridge

#include "kr_crsf_interface/crsf_msg.h"

#include "kr_crsf_interface/crsf_channel_mapping.h"

namespace crsf_bridge {

CrsfMsg::CrsfMsg()
    : timestamp(rclcpp::Clock().now()),
      digital_channel_1(false),
      digital_channel_2(false),
      frame_lost(false),
      failsafe(false) {
  for (int i = 0; i < kNChannels; i++) {
    channels[i] = kMeanCmd;
  }
}

CrsfMsg::~CrsfMsg() {}

void CrsfMsg::limitAllChannelsFeasible() {
  for (uint8_t i = 0; i < kNChannels; i++) {
    limitCrsfChannelFeasible(i);
  }
}

void CrsfMsg::limitCrsfChannelFeasible(const int channel_idx) {
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

void CrsfMsg::setThrottleCommand(const uint16_t throttle_cmd) {
  // Update with CRSF channel mapping if needed
  channels[0] = throttle_cmd;
}

void CrsfMsg::setRollCommand(const uint16_t roll_cmd) {
  channels[1] = roll_cmd;
}

void CrsfMsg::setPitchCommand(const uint16_t pitch_cmd) {
  channels[2] = pitch_cmd;
}

void CrsfMsg::setYawCommand(const uint16_t yaw_cmd) {
  channels[3] = yaw_cmd;
}

void CrsfMsg::setControlMode(const ControlMode& control_mode) {
  // Update with CRSF-specific logic if needed
  if (control_mode == ControlMode::ATTITUDE) {
    channels[4] = kMinCmd;
  } else if (control_mode == ControlMode::BODY_RATES) {
    channels[4] = kMaxCmd;
  }
}

void CrsfMsg::setControlModeAttitude() {
  setControlMode(ControlMode::ATTITUDE);
}

void CrsfMsg::setControlModeBodyRates() {
  setControlMode(ControlMode::BODY_RATES);
}

void CrsfMsg::setArmState(const ArmState& arm_state) {
  if (arm_state == ArmState::ARMED) {
    channels[5] = kMaxCmd;
  } else {
    channels[5] = kMinCmd;
  }
}

void CrsfMsg::setArmStateArmed() { setArmState(ArmState::ARMED); }

void CrsfMsg::setArmStateDisarmed() {
  setArmState(ArmState::DISARMED);
  setThrottleCommand(kMinCmd);
}

bool CrsfMsg::isArmed() const {
  return channels[5] > kMeanCmd;
}

bool CrsfMsg::isKillSwitch() const {
  return channels[6] <= kMeanCmd;
}

ControlMode CrsfMsg::getControlMode() const {
  if (channels[4] > kMeanCmd) {
    return ControlMode::BODY_RATES;
  }
  return ControlMode::ATTITUDE;
}

}  // namespace crsf_bridge

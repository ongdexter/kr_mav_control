#pragma once
#include <memory>
#include <kr_mav_msgs/msg/so3_command.hpp>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include "kr_betaflight_interface/protocol_msg_base.h"

class ProtocolBridgeBase {
public:
    virtual ~ProtocolBridgeBase() = default;
    virtual void controlCommandCallback(const kr_mav_msgs::msg::SO3Command::ConstPtr &msg, const Eigen::Quaterniond &odom_q) = 0;
    virtual void armBridge() = 0;
    virtual void disarmBridge() = 0;
    virtual bool isBridgeArmed() const = 0;
    virtual const ProtocolMsgBase* getLastProtocolMsg() const = 0;
};
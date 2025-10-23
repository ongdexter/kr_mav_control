#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Geometry>
#include <memory>
#include <chrono>

#include <kr_mav_msgs/msg/so3_command.hpp>
#include <kr_betaflight_interface/protocol_bridge_base.h>
#include <kr_betaflight_interface/crsf/crsf_bridge.h>
#include <kr_betaflight_interface/sbus/sbus_bridge.h>

using namespace std::chrono_literals;

class SO3CmdToBetaflight : public rclcpp::Node
{
public:
  explicit SO3CmdToBetaflight(const rclcpp::NodeOptions &options);
  
private:
  void so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void check_so3_cmd_timeout();
  void motors_on();
  void motors_off();
  
  // Controller state
  bool odom_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_;

  // Subscribers
  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Timeout handling
  double so3_cmd_timeout_;
  rclcpp::Time last_so3_cmd_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;

  // Protocol bridge (CRSF or SBUS)
  std::shared_ptr<ProtocolBridgeBase> bridge_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

SO3CmdToBetaflight::SO3CmdToBetaflight(const rclcpp::NodeOptions &options)
  : Node("so3cmd_to_bridge", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
    odom_set_(false),
    so3_cmd_set_(false)
{
  this->declare_parameter("so3_cmd_timeout", 0.25);
  this->declare_parameter("protocol_type", "crsf");
  so3_cmd_timeout_ = this->get_parameter("so3_cmd_timeout").as_double();
  std::string protocol_type = this->get_parameter("protocol_type").as_string();

  // Create subscribers
  so3_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::SO3Command>(
      "so3_cmd", 10,
      std::bind(&SO3CmdToBetaflight::so3_cmd_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "control_odom", 10,
      std::bind(&SO3CmdToBetaflight::odom_callback, this, std::placeholders::_1));
  
  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(0),
    [this, protocol_type]() {
      this->init_timer_->cancel();
      if (protocol_type == "crsf") {
        bridge_ = std::make_shared<crsf_bridge::CrsfBridge>(shared_from_this());
      } else if (protocol_type == "sbus") {
        bridge_ = std::make_shared<sbus_bridge::SBusBridge>(shared_from_this());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown protocol_type: %s", protocol_type.c_str());
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "SO3CmdToBetaflight component initialized with protocol: %s", protocol_type.c_str());
}

void SO3CmdToBetaflight::so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg)
{
  if (!so3_cmd_set_)
      so3_cmd_set_ = true;
  
  if (!bridge_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Bridge not initialized yet.");
    return;
  }

  // switch on motors
  if (msg->aux.enable_motors && !bridge_->isBridgeArmed() && odom_set_)
  {
      motors_on();
  }
  else if (!msg->aux.enable_motors)
  {
      motors_off();
  }

  // only send if valid odom
  if (odom_set_)
  {
    bridge_->controlCommandCallback(msg, odom_q_);
  }

  // Save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = this->now();
}

void SO3CmdToBetaflight::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  odom_q_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);
  // check if odom is valid
  if (std::isnan(odom_q_.w()) || std::isnan(odom_q_.x()) || std::isnan(odom_q_.y()) || std::isnan(odom_q_.z()))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received nan in odom orientation");
    odom_set_ = false;
  }
  else
  {
    odom_set_ = true;
  }
}

void SO3CmdToBetaflight::check_so3_cmd_timeout()
{
  if (so3_cmd_set_ && ((this->now() - last_so3_cmd_time_).seconds() >= so3_cmd_timeout_))
  {
    RCLCPP_WARN(this->get_logger(), "so3_cmd timeout. %f seconds since last command", 
                  (this->now() - last_so3_cmd_time_).seconds());
    
    auto last_so3_cmd_ptr = std::make_shared<kr_mav_msgs::msg::SO3Command>(last_so3_cmd_);
    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToBetaflight::motors_on()
{

  if (bridge_) bridge_->armBridge();
}

void SO3CmdToBetaflight::motors_off()
{
  if (bridge_) bridge_->disarmBridge();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SO3CmdToBetaflight)

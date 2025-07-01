#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Geometry>
#include <memory>
#include <chrono>

#include <kr_mav_msgs/msg/so3_command.hpp>
#include <kr_sbus_interface/sbus_bridge.h>

using namespace std::chrono_literals;

class SO3CmdToSBUS : public rclcpp::Node
{
public:
  explicit SO3CmdToSBUS(const rclcpp::NodeOptions &options);
  
private:
  void so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr pose);
  void check_so3_cmd_timeout();
  void motors_on();
  void motors_off();
  
  // Controller state
  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;

  // Subscribers
  rclcpp::Subscription<kr_mav_msgs::msg::SO3Command>::SharedPtr so3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Motor status
  int motor_status_;

  // Timeout handling[18][21]
  double so3_cmd_timeout_;
  rclcpp::Time last_so3_cmd_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;

  // SBUS bridge
  std::shared_ptr<sbus_bridge::SBusBridge> sbus_bridge_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

SO3CmdToSBUS::SO3CmdToSBUS(const rclcpp::NodeOptions &options)
  : Node("so3cmd_to_sbus", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
    odom_set_(false),
    imu_set_(false),
    so3_cmd_set_(false),
    motor_status_(0)
{
  this->declare_parameter("so3_cmd_timeout", 0.25);
  so3_cmd_timeout_ = this->get_parameter("so3_cmd_timeout").as_double();

  // Create subscribers
  so3_cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::SO3Command>(
      "so3_cmd", 10,
      std::bind(&SO3CmdToSBUS::so3_cmd_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&SO3CmdToSBUS::odom_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10,
      std::bind(&SO3CmdToSBUS::imu_callback, this, std::placeholders::_1));
  
  init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() {
            this->init_timer_->cancel();
            sbus_bridge_ = std::make_shared<sbus_bridge::SBusBridge>(shared_from_this());
        }
    );

  RCLCPP_INFO(this->get_logger(), "SO3CmdToSbus component initialized");
}

void SO3CmdToSBUS::so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg)
{
  if (!so3_cmd_set_)
      so3_cmd_set_ = true;

  // Switch on motors
  if (msg->aux.enable_motors && !sbus_bridge_->isBridgeArmed() && !motor_status_)
  {
      motors_on();
  }
  else if (!msg->aux.enable_motors)
  {
      motors_off();
  }

  sbus_bridge_->controlCommandCallback(msg, odom_q_);

  // Save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = this->now();
}

void SO3CmdToSBUS::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (!odom_set_)
    odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, 
                                odom->pose.pose.orientation.x,
                                odom->pose.pose.orientation.y, 
                                odom->pose.pose.orientation.z);

  check_so3_cmd_timeout();
}

void SO3CmdToSBUS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr pose)
{
  if (!imu_set_)
    imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, 
                              pose->orientation.y, pose->orientation.z);

  check_so3_cmd_timeout();
}

void SO3CmdToSBUS::check_so3_cmd_timeout()
{
  if (so3_cmd_set_ && ((this->now() - last_so3_cmd_time_).seconds() >= so3_cmd_timeout_))
  {
    RCLCPP_DEBUG(this->get_logger(), "so3_cmd timeout. %f seconds since last command", 
                  (this->now() - last_so3_cmd_time_).seconds());
    
    auto last_so3_cmd_ptr = std::make_shared<kr_mav_msgs::msg::SO3Command>(last_so3_cmd_);
    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToSBUS::motors_on()
{
  RCLCPP_INFO(this->get_logger(), "Motors on");
  sbus_bridge_->armBridge();
  motor_status_ = 1;
}

void SO3CmdToSBUS::motors_off()
{
  sbus_bridge_->disarmBridge();
  motor_status_ = 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SO3CmdToSBUS)

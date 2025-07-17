#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Geometry>
#include <memory>
#include <chrono>

#include <kr_mav_msgs/msg/so3_command.hpp>
#include <kr_crsf_interface/crsf_bridge.h>

using namespace std::chrono_literals;

class SO3CmdToCRSF : public rclcpp::Node
{
public:
  explicit SO3CmdToCRSF(const rclcpp::NodeOptions &options);
  
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

  // Timeout handling
  double so3_cmd_timeout_;
  rclcpp::Time last_so3_cmd_time_;
  kr_mav_msgs::msg::SO3Command last_so3_cmd_;

  // CRSF bridge
  std::shared_ptr<crsf_bridge::CrsfBridge> crsf_bridge_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

SO3CmdToCRSF::SO3CmdToCRSF(const rclcpp::NodeOptions &options)
  : Node("so3cmd_to_crsf", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
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
      std::bind(&SO3CmdToCRSF::so3_cmd_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&SO3CmdToCRSF::odom_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10,
      std::bind(&SO3CmdToCRSF::imu_callback, this, std::placeholders::_1));
  
  init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() {
            this->init_timer_->cancel();
            crsf_bridge_ = std::make_shared<crsf_bridge::CrsfBridge>(shared_from_this());
        }
    );

  RCLCPP_INFO(this->get_logger(), "SO3CmdToCRSF component initialized");
}

void SO3CmdToCRSF::so3_cmd_callback(const kr_mav_msgs::msg::SO3Command::SharedPtr msg)
{
  if (!so3_cmd_set_)
      so3_cmd_set_ = true;

  // Switch on motors
  if (msg->aux.enable_motors && !crsf_bridge_->isBridgeArmed() && !motor_status_)
  {
      motors_on();
  }
  else if (!msg->aux.enable_motors)
  {
      motors_off();
  }

  crsf_bridge_->controlCommandCallback(msg, odom_q_);

  // Save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = this->now();
}

void SO3CmdToCRSF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  odom_q_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z);
  odom_set_ = true;
}

void SO3CmdToCRSF::imu_callback(const sensor_msgs::msg::Imu::SharedPtr pose)
{
  imu_q_ = Eigen::Quaterniond(
      pose->orientation.w,
      pose->orientation.x,
      pose->orientation.y,
      pose->orientation.z);
  imu_set_ = true;
}

void SO3CmdToCRSF::check_so3_cmd_timeout()
{
  // Implement timeout logic if needed
}

void SO3CmdToCRSF::motors_on()
{
  crsf_bridge_->armBridge();
  motor_status_ = 1;
}

void SO3CmdToCRSF::motors_off()
{
  crsf_bridge_->disarmBridge();
  motor_status_ = 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SO3CmdToCRSF)

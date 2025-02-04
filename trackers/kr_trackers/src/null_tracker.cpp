// TODO: convert to ros2 compatible format

#include "kr_tracker_msgs/msg/tracker_status.hpp"
#include "kr_trackers/Tracker.hpp"
#include "rclcpp/rclcpp.hpp"

class NullTracker : public kr_trackers_manager::Tracker
{
 public:
  void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent);
  bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd);
  void Deactivate(void);

  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg);
  uint8_t status();
 protected:
  rclcpp::Logger logger_{rclcpp::get_logger("trackers_manager")};

};

void NullTracker::Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent) {
  RCLCPP_INFO(logger_, "Initialized NullTracker");
}

bool NullTracker::Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  return true;
}

void NullTracker::Deactivate(void) {}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr NullTracker::update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Return a null message (will not publish the position command)
  return std::make_shared<kr_mav_msgs::msg::PositionCommand>();
}

uint8_t NullTracker::status()
{
  return kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(NullTracker, kr_trackers_manager::Tracker);

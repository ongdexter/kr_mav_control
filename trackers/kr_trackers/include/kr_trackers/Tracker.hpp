#pragma once

#include "kr_mav_msgs/msg/position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace kr_trackers_manager
{
class Tracker
{
 public:
  virtual ~Tracker(void) {}

  /**
   * @brief Initialize the tracker. Should be used to get the params, construct the publishers and subscribers.
   *
   * @param parent pointer to the kr_trackers_manager node and can be used to read params and contruct action servers for trackers
   */
  virtual void Initialize(rclcpp_lifecycle::LifecycleNode::WeakPtr &parent) = 0;

  /**
   * @brief Activate the tracker. This indicates that the tracker should get ready to publish commands.
   *
   * @param cmd The last PositionCommand that was published, can be used to maintain continuity of commands when
   * switching trackers.
   *
   * @return Should return true if the tracker is ready to publish commands, else return false.
   */
  virtual bool Activate(const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd) = 0;

  /**
   * @brief Deactivate the tracker. This is called when the kr_trackers_manager switches to another tracker.
   */
  virtual void Deactivate(void) = 0;

  /**
   * @brief Get the current command output from the tracker.
   * Note that this function is still called even if the tracker has not been activated. This is for cases when the
   * tracker would want to use the previous robot odometry to compute current commands.
   *
   * @param msg The current odometry message which should be used by the tracker to generate the command.
   *
   * @return The PositionCommand message which would be published. If an uninitialized ConstSharedPtr is returned, then no
   * PositionCommand message would be published.
   */
  virtual kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(const nav_msgs::msg::Odometry::SharedPtr msg) = 0;

  /**
   * @brief Get status of the tracker. Only called when the tracker has been activated.
   *
   * @return The tracker status (see the options in the TrackerStatus message).
   */
  virtual uint8_t status() = 0;
};

}  // namespace kr_trackers_manager

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrackersManagerLifecycleManager : public rclcpp_lifecycle::LifecycleNode
{
 public:
  TrackersManagerLifecycleManager(const rclcpp::NodeOptions &options);
  ~TrackersManagerLifecycleManager();
  // LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

 private: 
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
  std::string node_name;
  std::string node_namespace;
  std::string service_change_state_name;
};

/*
 * @brief This is a lifecycle node manager for TrackersManager LifecycleNode. 
 */
TrackersManagerLifecycleManager::TrackersManagerLifecycleManager(const rclcpp::NodeOptions &options)
  : LifecycleNode("lifecycle_manager", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  this->declare_parameter("node_name", "kr_trackers_manager");

  node_namespace = this->get_namespace();
  node_name = this->get_parameter("node_name").as_string();
  service_change_state_name =  
    node_namespace +std::string("/") + node_name + std::string("/change_state");
  client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(service_change_state_name);

  RCLCPP_INFO(this->get_logger(), service_change_state_name.c_str());

  client_->wait_for_service();

  // Configure node
  RCLCPP_INFO(this->get_logger(), "Trying to configure node");

  auto transition = lifecycle_msgs::msg::Transition();
  transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  transition.label = std::string("configure");
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition = transition;
  auto future = client_->async_send_request(request);

  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if(response->success)
      RCLCPP_INFO(this->get_logger(), "Node configured successfully");
  } else {
      RCLCPP_WARN(this->get_logger(), "Failed to configure Node");
  }


  // TODO: If node not configured, then don't run activation sequence.

  // Activate node
  RCLCPP_INFO(this->get_logger(), "Trying to activate node");
  transition = lifecycle_msgs::msg::Transition();
  transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  transition.label = std::string("activate");
  request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition = transition;
  future = client_->async_send_request(request);
  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if(response->success)
      RCLCPP_INFO(this->get_logger(), "Node activated successfully");
  }
}

TrackersManagerLifecycleManager::~TrackersManagerLifecycleManager()
{}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TrackersManagerLifecycleManager)

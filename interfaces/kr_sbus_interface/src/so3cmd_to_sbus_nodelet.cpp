#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

#include <kr_sbus_interface/sbus_bridge.h>

class SO3CmdToSbus : public nodelet::Nodelet
{
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);
  void so3_cmd_to_sbus_interface(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void motors_on();
  void motors_off();

  // controller state
  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;

  int motor_status_;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  kr_mav_msgs::SO3Command last_so3_cmd_;

  sbus_bridge::SBusBridge sbus_bridge_;
};

void SO3CmdToSbus::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  if(!odom_set_)
    odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  if(so3_cmd_set_ && ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_))
  {
    ROS_DEBUG("so3_cmd timeout. %f seconds since last command", (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr = boost::make_shared<kr_mav_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToSbus::imu_callback(const sensor_msgs::Imu::ConstPtr &pose)
{
  if(!imu_set_)
    imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);

  if(so3_cmd_set_ && ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_))
  {
    ROS_DEBUG("so3_cmd timeout. %f seconds since last command", (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr = boost::make_shared<kr_mav_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToSbus::motors_on()
{
  // call the update 0 success
//   int res_update = sn_update_data();
//   if(res_update == -1)
//   {
//     ROS_ERROR("Likely failure in snav, ensure it is running");
//     return;
//   }

//   switch(snav_cached_data_struct_->general_status.props_state)
//   {
//     case SN_PROPS_STATE_NOT_SPINNING:
//     {
//       sn_send_thrust_att_ang_vel_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//       int ret = sn_spin_props();
//       if(ret == -1)
//         ROS_ERROR("Not able to send spinning command");
//       break;
//     }
//     case SN_PROPS_STATE_STARTING:
//     {
//       ROS_WARN("Propellers are starting to spin");
//       break;
//     }
//     case SN_PROPS_STATE_SPINNING:
//     {
//       ROS_INFO("Propellers are spinning");
//       motor_status_ = 1;
//       break;
//     }
//     default:
//       ROS_ERROR("SN_PROPS_STATE_UNKNOWN");
//   }
}

void SO3CmdToSbus::motors_off()
{
//   do
//   {
//     // call the update, 0-success
//     int res_update = sn_update_data();
//     if(res_update == -1)
//     {
//       ROS_ERROR("Likely failure in snav, ensure it is running");
//     }

//     // send minimum thrust and identity attitude
//     sn_send_thrust_att_ang_vel_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

//     // stop the props, 0-success
//     int r = sn_stop_props();
//     if(r == 0)
//       motor_status_ = 0;
//     else if(r == -1)
//     {
//       ROS_ERROR("Not able to send switch off propellers");
//     }
//   } while(snav_cached_data_struct_->general_status.props_state == SN_PROPS_STATE_SPINNING);

//   // check the propellers status
//   if(snav_cached_data_struct_->general_status.props_state == SN_PROPS_STATE_SPINNING)
//   {
//     ROS_ERROR("All the propellers are still spinning");
//     motor_status_ = 1;
//   }
}

void SO3CmdToSbus::so3_cmd_to_sbus_interface(const kr_mav_msgs::SO3Command::ConstPtr &msg)
{
  sbus_bridge_.controlCommandCallback(msg, odom_q_);
}

void SO3CmdToSbus::so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg)
{
  if(!so3_cmd_set_)
    so3_cmd_set_ = true;

  // switch on motors
  if(msg->aux.enable_motors && !motor_status_)
    motors_on();
  else if(!msg->aux.enable_motors)
    motors_off();

  so3_cmd_to_sbus_interface(msg);

  // save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = ros::Time::now();
}

void SO3CmdToSbus::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  // get param for so3 command timeout duration
  priv_nh.param("so3_cmd_timeout", so3_cmd_timeout_, 0.25);

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;
  motor_status_ = 0;
//   snav_cached_data_struct_ = NULL;

//   if(sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_cached_data_struct_) != 0)
//   {
//     ROS_ERROR("Failed to get flight data pointer!");
//     return;
//   }
  // attitude_raw_pub_ =
  // priv_nh.advertise<mavros_msgs::AttitudeTarget>("attitude_raw", 10);

  so3_cmd_sub_ =
      priv_nh.subscribe("so3_cmd", 10, &SO3CmdToSbus::so3_cmd_callback, this, ros::TransportHints().tcpNoDelay());

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3CmdToSbus::odom_callback, this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 10, &SO3CmdToSbus::imu_callback, this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3CmdToSbus, nodelet::Nodelet);

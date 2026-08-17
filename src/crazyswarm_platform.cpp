// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "as2_platform_crazyswarm/crazyswarm_platform.hpp"
#include <as2_core/utils/tf_utils.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

using namespace std::placeholders;

void CrazyswarmPlatform::configureParams()
{
  // Default cf_name is derived from the node namespace (strip leading '/')
  std::string default_cf_name = this->get_namespace();
  if (!default_cf_name.empty() && default_cf_name[0] == '/') {
    default_cf_name = default_cf_name.substr(1);
  }
  cf_name_ = this->getParameter<std::string>("cf_name", default_cf_name);
}

void CrazyswarmPlatform::init()
{
  base_frame_ = this->getBaseFrameId();
  odom_frame_ = this->getOdomFrameId();

  /*    PARAMETERS    */
  enable_multiranger_ = this->getParameter<bool>("multi_ranger_deck", false);
  connection_timeout_ = this->getParameter<double>("connection_timeout", 1.0);

  configureSensors();

  /*    CRAZYSWARM2 INTERFACE SETUP    */
  const std::string prefix = "/" + cf_name_;

  // Command publishers (absolute paths so they reach the Crazyswarm2 server
  // regardless of which AS2 namespace this node runs in)
  cmd_position_pub_ = this->create_publisher<crazyflie_interfaces::msg::Position>(
    prefix + "/cmd_position", rclcpp::QoS(10));
  cmd_velocity_world_pub_ = this->create_publisher<crazyflie_interfaces::msg::VelocityWorld>(
    prefix + "/cmd_velocity_world", rclcpp::QoS(10));
  cmd_hover_pub_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
    prefix + "/cmd_hover", rclcpp::QoS(10));

  // Service clients
  emergency_client_ = this->create_client<std_srvs::srv::Empty>(prefix + "/emergency");
  notify_stop_client_ = this->create_client<crazyflie_interfaces::srv::NotifySetpointsStop>(
    prefix + "/notify_setpoints_stop");
  arm_client_ = this->create_client<crazyflie_interfaces::srv::Arm>(prefix + "/arm");

  // Sensor subscriptions
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    prefix + "/odom", rclcpp::SensorDataQoS(),
    std::bind(&CrazyswarmPlatform::odomCB, this, _1));

  status_sub_ = this->create_subscription<crazyflie_interfaces::msg::Status>(
    prefix + "/status", rclcpp::SensorDataQoS(),
    std::bind(&CrazyswarmPlatform::statusCB, this, _1));

  if (enable_multiranger_) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      prefix + "/scan", rclcpp::SensorDataQoS(),
      std::bind(&CrazyswarmPlatform::scanCB, this, _1));
  }

  imu_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
    prefix + "/imu", rclcpp::SensorDataQoS(),
    std::bind(&CrazyswarmPlatform::imuCB, this, _1));

  /*  TIMERS */
  connection_check_timer_ = this->create_timer(
    std::chrono::milliseconds(500), [this]() {connectionCheckCB();});

  RCLCPP_INFO(
    this->get_logger(), "CrazyswarmPlatform: ns=%s cf_name=%s",
    this->get_namespace(), cf_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Waiting for Crazyswarm2 server on prefix: %s", prefix.c_str());
}

CrazyswarmPlatform::CrazyswarmPlatform()
: as2::AerialPlatform()
{
  configureParams();
  init();
}

CrazyswarmPlatform::CrazyswarmPlatform(const std::string & ns)
: as2::AerialPlatform(ns)
{
  configureParams();
  init();
}

CrazyswarmPlatform::CrazyswarmPlatform(
  const std::string & ns, const rclcpp::NodeOptions & options)
: as2::AerialPlatform(ns, options)
{
  configureParams();
  init();
}

void CrazyswarmPlatform::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Remap Crazyswarm2 frame IDs to AS2 frame names
  auto odom_msg = *msg;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;
  odom_estimate_ptr_->updateData(odom_msg);

  if (!is_connected_) {
    RCLCPP_INFO(this->get_logger(), "Connected to Crazyswarm2 server (%s)", cf_name_.c_str());
  }
  is_connected_ = true;
  last_odom_time_ = this->get_clock()->now();
}

void CrazyswarmPlatform::statusCB(const crazyflie_interfaces::msg::Status::SharedPtr msg)
{
  sensor_msgs::msg::BatteryState bat_msg;
  bat_msg.header.stamp = this->get_clock()->now();
  bat_msg.voltage = msg->battery_voltage;
  bat_msg.percentage = msg->battery_voltage / 4.2f * 100.0f;
  battery_sensor_ptr_->updateData(bat_msg);
}

void CrazyswarmPlatform::scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  auto scan_msg = *msg;
  scan_msg.header.frame_id = base_frame_;
  multi_ranger_sensor_ptr_->updateData(scan_msg);
}

void CrazyswarmPlatform::imuCB(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
{
  // Expected vars order: acc.x, acc.y, acc.z (G), gyro.x, gyro.y, gyro.z (deg/s)
  if (msg->values.size() < 6) {return;}

  static constexpr double G_TO_MS2 = 9.80665;
  static constexpr double DEG_TO_RAD = M_PI / 180.0;

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = msg->header.stamp;
  imu_msg.header.frame_id = base_frame_;

  imu_msg.linear_acceleration.x = msg->values[0] * G_TO_MS2;
  imu_msg.linear_acceleration.y = msg->values[1] * G_TO_MS2;
  imu_msg.linear_acceleration.z = msg->values[2] * G_TO_MS2;

  imu_msg.angular_velocity.x = msg->values[3] * DEG_TO_RAD;
  imu_msg.angular_velocity.y = msg->values[4] * DEG_TO_RAD;
  imu_msg.angular_velocity.z = msg->values[5] * DEG_TO_RAD;

  // Orientation is not provided by this log block
  imu_msg.orientation_covariance[0] = -1.0;

  imu_sensor_ptr_->updateData(imu_msg);
}

void CrazyswarmPlatform::connectionCheckCB()
{
  if (!is_connected_) {return;}
  const double elapsed = (this->get_clock()->now() - last_odom_time_).seconds();
  if (elapsed > connection_timeout_) {
    if (is_connected_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Lost connection to Crazyswarm2 (no odom for %.1fs) — FSM state: %d",
        elapsed, static_cast<int>(platform_info_msg_.status.state));
    }
    is_connected_ = false;
  }
}

void CrazyswarmPlatform::configureSensors()
{
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  odom_estimate_ptr_ =
    std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
  battery_sensor_ptr_ =
    std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>>("battery", this);
  if (enable_multiranger_) {
    multi_ranger_sensor_ptr_ =
      std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::LaserScan>>("lidar/scan", this);
  }
}

bool CrazyswarmPlatform::ownSendCommand()
{
  static rclcpp::Clock dbg_clk;
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), dbg_clk, 5000,
    "ownSendCommand: armed=%s offboard=%s mode=%d",
    getArmingState() ? "true" : "false",
    getOffboardMode() ? "true" : "false",
    static_cast<int>(getControlMode().control_mode));

  as2_msgs::msg::ControlMode platform_control_mode = this->getControlMode();

  if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED &&
    this->getArmingState() && is_connected_)
  {
    switch (platform_control_mode.control_mode) {
      case as2_msgs::msg::ControlMode::SPEED: {
          crazyflie_interfaces::msg::VelocityWorld msg;
          msg.header.stamp = this->get_clock()->now();
          msg.vel.x = this->command_twist_msg_.twist.linear.x;
          msg.vel.y = this->command_twist_msg_.twist.linear.y;
          msg.vel.z = this->command_twist_msg_.twist.linear.z;
          // Crazyswarm2 handles firmware yaw convention internally; pass ROS CCW-positive directly
          msg.yaw_rate = this->command_twist_msg_.twist.angular.z;
          cmd_velocity_world_pub_->publish(msg);
        } break;

      case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
          crazyflie_interfaces::msg::Hover msg;
          msg.header.stamp = this->get_clock()->now();
          msg.vx = this->command_twist_msg_.twist.linear.x;
          msg.vy = this->command_twist_msg_.twist.linear.y;
          msg.yaw_rate = this->command_twist_msg_.twist.angular.z;
          msg.z_distance = this->command_pose_msg_.pose.position.z;
          cmd_hover_pub_->publish(msg);
          RCLCPP_DEBUG(this->get_logger(), "Hover set to z: %f", msg.z_distance);
        } break;

      default:
        static rclcpp::Clock clock;
        RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2000, "Command/Control Mode not supported");
        return false;
    }
  } else if (platform_control_mode.control_mode == as2_msgs::msg::ControlMode::POSITION && // NOLINT
    platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE)
  {
    const auto eulerAngles = this->quaternion2Euler(this->command_pose_msg_.pose.orientation);
    crazyflie_interfaces::msg::Position msg;
    msg.header.stamp = this->get_clock()->now();
    msg.x = this->command_pose_msg_.pose.position.x;
    msg.y = this->command_pose_msg_.pose.position.y;
    msg.z = this->command_pose_msg_.pose.position.z;
    msg.yaw = static_cast<float>(eulerAngles[2] / 3.1416 * 180.0);
    cmd_position_pub_->publish(msg);
    static rclcpp::Clock pos_clk;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), pos_clk, 2000,
      "cmd_position → x=%.2f y=%.2f z=%.2f yaw=%.1f°",
      msg.x, msg.y, msg.z, msg.yaw);

  } else if (platform_control_mode.control_mode == as2_msgs::msg::ControlMode::UNSET) {
    auto req = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();
    req->remain_valid_millisecs = 0;
    notify_stop_client_->async_send_request(req);

  } else {
    static rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 2000, "Command/Control Mode not supported");
    return false;
  }
  return true;
}

bool CrazyswarmPlatform::ownSetArmingState(bool state)
{
  // Crazyflie has no explicit arming; disarming stops setpoints
  RCLCPP_INFO(
    this->get_logger(), "ownSetArmingState: %s (connected=%s, FSM state=%d)",
    state ? "ARM" : "DISARM", is_connected_ ? "true" : "false",
    static_cast<int>(platform_info_msg_.status.state));
  auto arm_req = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
  arm_req->arm = state;
  arm_client_->async_send_request(arm_req);

  if (!state) {
    RCLCPP_WARN(this->get_logger(), "STOP — sending notify_setpoints_stop");
    auto req = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();
    req->remain_valid_millisecs = 0;
    notify_stop_client_->async_send_request(req);
  }
  is_armed_ = state;
  return is_connected_;
}

bool CrazyswarmPlatform::ownSetOffboardControl(bool offboard) {return is_connected_;}

bool CrazyswarmPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // The crazyswarm2 setpoints are expressed in the local reference frame
  setCommandPoseFrameId(odom_frame_);
  setCommandTwistFrameId(odom_frame_);

  if (msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    switch (msg.control_mode) {
      case as2_msgs::msg::ControlMode::SPEED:
        RCLCPP_DEBUG(this->get_logger(), "SPEED ENABLED");
        break;

      case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
        RCLCPP_DEBUG(this->get_logger(), "SPEED_IN_A_PLANE ENABLED");
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
        return false;
    }
    return true;
  } else if (msg.control_mode == as2_msgs::msg::ControlMode::POSITION && // NOLINT
    msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE)
  {
    RCLCPP_DEBUG(this->get_logger(), "POSITION ENABLED");
    return true;

  } else if (msg.control_mode == as2_msgs::msg::ControlMode::UNSET) {
    return true;
  } else {
    return false;
  }
}

void CrazyswarmPlatform::ownKillSwitch()
{
  RCLCPP_ERROR(this->get_logger(), "KILL SWITCH — sending emergency to %s", cf_name_.c_str());
  emergency_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
}

void CrazyswarmPlatform::ownStopPlatform()
{
  RCLCPP_WARN(this->get_logger(), "ownStopPlatform — sending notify_setpoints_stop");
  auto req = std::make_shared<crazyflie_interfaces::srv::NotifySetpointsStop::Request>();
  req->remain_valid_millisecs = 0;
  notify_stop_client_->async_send_request(req);
}

Eigen::Vector3d CrazyswarmPlatform::quaternion2Euler(geometry_msgs::msg::Quaternion quat)
{
  Eigen::Quaterniond quaternion;
  quaternion.x() = quat.x;
  quaternion.y() = quat.y;
  quaternion.z() = quat.z;
  quaternion.w() = quat.w;
  Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
  return euler;
}

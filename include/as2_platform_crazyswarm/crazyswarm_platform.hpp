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

#ifndef AS2_PLATFORM_CRAZYSWARM__CRAZYSWARM_PLATFORM_HPP_
#define AS2_PLATFORM_CRAZYSWARM__CRAZYSWARM_PLATFORM_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"

#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/status.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

class CrazyswarmPlatform : public as2::AerialPlatform
{
  as2::tf::TfHandler tf_handler_;
  std::string base_frame_;
  std::string odom_frame_;

public:
  void init();
  CrazyswarmPlatform();
  explicit CrazyswarmPlatform(const std::string & ns);
  CrazyswarmPlatform(const std::string & ns, const rclcpp::NodeOptions & options);
  void configureParams();

  /*  --  AS2 FUNCTIONS --  */

  void configureSensors() override;

  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  bool ownSendCommand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

  /*  --  CRAZYSWARM2 CALLBACKS --  */

  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
  void statusCB(const crazyflie_interfaces::msg::Status::SharedPtr msg);
  void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imuCB(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg);
  void connectionCheckCB();

  /*  --  AUX FUNCTIONS --  */

  Eigen::Vector3d quaternion2Euler(geometry_msgs::msg::Quaternion quat);

private:
  std::string cf_name_;  // Crazyswarm2 drone name (e.g. "cf1"), replaces uri_

  bool is_connected_ = false;
  bool is_armed_ = false;
  bool enable_multiranger_ = false;
  double connection_timeout_ = 1.0;

  rclcpp::Time last_odom_time_;
  rclcpp::TimerBase::SharedPtr connection_check_timer_;

  /*  --  CRAZYSWARM2 PUBLISHERS --  */

  rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cmd_position_pub_;
  rclcpp::Publisher<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr cmd_velocity_world_pub_;
  rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr cmd_hover_pub_;

  /*  --  CRAZYSWARM2 SERVICE CLIENTS --  */

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr emergency_client_;
  rclcpp::Client<crazyflie_interfaces::srv::NotifySetpointsStop>::SharedPtr notify_stop_client_;
  rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr arm_client_;

  /*  --  CRAZYSWARM2 SUBSCRIPTIONS --  */

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr imu_sub_;

  /*  --  SENSORS --  */

  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_estimate_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::LaserScan>> multi_ranger_sensor_ptr_;
};

#endif  // AS2_PLATFORM_CRAZYSWARM__CRAZYSWARM_PLATFORM_HPP_

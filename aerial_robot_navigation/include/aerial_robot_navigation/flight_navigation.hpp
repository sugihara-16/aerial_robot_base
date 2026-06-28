/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *   3. Neither the name of the DRAGON Laboratory nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

/* ROS 2 */
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geodesy/utm.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/* Aerial robot packages */
#include "aerial_robot_estimation/state_estimation.h"
#include "aerial_robot_estimation/sensor/base_plugin.h"
#include "aerial_robot_navigation/trajectory/trajectory_reference/polynomial_trajectory.hpp"
#include "aerial_robot_navigation/util/joy_parser.hpp"
#include "aerial_robot_msgs/msg/flight_nav.hpp"
#include "spinal_msgs/msg/flight_config_cmd.hpp"
#include "spinal_msgs/msg/pwms.hpp"


namespace aerial_robot_navigation
{
// Control mode
enum control_mode
{
  POS_CONTROL_MODE,
  VEL_CONTROL_MODE,
  ACC_CONTROL_MODE
};
// Control frame
enum control_frame
{
  WORLD_FRAME,  // Global frame, e.g., ENU, mocap
  LOCAL_FRAME   // Head frame which is identical with IMU head direction
};

// Navigation state
enum flight_state
{
  ARM_OFF_STATE,
  START_STATE,
  ARM_ON_STATE,
  TAKEOFF_STATE,
  LAND_STATE,
  HOVER_STATE,
  STOP_STATE
};

class NavigationBase
{
public:
  NavigationBase();

  virtual ~NavigationBase() = default;

  static constexpr uint8_t POS_CONTROL_COMMAND = 0;
  static constexpr uint8_t VEL_CONTROL_COMMAND = 1;

  // Abnormal state
  static constexpr uint8_t LOW_BATTERY_STATE = 0x10;
  static constexpr uint8_t FORCE_LANDING_STATE = 0x11;

  // Battery check
  static constexpr float VOL_100P = 4.2;
  static constexpr float VOL_90P = 4.085;
  static constexpr float VOL_80P = 3.999;
  static constexpr float VOL_70P = 3.936;
  static constexpr float VOL_60P = 3.883;
  static constexpr float VOL_50P = 3.839;
  static constexpr float VOL_40P = 3.812;
  static constexpr float VOL_30P = 3.791;
  static constexpr float VOL_20P = 3.747;
  static constexpr float VOL_10P = 3.683;
  static constexpr float VOL_0P = 3.209;

  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double nav_loop_dt);

  virtual void update();

  rclcpp::Publisher<spinal_msgs::msg::FlightConfigCmd>::SharedPtr getFlightConfigPublisher()
  {
    return flight_config_pub_;
  }

  inline void setNaviState(const uint8_t state) { navi_state_ = state; }
  inline uint8_t getNaviState() { return navi_state_; }
  virtual bool isInflightState();

  inline void setTeleopFlag(bool teleop_flag) { teleop_flag_ = teleop_flag; }
  inline bool getTeleopFlag() { return teleop_flag_; }

  inline void setInitHeight(double height) { init_height_ = height; }
  inline const double getInitHeight() const { return init_height_; }

  inline void setControlframe(uint8_t frame_type) { control_frame_ = frame_type; }
  inline uint8_t getControlframe() { return (uint8_t)control_frame_; }

  void setEstimateMode(uint8_t estimate_mode) { estimate_mode_ = estimate_mode; }
  uint8_t getEstimateMode() { return estimate_mode_; }

  inline void setXyControlMode(uint8_t mode) { xy_control_mode_ = mode; }
  inline uint8_t getXyControlMode() { return (uint8_t)xy_control_mode_; }

  inline bool getXyVelModePosCtrlTakeoff() { return xy_vel_mode_pos_ctrl_takeoff_; }
  inline bool getForceLandingFlag() { return force_landing_flag_; }
  inline double getForceLandingStartTime() { return force_landing_start_time_; }

  inline void setTargetCogPos(KDL::Vector pos) { target_pos_ = pos; }
  inline void setTargetCogPos(double x, double y, double z) { setTargetCogPos(KDL::Vector(x, y, z)); }
  inline void addTargetCogPos(KDL::Vector diff_pos) { target_pos_ += diff_pos; }
  inline void addTargetCogPos(double x, double y, double z) { addTargetCogPos(KDL::Vector(x, y, z)); }
  inline void setTargetCogVel(KDL::Vector vel) { target_vel_ = vel; }
  inline void setTargetCogVel(double x, double y, double z) { setTargetCogVel(KDL::Vector(x, y, z)); }
  inline void setTargetZeroCogVel() { setTargetCogVel(KDL::Vector(0, 0, 0)); }
  inline void setTargetCogAcc(KDL::Vector vel) { target_acc_ = vel; }
  inline void setTargetCogAcc(double x, double y, double z) { setTargetCogAcc(KDL::Vector(x, y, z)); }
  inline void setTargetZeroCogAcc() { setTargetCogAcc(KDL::Vector(0, 0, 0)); }

  inline void setTargetCogRoll(float value) { target_rpy_.x(value); }
  inline void setTargetCogOmega(KDL::Vector omega) { target_omega_ = omega; }
  inline void setTargetCogOmega(double x, double y, double z) { setTargetCogOmega(KDL::Vector(x, y, z)); }
  inline void setTargetZeroCogOmega() { setTargetCogOmega(KDL::Vector(0, 0, 0)); }
  inline void setTargetCogOmegaX(float value) { target_omega_.x(value); }
  inline void setTargetCogPitch(float value) { target_rpy_.y(value); }
  inline void setTargetCogOmegaY(float value) { target_omega_.y(value); }
  inline void setTargetCogYaw(float value) { target_rpy_.z(value); }
  inline void addTargetCogYaw(float value) { setTargetCogYaw(angles::normalize_angle(target_rpy_.z() + value)); }
  inline void setTargetCogOmegaZ(float value) { target_omega_.z(value); }
  inline void setTargetCogRPY(KDL::Vector value) { target_rpy_ = value; }
  inline void setTargetCogAngAcc(KDL::Vector acc) { target_ang_acc_ = acc; }
  inline void setTargetCogAngAcc(double x, double y, double z) { setTargetCogAngAcc(KDL::Vector(x, y, z)); }
  inline void setTargetZeroCogAngAcc() { setTargetCogAngAcc(KDL::Vector(0, 0, 0)); }
  inline void setTargetCogAngAccX(double value) { target_ang_acc_.x(value); }
  inline void setTargetCogAngAccY(double value) { target_ang_acc_.y(value); }
  inline void setTargetCogAngAccZ(double value) { target_ang_acc_.z(value); }

  inline void setTargetCogPosX(float value) { target_pos_.x(value); }
  inline void setTargetCogVelX(float value) { target_vel_.x(value); }
  inline void setTargetCogAccX(float value) { target_acc_.x(value); }
  inline void setTargetCogPosY(float value) { target_pos_.y(value); }
  inline void setTargetCogVelY(float value) { target_vel_.y(value); }
  inline void setTargetCogAccY(float value) { target_acc_.y(value); }
  inline void setTargetCogPosZ(float value) { target_pos_.z(value); }
  inline void setTargetCogVelZ(float value) { target_vel_.z(value); }
  inline void setTargetCogAccZ(float value) { target_acc_.z(value); }
  inline void addTargetCogPosZ(float value) { target_pos_ += KDL::Vector(0, 0, value); }

  inline KDL::Vector getTargetCogPos() { return target_pos_; }
  inline KDL::Vector getTargetCogVel() { return target_vel_; }
  inline KDL::Vector getTargetCogAcc() { return target_acc_; }
  inline KDL::Vector getTargetCogRPY() { return target_rpy_; }
  inline KDL::Vector getTargetCogOmega() { return target_omega_; }
  inline KDL::Vector getTargetCogAngularAcc() { return target_ang_acc_; }

  void generateNewTrajectory(std::vector<geometry_msgs::msg::PoseStamped> path);

protected:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<spinal_msgs::msg::FlightConfigCmd>::SharedPtr flight_config_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr flight_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Subscription<aerial_robot_msgs::msg::FlightNav>::SharedPtr flight_nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr single_goal_sub_, simple_move_base_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr flight_status_ack_sub_, stop_teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_sub_, start_sub_, land_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr force_landing_sub_, halt_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr ctrl_mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_stick_sub_;
  rclcpp::Subscription<spinal_msgs::msg::Pwms>::SharedPtr motor_pwms_sub_;

  std::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;

  bool param_verbose_;

  uint8_t navi_state_;

  double nav_loop_dt_;
  int control_frame_;
  int estimate_mode_;
  bool force_att_control_flag_;
  bool trajectory_mode_;
  double force_landing_start_time_;

  double takeoff_height_;
  double init_height_;
  double land_height_;
  double land_descend_vel_;

  int xy_control_mode_;
  int prev_xy_control_mode_;
  bool xy_vel_mode_pos_ctrl_takeoff_;

  double takeoff_xy_pos_tolerance_;
  double takeoff_z_pos_tolerance_;
  double hover_convergent_start_time_;
  double hover_convergent_duration_;
  double land_check_start_time_;
  double land_check_duration_;
  double trajectory_reset_time_;
  double trajectory_reset_duration_;
  double teleop_reset_time_;
  double teleop_reset_duration_;
  double z_convergent_thresh_;
  double xy_convergent_thresh_;
  double land_pos_convergent_thresh_;
  double land_vel_convergent_thresh_;
  bool require_spinal_ready_for_arm_;
  bool spinal_ready_seen_;
  double spinal_ready_timeout_;
  double last_spinal_msg_time_;

  KDL::Vector target_pos_, target_vel_, target_acc_;
  KDL::Vector target_rpy_, target_omega_, target_ang_acc_;

  /* Teleoperation */
  bool teleop_flag_;
  bool xy_control_flag_;
  bool force_landing_flag_;
  bool check_joy_stick_heart_beat_;
  bool joy_stick_heart_beat_;

  double max_teleop_xy_vel_;
  double max_teleop_z_vel_;
  double max_teleop_yaw_vel_;
  double max_teleop_rp_angle_;

  double joy_stick_deadzone_;
  double joy_stick_prev_time_;
  double joy_stick_heart_beat_duration_;
  double force_landing_to_halt_duration_;

  std::string teleop_local_frame_;

  /* Trajectory */
  double trajectory_mean_vel_;
  double trajectory_mean_yaw_rate_;
  double trajectory_min_duration_;
  bool enable_latch_yaw_trajectory_;
  std::shared_ptr<agi::MinJerkTrajectory> traj_generator_ptr_;

  /* GPS waypoint */
  bool gps_waypoint_;
  geographic_msgs::msg::GeoPoint target_wp_;
  double gps_waypoint_time_;
  double gps_waypoint_check_duration_;
  double gps_waypoint_threshold_;

  /* Auto velocity navigation */
  bool vel_based_waypoint_;
  double nav_vel_limit_;
  double vel_nav_threshold_;  // The range (board) to switch between vel_nav and pos_nav
  double vel_nav_gain_;

  /* Battery info */
  double low_voltage_thre_;
  bool low_voltage_flag_;
  double high_voltage_cell_thre_;
  bool high_voltage_flag_;
  int bat_cell_;
  double bat_resistance_;
  double bat_resistance_voltage_rate_;
  double hovering_current_;

  /* ROS API */
  virtual void rosParamInit();
  virtual void naviCallback(const aerial_robot_msgs::msg::FlightNav::ConstSharedPtr msg);
  virtual void joyStickControl(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);
  void flightStatusAckCallback(std_msgs::msg::UInt8::ConstSharedPtr msg);
  void startCallback(std_msgs::msg::Empty::ConstSharedPtr msg) { motorArming(); }
  void takeoffCallback(std_msgs::msg::Empty::ConstSharedPtr msg) { startTakeoff(); }
  void landCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
  void haltCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
  void forceLandingCallback(std_msgs::msg::Empty::ConstSharedPtr msg);
  void stopTeleopCallback(std_msgs::msg::UInt8::ConstSharedPtr stop_msg);
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void singleGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void simpleMoveBaseGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void batteryCheckCallback(const std_msgs::msg::Float32::ConstSharedPtr msg);
  void motorPwmsCallback(const spinal_msgs::msg::Pwms::ConstSharedPtr msg);

  /* Basic navigation functions */
  virtual void halt() {}  // TODO: Currently does nothing! Remove?
  virtual void reset();
  void startTakeoff();
  void motorArming();
  bool spinalReadyForArming();
  virtual void updateLandCommand();

  void setTargetCogXyFromCurrentState();
  void setTargetCogZFromCurrentState();
  void setTargetCogYawFromCurrentState();

  /* Trajectory */
  void updatePoseFromTrajectory();
  KDL::Vector frameConversion(KDL::Vector origin_val, KDL::Rotation rot) { return rot * origin_val; }
  KDL::Vector frameConversion(KDL::Vector origin_val, float yaw)
  {
    return frameConversion(origin_val, KDL::Rotation::RPY(0, 0, yaw));
  }

  /* GPS waypoint tracking */
  KDL::Vector getDeltaPosFromGpsWaypoint();
  void gpsWaypointTracking();

  template <class T> void getParam(std::string param_name, T &param, T default_value, bool verbose = false)
  {
    std::string full_param_name = "navigation." + param_name;
    node_->get_parameter_or<T>(full_param_name, param, default_value);

    if (param_verbose_ || verbose)
    {
      rclcpp::Parameter ros_param(full_param_name, param);
      RCLCPP_INFO(node_->get_logger(), "[%s] %s: %s", node_->get_namespace(), param_name.c_str(),
                  ros_param.value_to_string().c_str());
    }
  }
};
}

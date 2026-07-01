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
#include <gimbalrotor/gimbalrotor_navigation.h>

#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace aerial_robot_navigation
{
GimbalrotorNavigator::GimbalrotorNavigator()
  : NavigationBase(),
    prev_rotation_stamp_(0.0),
    eq_cog_world_(false),
    baselink_rot_change_thresh_(0.02),
    baselink_rot_pub_interval_(0.1)
{
  curr_target_baselink_rot_.setRPY(0.0, 0.0, 0.0);
  final_target_baselink_rot_.setRPY(0.0, 0.0, 0.0);
}

void GimbalrotorNavigator::initialize(rclcpp::Node::SharedPtr node,
                                      std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  NavigationBase::initialize(node, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = node_->create_publisher<spinal_msgs::msg::DesireCoord>("desire_coordinate", 1);
  final_target_baselink_rot_sub_ = node_->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "final_target_baselink_rot", rclcpp::SystemDefaultsQoS(),
      std::bind(&GimbalrotorNavigator::targetBaselinkRotCallback, this, std::placeholders::_1));
  final_target_baselink_rpy_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "final_target_baselink_rpy", rclcpp::SystemDefaultsQoS(),
      std::bind(&GimbalrotorNavigator::targetBaselinkRPYCallback, this, std::placeholders::_1));
  prev_rotation_stamp_ = node_->now().seconds();
}

void GimbalrotorNavigator::update()
{
  NavigationBase::update();

  if (!trajectory_mode_)
  {
    if (node_->get_clock()->now().seconds() > teleop_reset_time_)
    {
      setTargetCogOmegaX(0.0);
      setTargetCogOmegaY(0.0);
    }

    setTargetCogRoll(std::clamp(getTargetCogRPY().x() + getTargetCogOmega().x() * nav_loop_dt_, -max_teleop_rp_angle_,
                                max_teleop_rp_angle_));
    setTargetCogPitch(std::clamp(getTargetCogRPY().y() + getTargetCogOmega().y() * nav_loop_dt_, -max_teleop_rp_angle_,
                                 max_teleop_rp_angle_));
  }

  baselinkRotationProcess();
}

void GimbalrotorNavigator::reset()
{
  NavigationBase::reset();

  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0.0, 0.0, 0.0);
  final_target_baselink_rot_.setRPY(0.0, 0.0, 0.0);
  robot_model_->setCogDesireOrientation(0.0, 0.0, 0.0);
}

void GimbalrotorNavigator::targetBaselinkRotCallback(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg)
{
  tf2::fromMsg(msg->quaternion, final_target_baselink_rot_);
  target_omega_ = KDL::Vector::Zero();

  if (getTargetCogRPY().z() != 0.0)
  {
    curr_target_baselink_rot_.setRPY(0.0, 0.0, getTargetCogRPY().z());
    eq_cog_world_ = true;
  }
}

void GimbalrotorNavigator::targetBaselinkRPYCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_ = KDL::Vector::Zero();
}

void GimbalrotorNavigator::naviCallback(const aerial_robot_msgs::msg::FlightNav::ConstSharedPtr msg)
{
  if (getNaviState() != HOVER_STATE) return;

  NavigationBase::naviCallback(msg);

  if (msg->roll_nav_mode == aerial_robot_msgs::msg::FlightNav::POS_MODE)
  {
    setTargetCogRoll(msg->target_roll);
    setTargetCogOmegaX(0.0);
  }
  else if (msg->roll_nav_mode == aerial_robot_msgs::msg::FlightNav::VEL_MODE)
  {
    setTargetCogOmegaX(msg->target_omega_x);
    teleop_reset_time_ = node_->get_clock()->now().seconds() + teleop_reset_duration_;
  }

  if (msg->pitch_nav_mode == aerial_robot_msgs::msg::FlightNav::POS_MODE)
  {
    setTargetCogPitch(msg->target_pitch);
    setTargetCogOmegaY(0.0);
  }
  else if (msg->pitch_nav_mode == aerial_robot_msgs::msg::FlightNav::VEL_MODE)
  {
    setTargetCogOmegaY(msg->target_omega_y);
    teleop_reset_time_ = node_->get_clock()->now().seconds() + teleop_reset_duration_;
  }
}

void GimbalrotorNavigator::baselinkRotationProcess()
{
  tf2::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
  if (std::abs(delta_q.getAngle()) < 1.0e-9) return;

  if (node_->now().seconds() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
  {
    double angle = delta_q.getAngle();
    if (angle > M_PI) angle -= 2.0 * M_PI;

    if (std::abs(angle) > baselink_rot_change_thresh_)
    {
      tf2::Vector3 axis = delta_q.getAxis();
      if (axis.length2() > 1.0e-12)
      {
        curr_target_baselink_rot_ *= tf2::Quaternion(axis, std::copysign(baselink_rot_change_thresh_, angle));
      }
    }
    else
    {
      curr_target_baselink_rot_ = final_target_baselink_rot_;
    }
    curr_target_baselink_rot_.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(curr_target_baselink_rot_).getRPY(roll, pitch, yaw);
    robot_model_->setCogDesireOrientation(roll, pitch, yaw);

    spinal_msgs::msg::DesireCoord msg;
    msg.roll = static_cast<float>(roll);
    msg.pitch = static_cast<float>(pitch);
    msg.yaw = static_cast<float>(yaw);
    target_baselink_rpy_pub_->publish(msg);

    prev_rotation_stamp_ = node_->now().seconds();
  }
}

void GimbalrotorNavigator::rosParamInit()
{
  NavigationBase::rosParamInit();
  getParam<double>("baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);
  getParam<double>("baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1);
}
}  // namespace aerial_robot_navigation

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorNavigator, aerial_robot_navigation::NavigationBase)

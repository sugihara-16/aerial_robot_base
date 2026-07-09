// -*- mode: c++ -*-
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

#include <aerial_robot_navigation/flight_navigation.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <spinal_msgs/msg/desire_coord.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace aerial_robot_navigation
{
class GimbalrotorNavigator : public NavigationBase
{
public:
  GimbalrotorNavigator();
  ~GimbalrotorNavigator() override = default;

  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;

  void update() override;

private:
  rclcpp::Publisher<spinal_msgs::msg::DesireCoord>::SharedPtr target_baselink_rpy_pub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr final_target_baselink_rot_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr final_target_baselink_rpy_sub_;

  void baselinkRotationProcess();
  void rosParamInit() override;
  void targetBaselinkRotCallback(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg);
  void targetBaselinkRPYCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg);
  void naviCallback(const aerial_robot_msgs::msg::FlightNav::ConstSharedPtr msg) override;
  void reset() override;

  double prev_rotation_stamp_;
  tf2::Quaternion curr_target_baselink_rot_;
  tf2::Quaternion final_target_baselink_rot_;
  bool eq_cog_world_;
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;
};
}  // namespace aerial_robot_navigation

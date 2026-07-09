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

/* ROS 2 */
#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <sensor_msgs/msg/imu.hpp>

/* Aerial robot packages */
#include "aerial_robot_msgs/msg/states.hpp"
#include "spinal_msgs/msg/imu.hpp"


namespace sensor_plugin
{
class Imu : public sensor_plugin::SensorBase
{
public:
  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                          int index) override;

  ~Imu() {}
  Imu();

protected:
  rclcpp::Subscription<spinal_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_imu_pub_;
  rclcpp::Publisher<aerial_robot_msgs::msg::States>::SharedPtr state_pub_;

  int calib_count_, calib_max_count_;
  double calib_time_;
  double sensor_dt_; /* Sensor internal */
  bool use_msg_stamp_;


  /* Reconfigurable variable */
  double level_acc_noise_sigma_, z_acc_noise_sigma_;
  double level_acc_bias_noise_sigma_, z_acc_bias_noise_sigma_; /* Sigma for kf */

  KDL::Vector g_b_;       /* The *opposite* gravity vector in baselink frame */
  KDL::Vector omega_;     /* Omega both of body frame */
  KDL::Vector mag_;       /* Magnetometer of body frame */
  KDL::Vector acc_b_;     /* Acceleration in baselink frame */
  KDL::Rotation raw_rot_; /* Raw rotation matrix from IMU */
  /* Acceleration */
  std::array<KDL::Vector, 2> acc_w_;          /* Acceleration in world frame, for estimate_mode and expriment_mode */
  std::array<KDL::Vector, 2> acc_non_bias_w_; /* Acceleration without bias in world frame for estimate_mode and
                                                 expriment_mode */
  /* Acceleration bias */
  KDL::Vector acc_bias_b_;                /* Acceleration bias in baselink frame, only use z axis  */
  std::array<KDL::Vector, 2> acc_bias_w_; /* Acceleration bias in world frame for estimate_mode and expriment_mode*/

  aerial_robot_msgs::msg::States states_; /* for debug */

  /* Orientation */
  std::array<KDL::Rotation, 2> cog_rot_, base_rot_;

  virtual void imuCallback(const spinal_msgs::msg::Imu::SharedPtr msg);
  virtual void estimateProcess() override;

  void updateAcc();
  bool calibrateAcc();
  void setRotationalStates();
  void setTranslationalStates();

  void activateFuser() override;
  void fuse() override;

  void publish() override;
  void rosParamInit() override;
};
}

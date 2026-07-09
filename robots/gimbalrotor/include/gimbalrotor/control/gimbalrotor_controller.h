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

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <aerial_robot_control/PID/pose_pid_controller_base.hpp>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/utils/kdl_utils.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <spinal_msgs/msg/four_axis_command.hpp>
#include <spinal_msgs/msg/roll_pitch_yaw_terms.hpp>
#include <spinal_msgs/msg/torque_allocation_matrix_inv.hpp>

namespace aerial_robot_control
{
class GimbalrotorController : public PosePIDControllerBase
{
public:
  GimbalrotorController();
  ~GimbalrotorController() override = default;

  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator, double ctrl_loop_dt) override;

  bool update() override;
  void reset() override;

private:
  rclcpp::Publisher<spinal_msgs::msg::FourAxisCommand>::SharedPtr flight_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_control_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_vectoring_force_pub_;
  rclcpp::Publisher<spinal_msgs::msg::RollPitchYawTerms>::SharedPtr rpy_gain_pub_;
  rclcpp::Publisher<spinal_msgs::msg::TorqueAllocationMatrixInv>::SharedPtr torque_allocation_matrix_inv_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gimbal_dof_pub_;

  std::shared_ptr<GimbalrotorRobotModel> gimbalrotor_robot_model_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_full_thrust_;
  std::vector<double> target_gimbal_angles_;
  bool hovering_approximate_;
  Eigen::VectorXd target_vectoring_f_;
  Eigen::VectorXd target_vectoring_f_trans_;
  Eigen::VectorXd target_vectoring_f_rot_;
  Eigen::MatrixXd integrated_map_inv_trans_;
  Eigen::MatrixXd integrated_map_inv_rot_;
  double candidate_yaw_term_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;
  bool underactuate_;
  double target_roll_;
  double target_pitch_;

  void rosParamInit();
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
}  // namespace aerial_robot_control

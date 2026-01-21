/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief Function group to obtain optimization parameters

#include "joint_limit.hpp"

#include <limits>
#include <string>
#include <vector>

#include <tmc_utils/parameters.hpp>

namespace {
// The names of the cart joints from HSR-B onwards are fixed as odom_x/y/t
// It could be made changeable via parameters, but since I can't think of any cases where it would be changed, I'll go with constants
const std::vector<std::string> kOdomJoints = {"odom_x", "odom_y", "odom_t"};
}  // namespace

namespace hsrb_quick_path_optimizer {

bool GetLimit(const rclcpp::Node::SharedPtr& node,
              const std::string& type_name,
              const std::vector<std::string>& joint_names,
              Eigen::VectorXd& dst_limit) {
  dst_limit.resize(joint_names.size() + kOdomJoints.size());
  // Aligning with the writing style of tmc/hsrb_timeopt_ros config
  for (uint32_t i = 0; i < joint_names.size(); ++i) {
    dst_limit[i] = tmc_utils::GetParameter(node, joint_names[i] + "." + type_name, -1.0);
    if (dst_limit[i] < std::numeric_limits<double>::min()) {
      RCLCPP_ERROR(node->get_logger(), "Get parameter failure: %s.%s", joint_names[i].c_str(), type_name.c_str());
      return false;
    }
  }
  auto joint_num = joint_names.size();
  for (uint32_t i = 0; i < kOdomJoints.size(); ++i) {
    dst_limit[i + joint_num] = tmc_utils::GetParameter(node, kOdomJoints[i] + "." + type_name, -1.0);
    if (dst_limit[i + joint_num] < std::numeric_limits<double>::min()) {
      RCLCPP_ERROR(node->get_logger(), "Get parameter failure: %s.%s", kOdomJoints[i].c_str(), type_name.c_str());
      return false;
    }
  }
  return true;
}

// Obtain speed limit
bool GetVelocityLimit(const rclcpp::Node::SharedPtr& node,
                      const std::vector<std::string>& joint_names,
                      Eigen::VectorXd& dst_limit) {
    return GetLimit(node, "velocity", joint_names, dst_limit);
}

// Obtain acceleration limit
bool GetAccelerationLimit(const rclcpp::Node::SharedPtr& node,
                          const std::vector<std::string>& joint_names,
                          Eigen::VectorXd& dst_limit) {
  return GetLimit(node, "acceleration", joint_names, dst_limit);
}

}  // namespace hsrb_quick_path_optimizer

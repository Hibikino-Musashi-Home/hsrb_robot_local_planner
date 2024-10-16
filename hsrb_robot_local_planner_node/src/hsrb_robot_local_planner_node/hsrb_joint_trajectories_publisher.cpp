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

#include "hsrb_joint_trajectories_publisher.hpp"

#include <tmc_utils/parameters.hpp>

namespace {
std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> InitializePublisher(
    rclcpp::Node::SharedPtr node, const std::string& parameter_name, const std::string& default_controller_name) {
  const auto names = tmc_utils::GetParameter(node, parameter_name, std::vector<std::string>({default_controller_name}));
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> pubs;
  for (const auto& name : names) {
    pubs.emplace_back(std::make_shared<tmc_manipulation_util::JointTrajectoryPublisher>(node, name));
  }
  return pubs;
}

std::vector<std::string> ExractJointNames(
    const std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr>& pubs) {
  std::vector<std::string> names;
  for (const auto& pub : pubs) {
    const auto pub_names = pub->joint_names();
    names.insert(names.end(), pub_names.begin(), pub_names.end());
  }
  return names;
}

void Publish(std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> pubs,
             const trajectory_msgs::msg::JointTrajectory& trajectory) {
  for (const auto& pub : pubs) {
    pub->Publish(trajectory);
  }
}

}  // namespace

namespace hsrb_robot_local_planner_node {

HsrbJointTrajectoriesPublisher::HsrbJointTrajectoriesPublisher(rclcpp::Node::SharedPtr node) {
  head_trajetory_pubs_ = InitializePublisher(node, "head_trajectory_controllers", "head_trajectory_controller");
  arm_trajetory_pubs_ = InitializePublisher(node, "arm_trajectory_controllers", "arm_trajectory_controller");
  hand_trajetory_pubs_ = InitializePublisher(node, "hand_trajectory_controllers", "gripper_controller");
  base_trajetory_pub_ = std::make_shared<tmc_manipulation_util::JointTrajectoryPublisher>(
      node, "omni_base_controller");

  joint_names_.head_joints = ExractJointNames(head_trajetory_pubs_);
  joint_names_.arm_joints = ExractJointNames(arm_trajetory_pubs_);
  joint_names_.hand_joints = ExractJointNames(hand_trajetory_pubs_);
  joint_names_.base_coordinates = base_trajetory_pub_->joint_names();
}

void HsrbJointTrajectoriesPublisher::PublishTrajectory(
    const bool enable_head, const bool enable_arm, const bool enable_hand, const bool enable_base) {
  if (enable_head) Publish(head_trajetory_pubs_, head_trajectory_);
  if (enable_arm) Publish(arm_trajetory_pubs_, arm_trajectory_);
  if (enable_hand) Publish(hand_trajetory_pubs_, hand_trajectory_);
  if (enable_base) base_trajetory_pub_->Publish(base_trajectory_);
}

void HsrbJointTrajectoriesPublisher::PublishStopTrajectory() {
  for (const auto& pubs : {head_trajetory_pubs_, arm_trajetory_pubs_, {base_trajetory_pub_}}) {
    for (const auto& pub : pubs) {
      trajectory_msgs::msg::JointTrajectory msg;
      msg.joint_names = pub->joint_names();
      pub->Publish(msg);
    }
  }
}

}  // namespace hsrb_robot_local_planner_node

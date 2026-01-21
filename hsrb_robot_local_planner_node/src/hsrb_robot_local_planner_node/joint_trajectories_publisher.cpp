/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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

#include "joint_trajectories_publisher.hpp"

#include <tmc_utils/parameters.hpp>

namespace {

// Since the number of elements is relatively small, implement it simply
template <typename T>
bool HasCommonElement(const std::vector<T>& a, const std::vector<T>& b) {
  for (const auto& elem_a : a) {
    if (std::find(b.begin(), b.end(), elem_a) != b.end()) {
      return true;
    }
  }
  return false;
}

void PublishStopTrajectoryHelper(const std::vector<std::string>& target_joint_names,
                                 tmc_manipulation_util::JointTrajectoryPublisher::Ptr publisher,
                                 const rclcpp::Time& stamp) {
  const auto publisher_joints = publisher->joint_names();
  const auto do_publish = HasCommonElement(publisher_joints, target_joint_names);
  if (do_publish) {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = stamp;
    msg.joint_names = publisher->joint_names();
    publisher->Publish(msg);
  }
}

}  // namespace

namespace hsrb_robot_local_planner_node {

JointTrajectoriesPublisher::JointTrajectoriesPublisher(rclcpp::Node::SharedPtr node) {
  const auto controller_names = tmc_utils::GetParameter<std::vector<std::string>>(
      node, "controllers", std::vector<std::string>({"head_trajectory_controller",
                                                     "arm_trajectory_controller",
                                                     "gripper_controller"}));
  for (const auto& name : controller_names) {
    joint_trajectory_pubs_.emplace_back(
        std::make_shared<tmc_manipulation_util::JointTrajectoryPublisher>(node, name));
  }
  base_trajectory_pub_ = std::make_shared<tmc_manipulation_util::JointTrajectoryPublisher>(
      node, "omni_base_controller");
}

void JointTrajectoriesPublisher::PublishJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                                        const sensor_msgs::msg::JointState& current_state) {
  for (const auto& pub : joint_trajectory_pubs_) {
    const auto publisher_joints = pub->joint_names();
    const auto do_publish = HasCommonElement(publisher_joints, trajectory.joint_names);
    if (do_publish) pub->Publish(trajectory, current_state);
  }
}

void JointTrajectoriesPublisher::PublishStopTrajectory(
    const tmc_manipulation_types::TimedRobotTrajectory& last_trajectory) {
  PublishStopTrajectory(last_trajectory.joint_trajectory.joint_names);
}

void JointTrajectoriesPublisher::PublishStopTrajectory(const std::vector<std::string>& joint_names,
                                                       const rclcpp::Time& stamp) {
  for (const auto& pub : joint_trajectory_pubs_) {
    PublishStopTrajectoryHelper(joint_names, pub, stamp);
  }
}

void JointTrajectoriesPublisher::PublishBaseStopTrajectory(const rclcpp::Time& stamp) {
  trajectory_msgs::msg::JointTrajectory base_msg;
  base_msg.header.stamp = stamp;
  base_msg.joint_names = base_trajectory_pub_->joint_names();
  base_trajectory_pub_->Publish(base_msg);
}

}  // namespace hsrb_robot_local_planner_node

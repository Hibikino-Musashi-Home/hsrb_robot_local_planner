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
/// @file     hsrb_joint_trajectories_publisher.hpp
/// @brief    publish for hsrb joint trajectory
/// @author   Satoru Onoda
#ifndef HSRB_ROBOT_LOCAL_PLANNER_NODE_HSRB_JOINT_TRAJECTORIES_PUBLISHER_HPP
#define HSRB_ROBOT_LOCAL_PLANNER_NODE_HSRB_JOINT_TRAJECTORIES_PUBLISHER_HPP

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_manipulation_util/joint_trajectory_publisher.hpp>

#include "utils.hpp"

namespace hsrb_robot_local_planner_node {

class HsrbJointTrajectoriesPublisher {
 public:
  using Ptr = std::shared_ptr<HsrbJointTrajectoriesPublisher>;

  explicit HsrbJointTrajectoriesPublisher(rclcpp::Node::SharedPtr node);
  ~HsrbJointTrajectoriesPublisher() = default;

  void PublishTrajectory(const bool enable_head = true,
                         const bool enable_arm = true,
                         const bool enable_hand = true,
                         const bool enable_base = true);

  void PublishStopTrajectory();

  void SetHeadTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    head_trajectory_ = trajectory;
  }

  void SetArmTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    arm_trajectory_ = trajectory;
  }

  void SetHandTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    hand_trajectory_ = trajectory;
  }

  void SetBaseTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    base_trajectory_ = trajectory;
  }

  HsrbJointNames GetJointNames() const { return joint_names_; }

 private:
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> head_trajetory_pubs_;
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> arm_trajetory_pubs_;
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> hand_trajetory_pubs_;
  tmc_manipulation_util::JointTrajectoryPublisher::Ptr base_trajetory_pub_;

  trajectory_msgs::msg::JointTrajectory head_trajectory_;
  trajectory_msgs::msg::JointTrajectory arm_trajectory_;
  trajectory_msgs::msg::JointTrajectory hand_trajectory_;
  trajectory_msgs::msg::JointTrajectory base_trajectory_;

  HsrbJointNames joint_names_;
};
}  // namespace hsrb_robot_local_planner_node

#endif  // HSRB_ROBOT_LOCAL_PLANNER_NODE_HSRB_JOINT_TRAJECTORIES_PUBLISHER_HPP

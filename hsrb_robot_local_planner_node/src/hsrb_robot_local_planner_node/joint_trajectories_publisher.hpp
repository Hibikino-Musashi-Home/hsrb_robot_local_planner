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
#ifndef HSRB_ROBOT_LOCAL_PLANNER_NODE_JOINT_TRAJECTORIES_PUBLISHER_HPP
#define HSRB_ROBOT_LOCAL_PLANNER_NODE_JOINT_TRAJECTORIES_PUBLISHER_HPP

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_manipulation_util/joint_trajectory_publisher.hpp>

#include "utils.hpp"

namespace hsrb_robot_local_planner_node {

class JointTrajectoriesPublisher {
 public:
  using Ptr = std::shared_ptr<JointTrajectoriesPublisher>;

  explicit JointTrajectoriesPublisher(rclcpp::Node::SharedPtr node);
  ~JointTrajectoriesPublisher() = default;

  void PublishJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                              const sensor_msgs::msg::JointState& current_state);

  void PublishBaseTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    base_trajectory_pub_->Publish(trajectory);
  }
  std::vector<std::string> base_coordinates() const {
    return base_trajectory_pub_->joint_names();
  }

  void PublishStopTrajectory(const tmc_manipulation_types::TimedRobotTrajectory& last_trajectory);
  void PublishStopTrajectory(const std::vector<std::string>& joint_names, const rclcpp::Time& stamp = rclcpp::Time(0));
  void PublishBaseStopTrajectory(const rclcpp::Time& stamp = rclcpp::Time(0));

 private:
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> joint_trajectory_pubs_;
  tmc_manipulation_util::JointTrajectoryPublisher::Ptr base_trajectory_pub_;
};

}  // namespace hsrb_robot_local_planner_node

#endif  // HSRB_ROBOT_LOCAL_PLANNER_NODE_JOINT_TRAJECTORIES_PUBLISHER_HPP

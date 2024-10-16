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
#ifndef HSRB_ROBOT_LOCAL_PLANNER_NODE_TEST_UTILS_HPP
#define HSRB_ROBOT_LOCAL_PLANNER_NODE_TEST_UTILS_HPP

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_planning_msgs/msg/range_joint_constraint.hpp>
#include <tmc_planning_msgs/msg/robot_local_planner_status.hpp>
#include <tmc_utils/caching_subscriber.hpp>

namespace hsrb_robot_local_planner_node {

tmc_planning_msgs::msg::RangeJointConstraint GenerateRangeJointConstraint() {
  tmc_planning_msgs::msg::RangeJointConstraint rjc;
  rjc.header.frame_id = "odom";
  // RobotlocalplannerNodebase :: Generateinitialstate () is in the order of Arm, Hand, HEAD, so keep it in
  rjc.min.joint_state.name = {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                               "wrist_roll_joint", "hand_motor_joint", "head_pan_joint", "head_tilt_joint"};
  rjc.min.joint_state.position = {0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rjc.min.multi_dof_joint_state.joint_names = {"world_joint"};
  geometry_msgs::msg::Transform transform;
  transform.rotation.w = 1.0;
  rjc.min.multi_dof_joint_state.transforms = {transform};
  rjc.max = rjc.min;
  return rjc;
}

class RobotLocalPlannerStatusSubscriber {
 public:
  using Ptr = std::shared_ptr<RobotLocalPlannerStatusSubscriber>;

  explicit RobotLocalPlannerStatusSubscriber(const rclcpp::Node::SharedPtr& node)
      : clock_(node->get_clock()) {
    sub_ = node->create_subscription<tmc_planning_msgs::msg::RobotLocalPlannerStatus>(
        "hsrb_robot_local_planner/planner_status", 1,
        std::bind(&RobotLocalPlannerStatusSubscriber::Callback, this, std::placeholders::_1));
  }

  bool WaitFor(int32_t expected, std::function<void()> spin_func) {
    const auto timeout = clock_->now() + rclcpp::Duration::from_seconds(1.0);
    while (rclcpp::ok()) {
      if (IsSubscribed() && GetValue().planner_status == expected) {
        return true;
      }
      if (clock_->now() > timeout) {
        return false;
      }
      spin_func();
    }
    return false;
  }

  bool IsSubscribed() const { return !msgs_.empty(); }
  tmc_planning_msgs::msg::RobotLocalPlannerStatus GetValue() const { return msgs_.back(); }

  std::vector<rclcpp::Time> ExtractStatusStamps(const std::vector<std::tuple<std::string, int32_t>>& target_statuses) {
    auto extracted = msgs_;
    for (const auto& [id, value] : target_statuses) {
      std::vector<tmc_planning_msgs::msg::RobotLocalPlannerStatus> current_extracted;
      for (const auto& planner_status : extracted) {
        for (const auto& constraints_status : planner_status.constraints_statuses) {
          if (constraints_status.id == id && constraints_status.value == value) {
            current_extracted.push_back(planner_status);
            break;
          }
        }
      }
      extracted.swap(current_extracted);
    }
    std::vector<rclcpp::Time> stamps;
    for (const auto& planner_status : extracted) {
      stamps.push_back(planner_status.header.stamp);
    }
    return stamps;
  }

 private:
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<tmc_planning_msgs::msg::RobotLocalPlannerStatus>::SharedPtr sub_;
  std::vector<tmc_planning_msgs::msg::RobotLocalPlannerStatus> msgs_;

  void Callback(const tmc_planning_msgs::msg::RobotLocalPlannerStatus::SharedPtr msg) {
    msgs_.push_back(*msg);
  }
};

}  // namespace hsrb_robot_local_planner_node
#endif  // HSRB_ROBOT_LOCAL_PLANNER_NODE_TEST_UTILS_HPP

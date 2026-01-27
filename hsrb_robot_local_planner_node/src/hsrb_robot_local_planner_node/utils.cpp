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
/// @file     utils.cpp
/// @brief    utility for local planner node.
/// @author   Yuta Watanabe, Satoru Onoda

#include "utils.hpp"

#include <algorithm>
#include <vector>

#include <tmc_manipulation_types/utils.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_local_planner_utils/common.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>
#include <tmc_robot_local_planner_utils/extractor.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

namespace {
void InsertInitialStateIntoTimedRobotTrajectory(
    const tmc_manipulation_types::RobotState& state,
    tmc_manipulation_types::TimedRobotTrajectory& inserted_trajectory) {
  tmc_manipulation_types::TimedJointTrajectoryPoint point;
  point.positions = state.joint_state.position;
  point.velocities = state.joint_state.velocity;
  point.time_from_start = 0.0;
  tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint multi_dof_point;
  multi_dof_point.transforms = state.multi_dof_joint_state.poses;
  multi_dof_point.velocities = state.multi_dof_joint_state.twist;

  tmc_manipulation_types::TwistSeq acc;
  acc.resize(state.multi_dof_joint_state.twist.size());
  for (uint32_t i = 0; i < acc.size(); ++i) {
    acc[i] = Eigen::VectorXd::Zero(6);
  }
  multi_dof_point.accelerations = acc;
  multi_dof_point.time_from_start = 0.0;

  inserted_trajectory.joint_trajectory.points.push_front(point);
  inserted_trajectory.multi_dof_joint_trajectory.points.push_front(multi_dof_point);
}

template <typename T>
bool IsSubset(const std::vector<T>& A, const std::vector<T>& B) {
  for (const auto& elem : A) {
    if (std::find(B.begin(), B.end(), elem) == B.end()) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace hsrb_robot_local_planner_node {

RobotLocalGoal::RobotLocalGoal(const tmc_planning_msgs::msg::RobotLocalGoal& msg) {
  id = msg.id;

  // ConvertConstraints assumes that constraints are empty output arguments, so initialization is necessary
  constraints = tmc_robot_local_planner::Constraints();
  tmc_robot_local_planner_utils::ConvertConstraints(msg.constraints, constraints);

  constraints_msg = msg.constraints;
  normalized_velocity = msg.normalized_velocity;

  enable_base = msg.enable_base;
}

bool RobotLocalGoal::IsEmpty() const {
  return (constraints.soft_link_constraints.empty() &&
          constraints.hard_link_constraints.empty() &&
          constraints.soft_joint_constraints.empty() &&
          constraints.hard_joint_constraints.empty());
}

void RobotLocalGoal::Clear() {
  constraints = tmc_robot_local_planner::Constraints();

  // Empty check with IsEmpty => The flow of use is assumed, but initialize just in case
  constraints_msg = tmc_planning_msgs::msg::Constraints();
  normalized_velocity = 0.0;

  enable_base = false;
}

DisplacementChecker::DisplacementChecker(const rclcpp::Node::SharedPtr& node)
    : fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel"),
      logger_(node->get_logger()) {
  const auto kinematics_type = tmc_utils::GetParameter<std::string>(
      node, "kinematics_type", "tmc_robot_kinematics_model/PinocchioWrapper");
  robot_ = fk_loader_.createSharedInstance(kinematics_type);

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
    robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  robot_->Initialize(robot_description);

  joint_displacement_threshold_ = tmc_utils::GetParameter<double>(node, "joint_displacement_threshold", 0.1);
  link_displacement_threshold_ = tmc_utils::GetParameter<double>(node, "link_displacement_threshold", 0.005);

  joint_stall_threshold_ = tmc_utils::GetParameter<double>(node, "joint_stall_threshold", 0.005);
  link_stall_threshold_ = tmc_utils::GetParameter<double>(node, "link_stall_threshold", 0.001);;

  joint_stall_check_range_ = tmc_utils::GetParameter<double>(node, "joint_stall_check_range",
                                                             3.0 * joint_displacement_threshold_);
  link_stall_check_range_ = tmc_utils::GetParameter<double>(node, "link_stall_check_range",
                                                            3.0 * link_displacement_threshold_);

  base_names_ = tmc_utils::GetParameter<std::vector<std::string>>(
      node, "base_names", std::vector<std::string>({"world_joint"}));

  pub_ = node->create_publisher<tmc_planning_msgs::msg::RobotDisplacements>(
      "~/displacements", tmc_utils::ReliableVolatileQoS());
}

void DisplacementChecker::UpdateConstraints(const tmc_robot_local_planner::Constraints& constraints) {
  std::lock_guard<std::mutex> lock(mutex_);
  constraints_ = constraints;
  previous_joint_displacement_ = 1.0e10;
  previous_link_displacement_ = 1.0e10;
}

DisplacementChecker::Result DisplacementChecker::ShouldComplete(
    const std::string& id, const tmc_manipulation_types::RobotState& robot_state) {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock) {
    return Result::kNotComplete;
  }

  // To issue information, calculate displacements first and then make judgments, as implemented
  tmc_planning_msgs::msg::RobotDisplacements displacements_msg;
  displacements_msg.id = id;

  double min_joint_displacement = 1.0e10;
  if (!constraints_.hard_joint_constraints.empty()) {
    for (const auto& joint_constraint : constraints_.hard_joint_constraints) {
      if (!IsSubset(joint_constraint->GetJointName(), robot_state.joint_state.name)) {
        RCLCPP_WARN(logger_, "Joint names in constraint are not subset of robot state");
        return Result::kInvalidRobotState;
      }
      const auto [disp_pos, disp_pose] = joint_constraint->CalcSeparateDisplacements(robot_state);

      tmc_planning_msgs::msg::JointDisplacement joint_displacement;
      joint_displacement.joint_names = joint_constraint->GetJointName();
      joint_displacement.joint_values.resize(disp_pos.size());
      Eigen::VectorXd::Map(&joint_displacement.joint_values[0], disp_pos.size()) = disp_pos;
      joint_displacement.base_names = base_names_;
      joint_displacement.base_values.resize(disp_pose.size());
      Eigen::VectorXd::Map(&joint_displacement.base_values[0], disp_pose.size()) = disp_pose;
      displacements_msg.joint_displacements.push_back(joint_displacement);

      min_joint_displacement = std::min(disp_pos.norm() + disp_pose.norm(), min_joint_displacement);
    }
  }

  if (!robot_state.multi_dof_joint_state.poses.empty()) {
    robot_->SetRobotTransform(robot_state.multi_dof_joint_state.poses[0]);
  }
  robot_->SetNamedAngle(robot_state.joint_state);

  double min_link_displacement = 1.0e10;
  if (!constraints_.hard_link_constraints.empty()) {
    for (const auto& link_constraint : constraints_.hard_link_constraints) {
      const auto origin_to_link = robot_->GetObjectTransform(link_constraint->GetLinkName());
      const auto displacements = link_constraint->CalcSeparateDisplacements(origin_to_link);

      tmc_planning_msgs::msg::LinkDisplacement link_displacement;
      link_displacement.xyz[0] = displacements[0];
      link_displacement.xyz[1] = displacements[1];
      link_displacement.xyz[2] = displacements[2];
      link_displacement.rpy[0] = displacements[3];
      link_displacement.rpy[1] = displacements[4];
      link_displacement.rpy[2] = displacements[5];
      displacements_msg.link_displacements.push_back(link_displacement);

      min_link_displacement = std::min(displacements.norm(), min_link_displacement);
    }
  }

  // Issue displacements here
  displacements_msg.min_displacement = std::min(min_joint_displacement, min_link_displacement);
  pub_->publish(displacements_msg);

  // Judgment starts from here
  if (!constraints_.hard_joint_constraints.empty()) {
    if (min_joint_displacement < joint_displacement_threshold_) {
      return Result::kComplete;
    }

    if (min_joint_displacement < joint_stall_check_range_ &&
        std::abs(min_joint_displacement - previous_joint_displacement_) < joint_stall_threshold_) {
      previous_joint_displacement_ = min_joint_displacement;
      return Result::kComplete;
    }
    previous_joint_displacement_ = min_joint_displacement;
  }

  if (!constraints_.hard_link_constraints.empty()) {
    if (min_link_displacement < link_displacement_threshold_) {
      return Result::kComplete;
    }

    if (min_link_displacement < link_stall_check_range_ &&
        std::abs(min_link_displacement - previous_link_displacement_) < link_stall_threshold_) {
      previous_link_displacement_ = min_link_displacement;
      return Result::kComplete;
    }
    previous_link_displacement_ = min_link_displacement;
  }

  return Result::kNotComplete;
}

tmc_manipulation_types::RobotState ConvertRobotStateStampedToRobotState(
    const tf2::Stamped<tmc_manipulation_types::RobotState>& state) {
  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state = state.joint_state;
  robot_state.multi_dof_joint_state = state.multi_dof_joint_state;
  return robot_state;
}

std::optional<tmc_manipulation_types::TimedRobotTrajectory> GetPreviousTrajectory(
    const tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory> previous_trajectory,
    const tmc_manipulation_types::RobotState initial_state,
    const rclcpp::Time& target_time) {
  tmc_manipulation_types::TimedRobotTrajectory extracted_trajectory;
  if (tmc_robot_local_planner_utils::ExtractTrajectoryAtTime(
          previous_trajectory, target_time, extracted_trajectory)) {
    // Insert the current posture at the beginning to avoid setting a single-point trajectory
    if (extracted_trajectory.joint_trajectory.points.size() == 1) {
      InsertInitialStateIntoTimedRobotTrajectory(initial_state, extracted_trajectory);
    }
    // Shift the activation time
    tmc_manipulation_types::TimedRobotTrajectory shifted_trajectory;
    tmc_robot_local_planner_utils::ShiftTrajectoryTimes(
        extracted_trajectory, extracted_trajectory.joint_trajectory.points[0].time_from_start,
        shifted_trajectory);
    return shifted_trajectory;
  }
  return std::nullopt;
}

tf2::TimePoint ConvertToTf2(const rclcpp::Time& stamp) {
  return std::chrono::time_point<std::chrono::system_clock>(
      std::chrono::nanoseconds(stamp.nanoseconds()));
}

RobotLocalPlannerStatusPublisher::RobotLocalPlannerStatusPublisher(const rclcpp::Node::SharedPtr& node)
    : clock_(node->get_clock()) {
  pub_ = node->create_publisher<tmc_planning_msgs::msg::RobotLocalPlannerStatus>(
      "~/planner_status", tmc_utils::ReliableVolatileQoS());

  error_code_map_ = {{tmc_robot_local_planner::RobotLocalPlannerErrorCode::kSuccess,
                      tmc_planning_msgs::msg::RobotLocalPlannerStatus::SUCCESS},
                     {tmc_robot_local_planner::RobotLocalPlannerErrorCode::kConstraintsEmpty,
                      tmc_planning_msgs::msg::RobotLocalPlannerStatus::CONSTRAINTS_EMPTY},
                     {tmc_robot_local_planner::RobotLocalPlannerErrorCode::kGenerationFailure,
                      tmc_planning_msgs::msg::RobotLocalPlannerStatus::GENERATION_FAILURE},
                     {tmc_robot_local_planner::RobotLocalPlannerErrorCode::kEvaluationFailure,
                      tmc_planning_msgs::msg::RobotLocalPlannerStatus::EVALUATION_FAILURE},
                     {tmc_robot_local_planner::RobotLocalPlannerErrorCode::kValidationFailure,
                      tmc_planning_msgs::msg::RobotLocalPlannerStatus::VALIDATION_FAILURE},
                     {tmc_robot_local_planner::RobotLocalPlannerErrorCode::kOptimizationFailure,
                      tmc_planning_msgs::msg::RobotLocalPlannerStatus::OPTIMIZATION_FAILURE}};
}

void RobotLocalPlannerStatusPublisher::UpdateConstraintsStatus(const std::string& name, int32_t status) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Searching in reverse order might be more efficient, but it shouldn't make a big difference
  for (auto& msg : constraints_statuses_) {
    if (msg.id == name) {
      // Completed, so overwriting is prohibited
      if (msg.value == tmc_planning_msgs::msg::ConstraintsStatus::SATISFIED ||
          msg.value == tmc_planning_msgs::msg::ConstraintsStatus::PREEMPTED ||
          msg.value == tmc_planning_msgs::msg::ConstraintsStatus::EMPTY) {
        return;
      }
      msg.last_updated_stamp = clock_->now();
      msg.value = status;
      return;
    }
  }
  tmc_planning_msgs::msg::ConstraintsStatus msg;
  msg.id = name;
  msg.last_updated_stamp = clock_->now();
  msg.value = status;
  constraints_statuses_.push_back(msg);
}

void RobotLocalPlannerStatusPublisher::Publish(tmc_robot_local_planner::RobotLocalPlannerErrorCode error_code) {
  PublishImpl_(error_code_map_.at(error_code));
}

void RobotLocalPlannerStatusPublisher::Publish(RobotLocalPlannerErrorCodeLocal error_code) {
  int32_t code;
  switch (error_code) {
    case RobotLocalPlannerErrorCodeLocal::kInvalidInputRobotState:
      code = tmc_planning_msgs::msg::RobotLocalPlannerStatus::INVALID_INPUT_ROBOT_STATE;
      break;
    default:
      code = -999;
      break;
  }
  PublishImpl_(code);
}

void RobotLocalPlannerStatusPublisher::PublishImpl_(int32_t error_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto current = clock_->now();
  for (auto it = constraints_statuses_.begin(); it != constraints_statuses_.end(); ) {
    // 10 seconds is arbitrary, but there doesn't seem to be a request to make it a changeable parameter, so we'll go with this implementation
    if ((current - it->last_updated_stamp) > rclcpp::Duration(10, 0)) {
      it = constraints_statuses_.erase(it);
    } else {
      ++it;
    }
  }

  tmc_planning_msgs::msg::RobotLocalPlannerStatus status_msg;
  status_msg.header.stamp = current;
  status_msg.planner_status = error_code;
  status_msg.constraints_statuses = constraints_statuses_;
  pub_->publish(status_msg);
}

}  // namespace hsrb_robot_local_planner_node

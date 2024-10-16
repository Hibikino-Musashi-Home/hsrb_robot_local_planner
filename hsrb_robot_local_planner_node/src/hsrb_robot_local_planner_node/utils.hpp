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
/// @file     utils.hpp
/// @brief    utility for local planner node.
/// @author   Yuta Watanabe, Satoru Onoda
#ifndef HSRB_ROBOT_LOCAL_PLANNER_NODE_UTILS
#define HSRB_ROBOT_LOCAL_PLANNER_NODE_UTILS

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_planning_msgs/msg/robot_displacements.hpp>
#include <tmc_planning_msgs/msg/robot_local_goal.hpp>
#include <tmc_planning_msgs/msg/robot_local_planner_status.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/constraints.hpp>
#include <tmc_robot_local_planner/robot_local_planner.hpp>

namespace hsrb_robot_local_planner_node {

struct HsrbJointNames {
  // head joint name
  tmc_manipulation_types::NameSeq head_joints;
  // arm joint name
  tmc_manipulation_types::NameSeq arm_joints;
  // hand joint name
  tmc_manipulation_types::NameSeq hand_joints;
  // base joint name
  tmc_manipulation_types::NameSeq base_coordinates;
};

struct RobotLocalGoal {
  std::string id;
  tmc_robot_local_planner::Constraints constraints;
  // I don't think it's very good, but I sometimes use the constraints ROS message directly.
  tmc_planning_msgs::msg::Constraints constraints_msg;

  double normalized_velocity;
  bool enable_arm;
  bool enable_head;
  bool enable_gripper;
  bool enable_base;

  RobotLocalGoal() {}
  explicit RobotLocalGoal(const tmc_planning_msgs::msg::RobotLocalGoal& msg);

  bool IsEmpty() const;
  void Clear();
};

class DisplacementChecker {
 public:
  explicit DisplacementChecker(const rclcpp::Node::SharedPtr& node);

  // I'm going to determine if constraints stalled, so I will explicitly update constraints.
  void UpdateConstraints(const tmc_robot_local_planner::Constraints& constraints);
  bool ShouldComplete(const std::string& id, const tmc_manipulation_types::RobotState& robot_state);

 private:
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IRobotKinematicsModel> fk_loader_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  tmc_robot_local_planner::Constraints constraints_;

  double joint_displacement_threshold_;
  double link_displacement_threshold_;

  double previous_joint_displacement_;
  double previous_link_displacement_;

  double joint_stall_threshold_;
  double link_stall_threshold_;

  double joint_stall_check_range_;
  double link_stall_check_range_;

  std::vector<std::string> base_names_;

  rclcpp::Logger logger_;
  rclcpp::Publisher<tmc_planning_msgs::msg::RobotDisplacements>::SharedPtr pub_;

  std::mutex mutex_;
};

/// @brief Change the input orbit to HSRB orbit MSG
/// @param Trajectory input orbit
/// @param STAMP time stamp
/// @param head_trajectory_msg head output orbit
/// @param ARM_TRAJECTORY_MSG Output Orbit
/// @param Hand_trajectory_msg Hand Output Orbit
/// @param base_trajectory_msg Base output orbit
void RobotTrajectoryToHsrbTrajectoryMsg(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    const rclcpp::Time& stamp,
    const HsrbJointNames& hsrb_joint_names,
    trajectory_msgs::msg::JointTrajectory& head_trajectory_msg,
    trajectory_msgs::msg::JointTrajectory& arm_trajectory_msg,
    trajectory_msgs::msg::JointTrajectory& hand_trajectory_msg,
    trajectory_msgs::msg::JointTrajectory& base_trajectory_msg);

/// @brief Take out Robotstate from Robotstate with a time stamp
/// @param State with time stamp ROBOTSTATE
/// @return Robotstate
tmc_manipulation_types::RobotState ConvertRobotStateStampedToRobotState(
    const tf2::Stamped<tmc_manipulation_types::RobotState>& state);

std::optional<tmc_manipulation_types::TimedRobotTrajectory> GetPreviousTrajectory(
    const tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory> previous_trajectory,
    const tmc_manipulation_types::RobotState initial_state,
    const rclcpp::Time& target_time);

tf2::TimePoint ConvertToTf2(const rclcpp::Time& stamp);

class RobotLocalPlannerStatusPublisher {
 public:
  explicit RobotLocalPlannerStatusPublisher(const rclcpp::Node::SharedPtr& node);

  void UpdateConstraintsStatus(const std::string& name, int32_t status);
  void Publish(tmc_robot_local_planner::RobotLocalPlannerErrorCode error_code);

 private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<tmc_planning_msgs::msg::RobotLocalPlannerStatus>::SharedPtr pub_;
  std::map<tmc_robot_local_planner::RobotLocalPlannerErrorCode, int32_t> error_code_map_;

  std::vector<tmc_planning_msgs::msg::ConstraintsStatus> constraints_statuses_;

  std::mutex mutex_;
};

}  // namespace hsrb_robot_local_planner_node
#endif  // HSRB_ROBOT_LOCAL_PLANNER_NODE_UTILS

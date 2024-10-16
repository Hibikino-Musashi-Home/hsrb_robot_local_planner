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
/// @file     node.hpp
/// @brief    local planner node for HSRB.
/// @author   Yuta Watanabe, Satoru Onoda
#ifndef HSRB_ROBOT_LOCAL_PLANNER_NODE_NODE_HPP
#define HSRB_ROBOT_LOCAL_PLANNER_NODE_NODE_HPP

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tmc_planning_msgs/msg/robot_local_goal.hpp>
#include <tmc_robot_local_planner/robot_local_planner.hpp>
#include <tmc_robot_local_planner/trajectory_merger.hpp>
#include <tmc_robot_local_planner_utils/arrival_rate_calculator.hpp>

#include "arrival_percentage_publisher.hpp"
#include "hsrb_joint_trajectories_publisher.hpp"
#include "utils.hpp"

namespace hsrb_robot_local_planner_node {

/// @brief Robot Local Planner base class for HSR
class RobotLocalPlannerNodeBase : public rclcpp::Node {
 public:
  RobotLocalPlannerNodeBase();
  explicit RobotLocalPlannerNodeBase(const rclcpp::NodeOptions& options);

  virtual ~RobotLocalPlannerNodeBase() = default;

  void Run();
  void Run(std::function<bool()> interrupt);

 protected:
  virtual void InitializePlanner(const rclcpp::Node::SharedPtr& node) = 0;
  virtual void InitializeRosInterfaces(const rclcpp::Node::SharedPtr& node);
  virtual bool IsReady();

  using PlanResult = std::tuple<tmc_manipulation_types::TimedRobotTrajectory,
                                tmc_robot_local_planner::RobotLocalPlannerErrorCode>;
  virtual PlanResult PlanImpl(
      const std::optional<tmc_manipulation_types::TimedRobotTrajectory>& previous_trajectory,
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<std::string>& ignore_joints,
      const RobotLocalGoal& constraints) = 0;

 private:
  void Initialize();
  void Execute();

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Conditions of goals
  bool constraints_are_changed_;
  RobotLocalGoal constraints_;

  // Judgment of whether or not it has reached the goal
  std::shared_ptr<DisplacementChecker> displacement_checker_;
  bool use_current_state_for_displacement_;

  // Orbit planned with the previous loop
  tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory> current_trajectory_;

  // Parameter
  bool remove_completed_constraints_;
  double acceralation_limit_;

  // Subscribers
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr joint_state_;
  std::mutex joint_state_mutex_;

  void BaseControllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr base_controller_state_sub_;
  control_msgs::msg::JointTrajectoryControllerState::SharedPtr base_controller_state_;
  std::mutex base_controller_state_mutex_;

  void ConstraintCallback(const tmc_planning_msgs::msg::RobotLocalGoal::SharedPtr msg);
  rclcpp::Subscription<tmc_planning_msgs::msg::RobotLocalGoal>::SharedPtr constraint_sub_;
  std::mutex constraints_mutex_;

  // Publishers
  HsrbJointTrajectoriesPublisher::Ptr joint_trajectories_pub_;
  std::shared_ptr<RobotLocalPlannerStatusPublisher> status_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_empty_pub_;
  void PublishIsEmpty(bool is_empty) const;

  /// @brief Returns the state / position expected from the current ODOM and the speed
  /// @return RobotState
  tmc_manipulation_types::RobotState GenerateInitialState();

  /// @brief Calculate the bogie speed in the initial state
  /// @param ODOM Odometry
  /// @param ODOM_OUT After calculation
  void CalcInitOdomState(nav_msgs::msg::Odometry& odom_out);

  /// @brief Update the initial posture from the orbit to the posture after the target time
  /// @param target_time target time
  /// @param Initial_state posture after the target time
  /// @param Ref_STATE Robot state for termination judgment
  /// @param Connectable_Time Connected orbital point time
  void UpdateRobotstate(
      const rclcpp::Time& target_time,
      tmc_manipulation_types::RobotState& initial_state,
      tmc_manipulation_types::RobotState& ref_state,
      rclcpp::Time& connectable_time);

  /// @brief Delete several orbit points
  uint32_t DeleteTrajectoryFewPoints(const tmc_manipulation_types::RobotState& initial_state,
                                     const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const;
};

void RunNode(const std::shared_ptr<RobotLocalPlannerNodeBase>& node);


/// @brief Robot Local Planner implemented in Ros Action
class RobotLocalPlannerNodeWithAction : public RobotLocalPlannerNodeBase {
 public:
  RobotLocalPlannerNodeWithAction();
  explicit RobotLocalPlannerNodeWithAction(const rclcpp::NodeOptions& options);

  virtual ~RobotLocalPlannerNodeWithAction() = default;

 protected:
  void InitializePlanner(const rclcpp::Node::SharedPtr& node) override;
  bool IsReady() override;

  PlanResult PlanImpl(
      const std::optional<tmc_manipulation_types::TimedRobotTrajectory>& previous_trajectory,
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<std::string>& ignore_joints,
      const RobotLocalGoal& constraints) override;

 private:
  tmc_robot_local_planner::RobotLocalPlanner::Ptr robot_local_planner_;
  tmc_robot_local_planner::TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr trajectory_merger_;
};


/// Robot Local Planner that reads plug -ins of each component
class RobotLocalPlannerNodeWithPlugin : public RobotLocalPlannerNodeBase {
 public:
  RobotLocalPlannerNodeWithPlugin();
  explicit RobotLocalPlannerNodeWithPlugin(const rclcpp::NodeOptions& options);

  virtual ~RobotLocalPlannerNodeWithPlugin() = default;

 protected:
  void InitializePlanner(const rclcpp::Node::SharedPtr& node) override;

  PlanResult PlanImpl(
      const std::optional<tmc_manipulation_types::TimedRobotTrajectory>& previous_trajectory,
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<std::string>& ignore_joints,
      const RobotLocalGoal& constraints) override;

 private:
  tmc_robot_local_planner::RobotLocalPlannerPlugins::Loader plugin_loader_;
  tmc_robot_local_planner::RobotLocalPlannerComposition::Ptr robot_local_planner_;
  tmc_robot_local_planner::TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>::Ptr trajectory_merger_;
};

}  // namespace hsrb_robot_local_planner_node

#endif  // HSRB_ROBOT_LOCAL_PLANNER_NODE_NODE_HPP

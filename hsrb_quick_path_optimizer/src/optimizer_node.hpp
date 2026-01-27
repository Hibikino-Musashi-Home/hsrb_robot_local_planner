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
/// @brief Node providing time-optimized actions for HSR-B and later
#ifndef HSRB_QUICK_PATH_OPTIMIZER_OPTIMIZER_NODE_HPP_
#define HSRB_QUICK_PATH_OPTIMIZER_OPTIMIZER_NODE_HPP_

#include <memory>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tmc_planning_msgs/action/optimize_robot_trajectory.hpp>
#include <tmc_robot_local_planner/component_interfaces.hpp>
#include <tmc_utils/parameters.hpp>

#include "base_kinematics.hpp"
#include "trajectory_filter_adapter.hpp"

class ThreadPool;

namespace hsrb_quick_path_optimizer {

struct JointLimit;
struct InputTrajectory;

class OptimizerPluginCommon {
 public:
  OptimizerPluginCommon() = default;
  virtual ~OptimizerPluginCommon() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node);

  bool Reset();

  void UpdateBaseVelocities(const tmc_manipulation_types::TimedMultiDOFJointTrajectory& base_trajectory,
                            JointLimit& dst_joint_limit);

  void UpdateBaseAccelarations(const tmc_manipulation_types::TimedMultiDOFJointTrajectory& base_trajectory,
                               JointLimit& dst_joint_limit);

  ITrajectoryFilterAdapter::Ptr Optimize(std::function<bool()> interrupt,
                                         const InputTrajectory& trajectory,
                                         const JointLimit& joint_limit) const;

  ITrajectoryFilterAdapter::Ptr OptimizeViaStop(std::function<bool()> interrupt,
                                                const InputTrajectory& trajectory,
                                                const JointLimit& joint_limit) const;

  double sampling_interval_sec() const { return sampling_interval_sec_; }

 private:
  rclcpp::Node::SharedPtr node_;

  BaseKinematics::Ptr base_kinematics_;

  // Subscribers for obtaining current information
  // Strictly speaking, synchronization is better, but since the topic should be published at a high frequency, it is not a concern
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr joint_state_;

  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr odom_;

  std::mutex mutex_;

  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;
  tmc_utils::DynamicParameter<double>::Ptr optimize_timeout_;
  tmc_utils::DynamicParameter<double>::Ptr velocity_ratio_;
  tmc_utils::DynamicParameter<double>::Ptr acceleration_ratio_;
  tmc_utils::DynamicParameter<double>::Ptr acceleration_rate_for_stop_;

  std::chrono::time_point<std::chrono::system_clock> timeout_point_;
  double sampling_interval_sec_;
};


class OptimizerPlugin : public tmc_robot_local_planner::IOptimizer {
 public:
  OptimizerPlugin() = default;
  virtual ~OptimizerPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override;

  bool Optimize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in,
                std::function<bool()> interrupt,
                tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;
  bool Optimize(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override;

 private:
  OptimizerPluginCommon common_;

  rclcpp::Node::SharedPtr node_;

  // If true, force Result to Succeeded
  // Setting to continue operation even if optimization fails when the input trajectory contains non-optimal time
  bool is_force_succeeded_;
};


class OptimizerPluginMultiThread : public tmc_robot_local_planner::IOptimizer {
 public:
  OptimizerPluginMultiThread() = default;
  virtual ~OptimizerPluginMultiThread() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override;

  bool Optimize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in,
                std::function<bool()> interrupt,
                tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;
  bool Optimize(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override;

 private:
  OptimizerPluginCommon common_;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<ThreadPool> pool_;
  tmc_manipulation_types::TimedRobotTrajectory OptimizeImpl(
      std::function<ITrajectoryFilterAdapter::Ptr(const InputTrajectory&)> optimize_func,
      const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in);
};


class OptimizerNode : public rclcpp::Node {
 public:
  OptimizerNode();
  explicit OptimizerNode(const rclcpp::NodeOptions& options);
  virtual ~OptimizerNode() = default;

  // Want to use shared_from_this, so initialize after instance creation
  void Initialize();

 private:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<tmc_planning_msgs::action::OptimizeRobotTrajectory>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  rclcpp_action::GoalResponse GoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const tmc_planning_msgs::action::OptimizeRobotTrajectory::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle);
  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle);
  void Execute(const ServerGoalHandlePtr goal_handle);

  rclcpp_action::Server<tmc_planning_msgs::action::OptimizeRobotTrajectory>::SharedPtr server_;

  void PublishSucceeded(const ServerGoalHandlePtr goal_handle, const moveit_msgs::msg::RobotTrajectory& trajectory);
  void PublishAborted(const ServerGoalHandlePtr goal_handle, const moveit_msgs::msg::RobotTrajectory& trajectory);

  OptimizerPlugin impl_;
};

}  // namespace hsrb_quick_path_optimizer

#endif  // HSRB_QUICK_PATH_OPTIMIZER_OPTIMIZER_NODE_HPP_

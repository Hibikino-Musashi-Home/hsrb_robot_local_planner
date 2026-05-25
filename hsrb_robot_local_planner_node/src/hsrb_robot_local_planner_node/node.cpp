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
/// @file     node.cpp
/// @brief    local planner node for HSRB.
/// @author   Yuta Watanabe, Satoru Onoda

#include "node.hpp"

#include <algorithm>
#include <unordered_set>

//一時的に追加
#include <iomanip>
#include <sstream>
//

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_manipulation_types/utils.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_local_planner_utils/common.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>
#include <tmc_robot_local_planner_utils/extractor.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

namespace {
const double kConnectPoint = 0.2;           // 200msec/loop
const double kAcceralationLimit = 0.5;      // m/s^2
const double kTFTimeout = 0.05;
const double kPublishPeriodScale = 5;       // 200msec * 5 = 1.0s
const char* const kOriginFrame = "odom";
const char* const kBaseJointName = "world_joint";

//追加
std::string JoinStrings(const std::vector<std::string>& values, const std::size_t limit = 30) {
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < values.size() && i < limit; ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  if (values.size() > limit) {
    oss << ", ...";
  }
  oss << "]";
  return oss.str();
}

template <typename T>
std::string JoinValues(const std::vector<T>& values, const std::size_t limit = 30) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "[";
  for (std::size_t i = 0; i < values.size() && i < limit; ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << values[i];
  }
  if (values.size() > limit) {
    oss << ", ...";
  }
  oss << "]";
  return oss.str();
}

void LogRobotLocalGoalMsg(
    const rclcpp::Logger& logger,
    const tmc_planning_msgs::msg::RobotLocalGoal& goal,
    const char* label) {
  RCLCPP_INFO(
      logger,
      "[RLP_DEBUG] %s goal id=%s normalized_velocity=%.3f enable_arm=%d enable_head=%d enable_gripper=%d enable_base=%d",
      label,
      goal.id.c_str(),
      goal.normalized_velocity,
      goal.enable_arm,
      goal.enable_head,
      goal.enable_gripper,
      goal.enable_base);

  RCLCPP_INFO(
      logger,
      "[RLP_DEBUG] %s constraints count hard_joint=%zu hard_link=%zu soft_joint=%zu soft_link=%zu hard_path_link=%zu soft_path_joint=%zu",
      label,
      goal.constraints.hard_joint_constraints.size(),
      goal.constraints.hard_link_constraints.size(),
      goal.constraints.soft_joint_constraints.size(),
      goal.constraints.soft_link_constraints.size(),
      goal.constraints.hard_path_link_constraints.size(),
      goal.constraints.soft_path_joint_constraints.size());

  if (!goal.constraints.hard_joint_constraints.empty()) {
    const auto& hjc = goal.constraints.hard_joint_constraints.front();

    RCLCPP_INFO(
        logger,
        "[RLP_DEBUG] %s hard_joint[0] frame=%s min_names=%s min_pos=%s max_pos=%s min_mdof_names=%s min_mdof_transforms=%zu",
        label,
        hjc.header.frame_id.c_str(),
        JoinStrings(hjc.min.joint_state.name).c_str(),
        JoinValues(hjc.min.joint_state.position).c_str(),
        JoinValues(hjc.max.joint_state.position).c_str(),
        JoinStrings(hjc.min.multi_dof_joint_state.joint_names).c_str(),
        hjc.min.multi_dof_joint_state.transforms.size());
  }
}

void LogJointTrajectoryMsg(
    const rclcpp::Logger& logger,
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const char* label) {
  RCLCPP_INFO(
      logger,
      "[RLP_DEBUG] %s joint_trajectory names=%s points=%zu stamp=%d.%09u",
      label,
      JoinStrings(trajectory.joint_names).c_str(),
      trajectory.points.size(),
      trajectory.header.stamp.sec,
      trajectory.header.stamp.nanosec);

  if (!trajectory.points.empty()) {
    const auto& first = trajectory.points.front();
    const auto& last = trajectory.points.back();

    RCLCPP_INFO(
        logger,
        "[RLP_DEBUG] %s first_point positions=%s velocities=%s time=%d.%09u",
        label,
        JoinValues(first.positions).c_str(),
        JoinValues(first.velocities).c_str(),
        first.time_from_start.sec,
        first.time_from_start.nanosec);

    RCLCPP_INFO(
        logger,
        "[RLP_DEBUG] %s last_point positions=%s velocities=%s time=%d.%09u",
        label,
        JoinValues(last.positions).c_str(),
        JoinValues(last.velocities).c_str(),
        last.time_from_start.sec,
        last.time_from_start.nanosec);
  }
}
//ここまで
bool OverwriteRobotState(const tmc_manipulation_types::RobotState& source,
                         tmc_manipulation_types::RobotState& target_out) {
  for (uint32_t i = 0; i < source.joint_state.name.size(); ++i) {
    auto it = std::find(target_out.joint_state.name.begin(), target_out.joint_state.name.end(),
                        source.joint_state.name[i]);
    if (it == target_out.joint_state.name.end()) {
      return false;
    }
    auto j = std::distance(target_out.joint_state.name.begin(), it);
    target_out.joint_state.position[j] = source.joint_state.position[i];
    if (source.joint_state.velocity.size() == source.joint_state.position.size()) {
      target_out.joint_state.velocity[j] = source.joint_state.velocity[i];
    }
  }
  if (!source.multi_dof_joint_state.poses.empty()) {
    target_out.multi_dof_joint_state.names = source.multi_dof_joint_state.names;
    target_out.multi_dof_joint_state.poses = source.multi_dof_joint_state.poses;
    target_out.multi_dof_joint_state.twist = source.multi_dof_joint_state.twist;
  }
  return true;
}

std::vector<std::string> FindMissingStrings(const std::vector<std::string>& before,
                                            const std::vector<std::string>& after) {
  const std::unordered_set<std::string> after_set(after.begin(), after.end());
  std::vector<std::string> missing;
  for (const auto& s : before) {
    if (after_set.find(s) == after_set.end()) {
      missing.push_back(s);
    }
  }
  return missing;
}

}  // namespace

namespace hsrb_robot_local_planner_node {

RobotLocalPlannerNodeBase::RobotLocalPlannerNodeBase() : RobotLocalPlannerNodeBase(rclcpp::NodeOptions()) {}

RobotLocalPlannerNodeBase::RobotLocalPlannerNodeBase(const rclcpp::NodeOptions& options)
    : Node("hsrb_robot_local_planner", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      constraints_are_changed_(false),
      enable_base_prev_(false) {}

void RobotLocalPlannerNodeBase::Run() {
  Run([]() { return false; });
}

void RobotLocalPlannerNodeBase::Run(std::function<bool()> interrupt) {
  Initialize();

  rclcpp::WallRate rate(1.0 / kConnectPoint);
  while (rclcpp::ok() && !interrupt() && !IsReady()) {
    rate.sleep();
  }

  // TODO(Takeshita) タイマーで行いたかったが，上手くコールバックが回らなかったので諦めた
  // callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // timer_ = node->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(kConnectPoint * 1000)),
  //                                  std::bind(&HsrbRobotLocalPlannerNode::Execute, this), callback_group_);
  while (rclcpp::ok() && !interrupt()) {
    Execute();
    rate.sleep();
  }
}

void RobotLocalPlannerNodeBase::InitializeRosInterfaces(const rclcpp::Node::SharedPtr& node) {
  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", tmc_utils::BestEffortQoS(),
      std::bind(&RobotLocalPlannerNodeBase::JointStateCallback, this, std::placeholders::_1));
  base_controller_state_sub_ = node->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "base_trajectory_controller_state", tmc_utils::BestEffortQoS(),
      std::bind(&RobotLocalPlannerNodeBase::BaseControllerStateCallback, this, std::placeholders::_1));
  // pythonで送られてきたgoalを受け取る
  constraint_sub_ = node->create_subscription<tmc_planning_msgs::msg::RobotLocalGoal>(
      "~/constraints", tmc_utils::ReliableVolatileQoS(),
      std::bind(&RobotLocalPlannerNodeBase::ConstraintCallback, this, std::placeholders::_1));

  joint_trajectories_pub_ = std::make_shared<JointTrajectoriesPublisher>(node);
  status_pub_ = std::make_shared<RobotLocalPlannerStatusPublisher>(node);
  is_empty_pub_ = node->create_publisher<std_msgs::msg::Bool>(
      "~/is_constraints_empty", tmc_utils::ReliableVolatileQoS());
}

bool RobotLocalPlannerNodeBase::IsReady() {
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!joint_state_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Joint state is not ready");
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(base_controller_state_mutex_);
    if (!base_controller_state_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Base state is not ready");
      return false;
    }
  }
  return true;
}

void RobotLocalPlannerNodeBase::Initialize() {
  const auto node = shared_from_this();

  displacement_checker_ = std::make_shared<DisplacementChecker>(node);
  use_current_state_for_displacement_ = tmc_utils::GetParameter<bool>(
      node, "use_current_state_for_displacement", false);

  acceralation_limit_ = tmc_utils::GetParameter(node, "acceralation_limit", kAcceralationLimit);
  remove_completed_constraints_ = tmc_utils::GetParameter<bool>(node, "remove_completed_constraints", true);

  InitializePlanner(node);
  InitializeRosInterfaces(node);
}

void RobotLocalPlannerNodeBase::Execute() {
  RobotLocalGoal goal_constraints;
  bool constraints_are_changed;
  {
    std::lock_guard<std::mutex> lock(constraints_mutex_);
    goal_constraints = constraints_;
    constraints_are_changed = constraints_are_changed_;
    constraints_are_changed_ = false;
  }
  //追加
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[RLP_DEBUG] Execute goal id=%s changed=%d empty=%d normalized_velocity=%.3f enable_base=%d",
                       goal_constraints.id.c_str(),
                       constraints_are_changed,
                       goal_constraints.IsEmpty(),
                       goal_constraints.normalized_velocity,
                       goal_constraints.enable_base);
  //
  // Stop if the constraint is empty
  if (goal_constraints.IsEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "constraints are empty");
    if (constraints_are_changed) {
      joint_trajectories_pub_->PublishStopTrajectory(current_trajectory_);
      if (enable_base_prev_) {
        joint_trajectories_pub_->PublishBaseStopTrajectory();
      }
      current_trajectory_.setData(tmc_manipulation_types::TimedRobotTrajectory());
      status_pub_->UpdateConstraintsStatus(goal_constraints.id, tmc_planning_msgs::msg::ConstraintsStatus::EMPTY);
    }
    PublishIsEmpty(true);
    status_pub_->Publish(tmc_robot_local_planner::RobotLocalPlannerErrorCode::kConstraintsEmpty);
    return;
  }
  status_pub_->UpdateConstraintsStatus(goal_constraints.id, tmc_planning_msgs::msg::ConstraintsStatus::RUNNING);

  // 'initial' is the initial value of the motion plan, 'ref' is the robot state for termination judgment
  const auto target_time = this->now() + rclcpp::Duration::from_seconds(kConnectPoint);
  tmc_manipulation_types::RobotState initial_state;
  tmc_manipulation_types::RobotState ref_state;
  rclcpp::Time connectable_time;
  if (!UpdateRobotstate(target_time, initial_state, ref_state, connectable_time)) {
    PublishIsEmpty(false);
    status_pub_->Publish(RobotLocalPlannerErrorCodeLocal::kInvalidInputRobotState);
    return;
  }

  if (use_current_state_for_displacement_) {
    ref_state = GenerateInitialState();
  }
  const auto displacement_result = displacement_checker_->ShouldComplete(goal_constraints.id, ref_state);
  if (displacement_result == DisplacementChecker::Result::kComplete) {
    if (remove_completed_constraints_) {
      RCLCPP_INFO(this->get_logger(), "constraints are satisfied");
      current_trajectory_.setData(tmc_manipulation_types::TimedRobotTrajectory());

      std::lock_guard<std::mutex> lock(constraints_mutex_);
      constraints_.Clear();
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "constraints are satisfied");
    }
    PublishIsEmpty(true);
    status_pub_->UpdateConstraintsStatus(goal_constraints.id, tmc_planning_msgs::msg::ConstraintsStatus::SATISFIED);
    status_pub_->Publish(tmc_robot_local_planner::RobotLocalPlannerErrorCode::kConstraintsEmpty);
    return;
  } else if (displacement_result == DisplacementChecker::Result::kInvalidRobotState) {
    PublishIsEmpty(false);
    status_pub_->Publish(RobotLocalPlannerErrorCodeLocal::kInvalidInputRobotState);
    return;
  }
  if (!current_trajectory_.multi_dof_joint_trajectory.points.empty()) {
    const auto time_from_start = current_trajectory_.multi_dof_joint_trajectory.points.back().time_from_start;
    const auto trajectory_end_time = tf2::timeToSec(current_trajectory_.stamp_) + time_from_start;
    // If the remaining time is short, planning a new trajectory may result in a trajectory via unnecessary intermediate postures, so continue trajectory following for now
    // Success/failure judgment, replanning if the target is not reached will be done in the next loop
    if (trajectory_end_time < target_time.seconds()) {
      RCLCPP_INFO(this->get_logger(), "Remaining trajectory time (%f sec) is less than connect point (%f sec).",
                  time_from_start, kConnectPoint);
      // In the next loop, trajectory playback should be finished, so make it an empty trajectory
      current_trajectory_.setData(tmc_manipulation_types::TimedRobotTrajectory());
      PublishIsEmpty(false);
      status_pub_->Publish(tmc_robot_local_planner::RobotLocalPlannerErrorCode::kSuccess);
      return;
    }
  }

  PublishIsEmpty(false);

  // When switching constraints, exclude the previous trajectory from being selected
  std::optional<tmc_manipulation_types::TimedRobotTrajectory> previous_trajectory;
  if (!constraints_are_changed) {
    previous_trajectory = GetPreviousTrajectory(current_trajectory_, initial_state, target_time);
  }

  // Since it is used for trajectory issuance, hold it before trajectory generation
  sensor_msgs::msg::JointState joint_state_msg;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_state_msg = *joint_state_;
  }

  // Trajectory generation
  // const auto [trajectory, error_code] = PlanImpl(previous_trajectory, initial_state, goal_constraints);
  // status_pub_->Publish(error_code);

  //上記を変更
  const auto [trajectory, error_code] = PlanImpl(previous_trajectory, initial_state, goal_constraints);

  RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "[RLP_DEBUG] PlanImpl result goal id=%s error_code=%d joint_names=%s joint_points=%zu multi_dof_points=%zu",
      goal_constraints.id.c_str(),
      static_cast<int>(error_code),
      JoinStrings(trajectory.joint_trajectory.joint_names).c_str(),
      trajectory.joint_trajectory.points.size(),
      trajectory.multi_dof_joint_trajectory.points.size());

  status_pub_->Publish(error_code);
  //ここまで

  if (error_code != tmc_robot_local_planner::RobotLocalPlannerErrorCode::kSuccess) {
    RCLCPP_ERROR(this->get_logger(), "Fail to plan path.");
    if (constraints_are_changed) {
      std::lock_guard<std::mutex> lock(constraints_mutex_);
      constraints_are_changed_ = true;
    }
    return;
  }

  // To avoid excessive movement, extract only the trajectory of the last 1 second
  auto publish_solution = trajectory;
  tmc_robot_local_planner_utils::DeleteTrajectoryPointsAtTime(kConnectPoint * kPublishPeriodScale, publish_solution);

  // Convert to JointTrajectoryMsg and issue
  // trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
  // tmc_manipulation_types_bridge::TimedJointTrajectoryToJointTrajectoryMsg(
  //     publish_solution.joint_trajectory, joint_trajectory_msg);
  // joint_trajectory_msg.header.stamp = connectable_time;
  // joint_trajectories_pub_->PublishJointTrajectory(joint_trajectory_msg, joint_state_msg);
  //変更
  trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
  tmc_manipulation_types_bridge::TimedJointTrajectoryToJointTrajectoryMsg(
      publish_solution.joint_trajectory, joint_trajectory_msg);
  joint_trajectory_msg.header.stamp = connectable_time;
  
  LogJointTrajectoryMsg(this->get_logger(), joint_trajectory_msg, "publish_solution_all_joints");
  
  joint_trajectories_pub_->PublishJointTrajectory(joint_trajectory_msg, joint_state_msg);
  //ここまで

  if (goal_constraints.enable_base) {
    const auto base_trajectory = tmc_robot_local_planner_utils::ExtractMultiDOFJointTrajectory(
        publish_solution.multi_dof_joint_trajectory, joint_trajectories_pub_->base_coordinates());
    trajectory_msgs::msg::JointTrajectory base_trajectory_msg;
    tmc_manipulation_types_bridge::TimedJointTrajectoryToJointTrajectoryMsg(base_trajectory, base_trajectory_msg);
    base_trajectory_msg.header.stamp = connectable_time;

    uint32_t delete_point_num = DeleteTrajectoryFewPoints(initial_state, publish_solution);
    tmc_robot_local_planner_utils::DeleteTrajectoryPointsAtPoint(delete_point_num, base_trajectory_msg);
    joint_trajectories_pub_->PublishBaseTrajectory(base_trajectory_msg);
  }

  // If the joints used have changed, stop the joints that are no longer used
  const auto missing_joints = FindMissingStrings(
      current_trajectory_.joint_trajectory.joint_names, trajectory.joint_trajectory.joint_names);
  if (!missing_joints.empty()) {
    joint_trajectories_pub_->PublishStopTrajectory(missing_joints, connectable_time);
  }
  if (enable_base_prev_ && !goal_constraints.enable_base) {
    joint_trajectories_pub_->PublishBaseStopTrajectory(connectable_time);
  }

  current_trajectory_.setData(trajectory);
  current_trajectory_.stamp_ = ConvertToTf2(connectable_time);
  enable_base_prev_ = goal_constraints.enable_base;
}

void RobotLocalPlannerNodeBase::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  joint_state_ = msg;
}

void RobotLocalPlannerNodeBase::BaseControllerStateCallback(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(base_controller_state_mutex_);
  base_controller_state_ = msg;
}

void RobotLocalPlannerNodeBase::ConstraintCallback(const tmc_planning_msgs::msg::RobotLocalGoal::SharedPtr msg) {
  status_pub_->UpdateConstraintsStatus(constraints_.id, tmc_planning_msgs::msg::ConstraintsStatus::PREEMPTED);
  status_pub_->UpdateConstraintsStatus(msg->id, tmc_planning_msgs::msg::ConstraintsStatus::NOT_USED);

  // tmc_planning_msgs::msg::RobotLocalGoal goal_msg = *msg;
  // tmc_robot_local_planner_utils::TransformConstraints(kOriginFrame, kTFTimeout, tf_buffer_, goal_msg.constraints);
//上記の部分を変更
  tmc_planning_msgs::msg::RobotLocalGoal goal_msg = *msg;

  LogRobotLocalGoalMsg(this->get_logger(), goal_msg, "received");

  tmc_robot_local_planner_utils::TransformConstraints(kOriginFrame, kTFTimeout, tf_buffer_, goal_msg.constraints);

  LogRobotLocalGoalMsg(this->get_logger(), goal_msg, "after_transform");
  //

  RobotLocalGoal goal_impl(goal_msg);
  displacement_checker_->UpdateConstraints(goal_impl.constraints);

  {
    std::lock_guard<std::mutex> lock(constraints_mutex_);
    constraints_ = goal_impl;
    constraints_are_changed_ = true;
  }
}

void RobotLocalPlannerNodeBase::PublishIsEmpty(bool is_empty) const {
  std_msgs::msg::Bool is_empty_msg;
  is_empty_msg.data = is_empty;
  is_empty_pub_->publish(is_empty_msg);
}

tmc_manipulation_types::RobotState RobotLocalPlannerNodeBase::GenerateInitialState() {
  nav_msgs::msg::Odometry odom;
  CalcInitOdomState(odom);

  tmc_manipulation_types::JointState joint_state;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    tmc_manipulation_types_bridge::JointStateMsgToJointState(*joint_state_, joint_state);
  }

  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state = joint_state;
  robot_state.multi_dof_joint_state.names.push_back(kBaseJointName);
  robot_state.multi_dof_joint_state.poses.resize(1);
  tf2::fromMsg(odom.pose.pose, robot_state.multi_dof_joint_state.poses[0]);
  robot_state.multi_dof_joint_state.twist.resize(1);
  tf2::fromMsg(odom.twist.twist, robot_state.multi_dof_joint_state.twist[0]);

  return robot_state;
}

void RobotLocalPlannerNodeBase::CalcInitOdomState(nav_msgs::msg::Odometry& odom_out) {
  control_msgs::msg::JointTrajectoryControllerState base_state;
  {
    std::lock_guard<std::mutex> lock(base_controller_state_mutex_);
    base_state = *base_controller_state_;
  }
  odom_out.twist.twist.linear.x = base_state.actual.velocities[0];
  odom_out.twist.twist.linear.y = base_state.actual.velocities[1];
  odom_out.twist.twist.angular.z = base_state.actual.velocities[2];
  odom_out.pose.pose.position.x = base_state.actual.positions[0] + base_state.actual.velocities[0] * kConnectPoint;
  odom_out.pose.pose.position.y = base_state.actual.positions[1] + base_state.actual.velocities[1] * kConnectPoint;
  const double yaw_angle = base_state.actual.positions[2] + base_state.actual.velocities[2] * kConnectPoint;
  odom_out.pose.pose.orientation.z = std::sin(yaw_angle / 2.0);
  odom_out.pose.pose.orientation.w = std::cos(yaw_angle / 2.0);
}

bool RobotLocalPlannerNodeBase::UpdateRobotstate(
    const rclcpp::Time& target_time,
    tmc_manipulation_types::RobotState& initial_state,
    tmc_manipulation_types::RobotState& ref_state,
    rclcpp::Time& connectable_time) {
  initial_state = GenerateInitialState();

  tf2::Stamped<tmc_manipulation_types::RobotState> initial_state_stamped;
  if (tmc_robot_local_planner_utils::SearchConnectablePoint(
          current_trajectory_, target_time, connectable_time, initial_state_stamped)) {
    // If the robot state at target_time can be obtained from the previous trajectory, return it with initial/ref
    const auto initial_state_from_trajectory = ConvertRobotStateStampedToRobotState(initial_state_stamped);
    if (!OverwriteRobotState(initial_state_from_trajectory, initial_state)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint names in trajectory");
      return false;
    }
    ref_state = initial_state;
  } else {
    // If it cannot be obtained, perform motion planning from the current state
    // TODO(Takeshita) 動作計画は前回軌道の最終状態から行うのが正しい？ つまりrefとinitialは同じ？
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "non connectable point");
    ref_state = initial_state;
    if (!current_trajectory_.joint_trajectory.points.empty()) {
      tmc_manipulation_types::RobotState last_point_state;
      tmc_robot_local_planner_utils::GetLastPoint(current_trajectory_, last_point_state);
      if (!OverwriteRobotState(last_point_state, ref_state)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid joint names in trajectory");
        return false;
      }
    }
    connectable_time = target_time;
  }
  return true;
}

uint32_t RobotLocalPlannerNodeBase::DeleteTrajectoryFewPoints(
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const {
  // Distance traveled by acceleration/deceleration between kConnectPoint
  const double d = 0.5 * acceralation_limit_ * std::pow(kConnectPoint, 2.0);

  for (uint32_t i = 0; i < trajectory.multi_dof_joint_trajectory.points.size(); ++i) {
    const double trajectory_point_x = trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation().x();
    const double trajectory_point_y = trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation().y();
    const double initial_pose_x = initial_state.multi_dof_joint_state.poses[0].translation().x();
    const double initial_pose_y = initial_state.multi_dof_joint_state.poses[0].translation().y();
    const double diff_distance = std::sqrt(std::pow(trajectory_point_x - initial_pose_x, 2.0) +
                                           std::pow(trajectory_point_y - initial_pose_y, 2.0));
    if (diff_distance > d) {
      return i;
    }
  }
  return 0;
}

void RunNode(const std::shared_ptr<RobotLocalPlannerNodeBase>& node) {
  auto run_func = std::bind(static_cast<void (RobotLocalPlannerNodeBase::*)()>(&RobotLocalPlannerNodeBase::Run),
                            node);
  auto thread = std::thread(run_func);
  rclcpp::spin(node);
  rclcpp::shutdown();
  thread.join();
}

RobotLocalPlannerNodeWithAction::RobotLocalPlannerNodeWithAction() : RobotLocalPlannerNodeBase() {}

RobotLocalPlannerNodeWithAction::RobotLocalPlannerNodeWithAction(const rclcpp::NodeOptions& options)
    : RobotLocalPlannerNodeBase(options) {}

void RobotLocalPlannerNodeWithAction::InitializePlanner(const rclcpp::Node::SharedPtr& node) {
  trajectory_merger_ = std::make_shared<
      tmc_robot_local_planner::TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>>();

  // TODO(Takeshita) パラメータにする
  // Explicitly specify because remapping was not possible
  tmc_robot_local_planner::RobotLocalPlanner::ActionNames action_names;
  action_names.generate_action = "generator/generate";
  action_names.evaluate_action = "evaluator/evaluate";
  action_names.validate_action = "validator/validate";
  action_names.optimize_action = "optimizer/optimize";
  robot_local_planner_ = std::make_shared<tmc_robot_local_planner::RobotLocalPlanner>(
      node, trajectory_merger_, action_names);
}

bool RobotLocalPlannerNodeWithAction::IsReady() {
  if (!RobotLocalPlannerNodeBase::IsReady()) {
    return false;
  }
  if (!robot_local_planner_->AreActionServersReady()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Action servers are not ready");
    return false;
  }
  return true;
}

RobotLocalPlannerNodeBase::PlanResult RobotLocalPlannerNodeWithAction::PlanImpl(
    const std::optional<tmc_manipulation_types::TimedRobotTrajectory>& previous_trajectory,
    const tmc_manipulation_types::RobotState& initial_state,
    const RobotLocalGoal& constraints) {
  // Set the previous trajectory
  if (previous_trajectory) {
    moveit_msgs::msg::RobotTrajectory previous_trajectory_msg;
    tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(
        previous_trajectory.value(), previous_trajectory_msg);
    trajectory_merger_->SetTrajectory(previous_trajectory_msg);
  } else {
    trajectory_merger_->ClearTrajectory();
  }

  moveit_msgs::msg::RobotState initial_state_msg;
  tmc_manipulation_types_bridge::RobotStateToRobotStateMsg(initial_state, initial_state_msg);

  // Trajectory generation
  auto trajectory_future = robot_local_planner_->PlanPath(constraints.constraints_msg, initial_state_msg,
                                                          constraints.normalized_velocity, constraints.enable_base);
  while (trajectory_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {}
  const auto trajectory = trajectory_future.get();

  tmc_manipulation_types::TimedRobotTrajectory converted_solution;
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(trajectory, converted_solution);
  return {converted_solution, robot_local_planner_->last_error_code()};
}


RobotLocalPlannerNodeWithPlugin::RobotLocalPlannerNodeWithPlugin() : RobotLocalPlannerNodeBase() {}

RobotLocalPlannerNodeWithPlugin::RobotLocalPlannerNodeWithPlugin(const rclcpp::NodeOptions& options)
    : RobotLocalPlannerNodeBase(options) {}

void RobotLocalPlannerNodeWithPlugin::InitializePlanner(const rclcpp::Node::SharedPtr& node) {
  trajectory_merger_ = std::make_shared<
      tmc_robot_local_planner::TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>>();

  tmc_robot_local_planner::RobotLocalPlannerPlugins::Loader::PluginNames names;
  names.generate_action = tmc_utils::GetParameter(
      node, "generate_action", std::string("tmc_simple_path_generator/SimplePathGeneratorPlugin"));
  names.evaluate_action = tmc_utils::GetParameter(
      node, "evaluate_action", std::string("tmc_local_path_evaluator/LocalPathEvaluatorPlugin"));
  names.validate_action = tmc_utils::GetParameter(
      node, "validate_action", std::string("tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin"));
  names.optimize_action = tmc_utils::GetParameter(
      node, "optimize_action", std::string("hsrb_quick_path_optimizer/OptimizerPlugin"));

  auto plugins = plugin_loader_.CreatePluginInstances(names);
  plugins.Initialize(node);

  robot_local_planner_ = std::make_shared<tmc_robot_local_planner::RobotLocalPlannerComposition>(
      node, trajectory_merger_, plugins);
}

RobotLocalPlannerNodeBase::PlanResult RobotLocalPlannerNodeWithPlugin::PlanImpl(
    const std::optional<tmc_manipulation_types::TimedRobotTrajectory>& previous_trajectory,
    const tmc_manipulation_types::RobotState& initial_state,
    const RobotLocalGoal& constraints) {
  // Set the previous trajectory
  if (previous_trajectory) {
    trajectory_merger_->SetTrajectory(previous_trajectory.value());
  } else {
    trajectory_merger_->ClearTrajectory();
  }

  // Trajectory generation
  auto trajectory_future = robot_local_planner_->PlanPath(constraints.constraints, initial_state,
                                                          constraints.normalized_velocity, constraints.enable_base);
  while (trajectory_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {}
  const auto trajectory = trajectory_future.get();
  return {trajectory, robot_local_planner_->last_error_code()};
}


}  // namespace hsrb_robot_local_planner_node





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

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_manipulation_types/utils.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_local_planner_utils/common.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

namespace {
const double kConnectPoint = 0.2;           // 200msec/loop
const double kAcceralationLimit = 0.5;      // m/s^2
const double kTFTimeout = 0.05;
const double kPublishPeriodScale = 5;       // 200msec * 5 = 1.0s
const char* const kOriginFrame = "odom";
const char* const kBaseJointName = "world_joint";

}  // namespace

namespace hsrb_robot_local_planner_node {

std::vector<std::string> GetIgnoreJoints(const HsrbJointNames& joint_names, const RobotLocalGoal& constraints) {
  std::vector<std::string> joints;
  if (!constraints.enable_head) {
    joints.insert(joints.begin(), joint_names.head_joints.begin(), joint_names.head_joints.end());
  }
  if (!constraints.enable_arm) {
    joints.insert(joints.begin(), joint_names.arm_joints.begin(), joint_names.arm_joints.end());
  }
  if (!constraints.enable_gripper) {
    joints.insert(joints.begin(), joint_names.hand_joints.begin(), joint_names.hand_joints.end());
  }
  if (!constraints.enable_base) {
    joints.push_back(kBaseJointName);
  }
  return joints;
}


RobotLocalPlannerNodeBase::RobotLocalPlannerNodeBase() : RobotLocalPlannerNodeBase(rclcpp::NodeOptions()) {}

RobotLocalPlannerNodeBase::RobotLocalPlannerNodeBase(const rclcpp::NodeOptions& options)
    : Node("hsrb_robot_local_planner", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      constraints_are_changed_(false) {}

void RobotLocalPlannerNodeBase::Run() {
  Run([]() { return false; });
}

void RobotLocalPlannerNodeBase::Run(std::function<bool()> interrupt) {
  Initialize();

  rclcpp::WallRate rate(1.0 / kConnectPoint);
  while (rclcpp::ok() && !interrupt() && !IsReady()) {
    rate.sleep();
  }

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

  constraint_sub_ = node->create_subscription<tmc_planning_msgs::msg::RobotLocalGoal>(
      "~/constraints", tmc_utils::ReliableVolatileQoS(),
      std::bind(&RobotLocalPlannerNodeBase::ConstraintCallback, this, std::placeholders::_1));

  joint_trajectories_pub_ = std::make_shared<HsrbJointTrajectoriesPublisher>(node);
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

  // If constraint is empty, stop
  if (goal_constraints.IsEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "constraints are empty");
    if (constraints_are_changed) {
      joint_trajectories_pub_->PublishStopTrajectory();
      current_trajectory_.setData(tmc_manipulation_types::TimedRobotTrajectory());
      status_pub_->UpdateConstraintsStatus(goal_constraints.id, tmc_planning_msgs::msg::ConstraintsStatus::EMPTY);
    }
    PublishIsEmpty(true);
    status_pub_->Publish(tmc_robot_local_planner::RobotLocalPlannerErrorCode::kConstraintsEmpty);
    return;
  }
  status_pub_->UpdateConstraintsStatus(goal_constraints.id, tmc_planning_msgs::msg::ConstraintsStatus::RUNNING);

  // Initial is the initial value of the operation plan, and the REF is a robot state for the termination judgment
  const auto target_time = this->now() + rclcpp::Duration::from_seconds(kConnectPoint);
  tmc_manipulation_types::RobotState initial_state;
  tmc_manipulation_types::RobotState ref_state;
  rclcpp::Time connectable_time;
  UpdateRobotstate(target_time, initial_state, ref_state, connectable_time);

  if (use_current_state_for_displacement_) {
    ref_state = GenerateInitialState();
  }
  if (displacement_checker_->ShouldComplete(goal_constraints.id, ref_state)) {
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
  }
  PublishIsEmpty(false);

  // Play the last orbit selected last time when the constraint switch is switched.
  std::optional<tmc_manipulation_types::TimedRobotTrajectory> previous_trajectory;
  if (!constraints_are_changed) {
    previous_trajectory = GetPreviousTrajectory(current_trajectory_, initial_state, target_time);
  }

  // Orbital generation
  const auto ignore_joints = GetIgnoreJoints(joint_trajectories_pub_->GetJointNames(), goal_constraints);
  const auto [trajectory, error_code] = PlanImpl(previous_trajectory, initial_state, ignore_joints, goal_constraints);
  status_pub_->Publish(error_code);

  if (error_code != tmc_robot_local_planner::RobotLocalPlannerErrorCode::kSuccess) {
    RCLCPP_ERROR(this->get_logger(), "Fail to plan path.");
    if (constraints_are_changed) {
      std::lock_guard<std::mutex> lock(constraints_mutex_);
      constraints_are_changed_ = true;
    }
    return;
  }

  // Extract only the trajectory of the last 1 second so that you do not move too much
  auto publish_solution = trajectory;
  tmc_robot_local_planner_utils::DeleteTrajectoryPointsAtTime(kConnectPoint * kPublishPeriodScale, publish_solution);

  // Convert to HSR joint orbital
  trajectory_msgs::msg::JointTrajectory head_trajectory_msg;
  trajectory_msgs::msg::JointTrajectory arm_trajectory_msg;
  trajectory_msgs::msg::JointTrajectory hand_trajectory_msg;
  trajectory_msgs::msg::JointTrajectory base_trajectory_msg;
  RobotTrajectoryToHsrbTrajectoryMsg(publish_solution, connectable_time, joint_trajectories_pub_->GetJointNames(),
                                     head_trajectory_msg, arm_trajectory_msg,
                                     hand_trajectory_msg, base_trajectory_msg);
  joint_trajectories_pub_->SetHeadTrajectory(head_trajectory_msg);
  joint_trajectories_pub_->SetArmTrajectory(arm_trajectory_msg);
  joint_trajectories_pub_->SetHandTrajectory(hand_trajectory_msg);
  // Orbit points to be erased in consideration of speed change
  uint32_t delete_point_num = DeleteTrajectoryFewPoints(initial_state, publish_solution);
  tmc_robot_local_planner_utils::DeleteTrajectoryPointsAtPoint(delete_point_num, base_trajectory_msg);
  joint_trajectories_pub_->SetBaseTrajectory(base_trajectory_msg);

  // Issuance to each joint
  joint_trajectories_pub_->PublishTrajectory(goal_constraints.enable_head,
                                             goal_constraints.enable_arm,
                                             goal_constraints.enable_gripper,
                                             goal_constraints.enable_base);

  current_trajectory_.setData(trajectory);
  current_trajectory_.stamp_ = ConvertToTf2(connectable_time);
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

  tmc_planning_msgs::msg::RobotLocalGoal goal_msg = *msg;
  tmc_robot_local_planner_utils::TransformConstraints(kOriginFrame, kTFTimeout, tf_buffer_, goal_msg.constraints);

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

  const auto joint_names = joint_trajectories_pub_->GetJointNames();

  tmc_manipulation_types::NameSeq use_name;
  use_name.insert(use_name.end(), joint_names.arm_joints.begin(), joint_names.arm_joints.end());
  use_name.insert(use_name.end(), joint_names.hand_joints.begin(), joint_names.hand_joints.end());
  use_name.insert(use_name.end(), joint_names.head_joints.begin(), joint_names.head_joints.end());

  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state = tmc_manipulation_types::ExtractPartialJointState(joint_state, use_name);
  robot_state.joint_state.velocity = Eigen::VectorXd::Zero(use_name.size());
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

void RobotLocalPlannerNodeBase::UpdateRobotstate(
    const rclcpp::Time& target_time,
    tmc_manipulation_types::RobotState& initial_state,
    tmc_manipulation_types::RobotState& ref_state,
    rclcpp::Time& connectable_time) {
  tf2::Stamped<tmc_manipulation_types::RobotState> initial_state_stamped;
  if (tmc_robot_local_planner_utils::SearchConnectablePoint(
          current_trajectory_, target_time, connectable_time, initial_state_stamped)) {
    // If the robot state of Target_time can be obtained from the last orbit, return it with INITIAL/REF.
    initial_state = ConvertRobotStateStampedToRobotState(initial_state_stamped);
    ref_state = initial_state;
  } else {
    // If you cannot get it, the operation plan will be made from the current state
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "non connectable point");
    initial_state = GenerateInitialState();
    if (current_trajectory_.joint_trajectory.points.empty()) {
      ref_state = initial_state;
    } else {
      tmc_robot_local_planner_utils::GetLastPoint(current_trajectory_, ref_state);
    }
    connectable_time = target_time;
  }
}

uint32_t RobotLocalPlannerNodeBase::DeleteTrajectoryFewPoints(
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const {
  // The distance between KconnectPoint, depending on the speed and deceleration
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

  // Since it was not possible to remap, specify explicitly
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
    const std::vector<std::string>& ignore_joints,
    const RobotLocalGoal& constraints) {
  // Set the previous orbit
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

  // Orbital generation
  auto trajectory_future = robot_local_planner_->PlanPath(constraints.constraints_msg, initial_state_msg,
                                                          constraints.normalized_velocity, ignore_joints);
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
    const std::vector<std::string>& ignore_joints,
    const RobotLocalGoal& constraints) {
  // Set the previous orbit
  if (previous_trajectory) {
    trajectory_merger_->SetTrajectory(previous_trajectory.value());
  } else {
    trajectory_merger_->ClearTrajectory();
  }

  // Orbital generation
  auto trajectory_future = robot_local_planner_->PlanPath(constraints.constraints, initial_state,
                                                          constraints.normalized_velocity, ignore_joints);
  while (trajectory_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {}
  const auto trajectory = trajectory_future.get();
  return {trajectory, robot_local_planner_->last_error_code()};
}


}  // namespace hsrb_robot_local_planner_node

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

#include "optimizer_node.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp_action/create_server.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tmc_manipulation_types/utils.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_timeopt/quick_trajectory_filter.hpp>
#include <tmc_timeopt/trajectory_filter_via_stop_state.hpp>
#include <tmc_utils/qos.hpp>

#include "base_kinematics_ros.hpp"
#include "conversions.hpp"
#include "joint_limit.hpp"

#include "ThreadPool.h"

namespace {
// HSR's cart turning axis name
const char* const kBaseRollJointName = "base_roll_joint";
// HSR's right wheel axis name
const char* const kBaseRightWheelJointName = "base_r_drive_wheel_joint";
// HSR's left wheel axis name
const char* const kBaseLeftWheelJointName = "base_l_drive_wheel_joint";
// Number of cart joints
constexpr uint32_t kBaseJointNum = 3;
// Default value for the number of threads in multi-thread processing
constexpr int32_t kDefaultThreadPoolSize = 8;

// Default value for the optimized trajectory sampling interval [sec]
constexpr double kDefaultSamplingIntervalSec = 0.1;

template<typename Type>
bool ValidateSize(const std::vector<Type>& input_vector,
                  uint32_t expected_size) {
  return input_vector.size() == expected_size;
}

template<typename TypeA, typename TypeB>
bool ValidateSize(const std::vector<TypeA>& input_a,
                  const std::vector<TypeB>& input_b) {
  return input_a.size() == input_b.size();
}

bool ValidatePositionsSize(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points,
    uint32_t expected_size) {
  for (const auto point : points) {
    if (!ValidateSize(point.positions, expected_size)) {
      return false;
    }
  }
  return true;
}

bool ValidateTransformsSize(
    const std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>& points,
    uint32_t expected_size) {
  for (const auto point : points) {
    if (!ValidateSize(point.transforms, expected_size)) {
      return false;
    }
  }
  return true;
}

bool ValidateRobotTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) {
  if (!trajectory.joint_trajectory.joint_names.empty()) {
    if (!ValidatePositionsSize(trajectory.joint_trajectory.points,
                               trajectory.joint_trajectory.joint_names.size()) ||
        !ValidateSize(trajectory.joint_trajectory.points.front().velocities,
                      trajectory.joint_trajectory.joint_names.size())) {
      return false;
    }
  }
  if (!ValidateSize(trajectory.multi_dof_joint_trajectory.joint_names, 1) ||
      !ValidateTransformsSize(trajectory.multi_dof_joint_trajectory.points,
                              trajectory.multi_dof_joint_trajectory.joint_names.size()) ||
      !ValidateSize(trajectory.multi_dof_joint_trajectory.points.front().velocities,
                    trajectory.multi_dof_joint_trajectory.joint_names.size())) {
    return false;
  }
  if (trajectory.joint_trajectory.joint_names.empty()) {
    if (trajectory.multi_dof_joint_trajectory.points.size() < 2) {
      return false;
    }
  } else {
    if (trajectory.joint_trajectory.points.size() < 2 ||
        !ValidateSize(trajectory.joint_trajectory.points,
                      trajectory.multi_dof_joint_trajectory.points)) {
      return false;
    }
  }
  return true;
}

double ExtractBaseYaw(const nav_msgs::msg::Odometry& odom) {
  tf2::Quaternion quaternion;
  tf2::fromMsg(odom.pose.pose.orientation, quaternion);
  return tf2::getYaw(quaternion);
}

double ExtractBaseRollJointPosition(const sensor_msgs::msg::JointState& joint_state) {
  auto index = tmc_manipulation_types::GetJointIndex(
      joint_state.name, kBaseRollJointName);
  return joint_state.position[index];
}

double ExtractJointVelocity(const sensor_msgs::msg::JointState& joint_state,
                            const std::string& joint_name) {
  auto index = tmc_manipulation_types::GetJointIndex(
      joint_state.name, kBaseRollJointName);
  return joint_state.velocity[index];
}

Eigen::Vector3d ExtractBaseJointVelocities(const sensor_msgs::msg::JointState& joint_state) {
  // According to hsrb_base_controllers::OmniBaseJointID, in the order of right wheel, left wheel, turning
  return Eigen::Vector3d(
      ExtractJointVelocity(joint_state, kBaseRightWheelJointName),
      ExtractJointVelocity(joint_state, kBaseLeftWheelJointName),
      ExtractJointVelocity(joint_state, kBaseRollJointName));
}

double ComputeBaseDirection(const tmc_manipulation_types::TimedMultiDOFJointTrajectory& base_trajectory) {
  // When there is an initial velocity, it is correct to calculate the direction of velocity and acceleration considering it
  // However, since it worked with HSR even without considering it, prioritize calculation speed and proceed with this implementation
  auto diff_x = base_trajectory.points.at(1).transforms.front().translation().x()
              - base_trajectory.points.at(0).transforms.front().translation().x();
  auto diff_y = base_trajectory.points.at(1).transforms.front().translation().y()
              - base_trajectory.points.at(0).transforms.front().translation().y();
  return std::atan2(diff_y, diff_x);
}

void UpdateLimits(Eigen::VectorXd& current, const Eigen::Vector3d& calculated) {
  // Only relax the speed and acceleration limits when permissible
  // Assuming the target limit is written at the end
  for (uint32_t i = 0; i < kBaseJointNum; ++i) {
    current.tail(kBaseJointNum)[i] = std::max(current.tail(kBaseJointNum)[i], calculated[i]);
  }
}
}  // namespace

namespace hsrb_quick_path_optimizer {

// Speed and acceleration limits
struct JointLimit {
  Eigen::VectorXd max_velocities;
  Eigen::VectorXd max_accelerations;

  JointLimit(const rclcpp::Node::SharedPtr& node,
             const std::vector<std::string>& joint_names) {
    if (!GetVelocityLimit(node, joint_names, this->max_velocities)) {
      max_velocities.resize(0);
      return;
    }
    if (!GetAccelerationLimit(node, joint_names, this->max_accelerations)) {
      max_accelerations.resize(0);
      return;
    }
  }

  bool IsValid() const {
    return (this->max_velocities.size() == this->max_accelerations.size() &&
            this->max_velocities.size() != 0);
  }
};

// Input trajectory
struct InputTrajectory {
  Eigen::VectorXd initial_positions;
  Eigen::VectorXd initial_velocities;
  std::vector<Eigen::VectorXd> way_points;

  explicit InputTrajectory(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) {
    ExtractInitialPositions(trajectory, initial_positions);
    ExtractInitialVelocities(trajectory, initial_velocities);
    // To extract the first point as initial_*, processing to omit the first point is necessary
    // To make the input const, copying and deleting is necessary,
    // In that case, converting to a vector and then removing the first point should be lighter processing
    ConvertToWayPoints(trajectory, way_points);
    way_points.erase(way_points.begin());
  }
};

void OptimizerPluginCommon::Initialize(const rclcpp::Node::SharedPtr& node) {
  node_ = node;

  sampling_interval_sec_ = tmc_utils::GetParameter(node, "sampling_interval_sec", kDefaultSamplingIntervalSec);

  base_kinematics_ = std::make_shared<BaseKinematics>(BaseJointLimitsRos(node), OmniBaseSizeRos(node));

  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", tmc_utils::BestEffortQoS(),
      std::bind(&OptimizerPluginCommon::JointStateCallback, this, std::placeholders::_1));
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", tmc_utils::BestEffortQoS(),
      std::bind(&OptimizerPluginCommon::OdometryCallback, this, std::placeholders::_1));

  optimize_timeout_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "optimize_timeout", 0.1);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback, optimize_timeout_, std::placeholders::_1)));

  velocity_ratio_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "velocity_ratio", 1.0);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback, velocity_ratio_, std::placeholders::_1)));

  acceleration_ratio_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "acceleration_ratio", 1.0);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback,
                acceleration_ratio_, std::placeholders::_1)));

  // Considering compatibility, make it inactive by default
  acceleration_rate_for_stop_ = std::make_shared<tmc_utils::DynamicParameter<double>>(
      node, "acceleration_rate_for_stop", 0.0);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback,
                acceleration_rate_for_stop_, std::placeholders::_1)));
}

bool OptimizerPluginCommon::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (joint_state_ && odom_) {
    timeout_point_ = std::chrono::system_clock::now()
                   + std::chrono::nanoseconds(static_cast<int64_t>(optimize_timeout_->value() * 1.0e9));
    return true;
  } else {
    if (!joint_state_) {
      RCLCPP_ERROR(node_->get_logger(), "Joint state is not available.");
    }
    if (!odom_) {
      RCLCPP_ERROR(node_->get_logger(), "Odometry is not available.");
    }
    return false;
  }
}

void OptimizerPluginCommon::UpdateBaseVelocities(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& base_trajectory,
    JointLimit& dst_joint_limit) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto velocity = base_kinematics_->CalculateBaseMaxVelocity(
      ExtractBaseYaw(*odom_),
      ComputeBaseDirection(base_trajectory),
      ExtractBaseRollJointPosition(*joint_state_));
  UpdateLimits(dst_joint_limit.max_velocities, velocity);
}

void OptimizerPluginCommon::UpdateBaseAccelarations(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& base_trajectory,
    JointLimit& dst_joint_limit) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto acceleration = base_kinematics_->CalculateBaseMaxAcceleration(
      ExtractBaseYaw(*odom_),
      ComputeBaseDirection(base_trajectory),
      ExtractBaseRollJointPosition(*joint_state_),
      ExtractBaseJointVelocities(*joint_state_));
  UpdateLimits(dst_joint_limit.max_accelerations, acceleration);
}

ITrajectoryFilterAdapter::Ptr OptimizerPluginCommon::Optimize(
    std::function<bool()> interrupt, const InputTrajectory& trajectory, const JointLimit& joint_limit) const {
  std::function<bool()> interrupt_func = [this, interrupt]() {
      return interrupt() || std::chrono::system_clock::now() > this->timeout_point_;
  };
  auto trajectory_impl = std::make_shared<tmc_timeopt::QuickTrajectoryFilter>(
      trajectory.initial_positions,
      trajectory.initial_velocities,
      trajectory.way_points,
      velocity_ratio_->value() * joint_limit.max_velocities,
      acceleration_ratio_->value() * joint_limit.max_accelerations,
      interrupt_func);
  if (trajectory_impl->IsValid()) {
    return std::make_shared<TrajectoryFilterAdapter>(trajectory_impl);
  } else {
    return nullptr;
  }
}

ITrajectoryFilterAdapter::Ptr OptimizerPluginCommon::OptimizeViaStop(
    std::function<bool()> interrupt, const InputTrajectory& trajectory, const JointLimit& joint_limit) const {
  if (acceleration_rate_for_stop_->value() < std::numeric_limits<double>::min()) {
    return nullptr;
  }
  std::function<bool()> interrupt_func = [this, interrupt]() {
      return interrupt() || std::chrono::system_clock::now() > this->timeout_point_;
  };
  auto trajectory_impl = std::make_shared<tmc_timeopt::TrajectoryFilterViaStopState>(
      trajectory.initial_positions,
      trajectory.initial_velocities,
      trajectory.way_points,
      velocity_ratio_->value() * joint_limit.max_velocities,
      acceleration_ratio_->value() * joint_limit.max_accelerations,
      acceleration_rate_for_stop_->value(),
      interrupt_func);
  if (trajectory_impl->IsValid()) {
    return std::make_shared<TrajectoryFilterAdapter>(trajectory_impl);
  } else {
    return nullptr;
  }
}

void OptimizerPluginCommon::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (lock.try_lock()) {
    joint_state_ = msg;
  }
}
void OptimizerPluginCommon::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (lock.try_lock()) {
    odom_ = msg;
  }
}


void OptimizerPlugin::Initialize(const rclcpp::Node::SharedPtr& node) {
  common_.Initialize(node);

  node_ = node;
  is_force_succeeded_ = tmc_utils::GetParameter(node, "is_force_succeeded", true);
}

bool OptimizerPlugin::Optimize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in,
                               std::function<bool()> interrupt,
                               tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  const auto result = Optimize({trajectory_in}, interrupt, trajectories_out);
  if (result) {
    // If result is true, there is always the first trajectory
    auto min_index = 0;
    for (auto i = 1; i < trajectories_out.size(); ++i) {
      if (trajectories_out[i].joint_trajectory.points.back().time_from_start <
              trajectories_out[min_index].joint_trajectory.points.back().time_from_start) {
        min_index = i;
      }
    }
    trajectory_out = std::move(trajectories_out[min_index]);
  } else {
    if (is_force_succeeded_) {
      RCLCPP_INFO(node_->get_logger(), "Force success.");
      trajectory_out = trajectory_in;
      return true;
    }
  }
  return result;
}

bool OptimizerPlugin::Optimize(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                               std::function<bool()> interrupt,
                               std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) {
  if (trajectories_in.empty()) {
    return false;
  }
  if (!common_.Reset()) {
    return false;
  }

  // Since an empty check is performed first, [0] is always present
  auto joint_limit = JointLimit(node_, trajectories_in[0].joint_trajectory.joint_names);
  if (!joint_limit.IsValid()) {
    return false;
  }
  common_.UpdateBaseVelocities(trajectories_in[0].multi_dof_joint_trajectory, joint_limit);
  common_.UpdateBaseAccelarations(trajectories_in[0].multi_dof_joint_trajectory, joint_limit);

  for (const auto& trajectory_in : trajectories_in) {
    const auto input_trajectory = InputTrajectory(trajectory_in);
    std::vector<ITrajectoryFilterAdapter::Ptr> trajectory_candidates = {
      common_.Optimize(interrupt, input_trajectory, joint_limit),
      common_.OptimizeViaStop(interrupt, input_trajectory, joint_limit)};

    for (const auto& trajectory_impl : trajectory_candidates) {
      if (!trajectory_impl) {
        continue;
      }
      tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
      ConvertToRobotTrajectory(
          trajectory_impl,
          trajectories_in[0].joint_trajectory.joint_names,
          trajectories_in[0].multi_dof_joint_trajectory.joint_names.front(),
          common_.sampling_interval_sec(),
          trajectory_out);
      trajectories_out.emplace_back(trajectory_out);
    }
  }
  return !trajectories_out.empty();
}


void OptimizerPluginMultiThread::Initialize(const rclcpp::Node::SharedPtr& node) {
  common_.Initialize(node);

  node_ = node;
  int32_t thread_pool_size = tmc_utils::GetParameter(node, "thread_pool_size", kDefaultThreadPoolSize);
  if (thread_pool_size <= 0) {
    thread_pool_size = kDefaultThreadPoolSize;
  }
  pool_.reset(new ThreadPool(thread_pool_size));
}

bool OptimizerPluginMultiThread::Optimize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in,
                                          std::function<bool()> interrupt,
                                          tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  const auto result = Optimize({trajectory_in}, interrupt, trajectories_out);
  if (result) {
    trajectory_out = trajectories_out[0];
  }
  return result;
}

bool OptimizerPluginMultiThread::Optimize(
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
    std::function<bool()> interrupt,
    std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) {
  if (trajectories_in.empty()) {
    return false;
  }
  if (!common_.Reset()) {
    return false;
  }

  // Since an empty check is performed first, [0] is always present
  auto joint_limit = JointLimit(node_, trajectories_in[0].joint_trajectory.joint_names);
  if (!joint_limit.IsValid()) {
    return false;
  }
  common_.UpdateBaseVelocities(trajectories_in[0].multi_dof_joint_trajectory, joint_limit);
  common_.UpdateBaseAccelarations(trajectories_in[0].multi_dof_joint_trajectory, joint_limit);

  std::function<ITrajectoryFilterAdapter::Ptr(const InputTrajectory&)> func_normal =
      std::bind(&OptimizerPluginCommon::Optimize, std::cref(common_),
                interrupt,  std::placeholders::_1, std::cref(joint_limit));
  std::function<ITrajectoryFilterAdapter::Ptr(const InputTrajectory&)> func_via_stop =
      std::bind(&OptimizerPluginCommon::OptimizeViaStop, std::cref(common_),
                interrupt,  std::placeholders::_1, std::cref(joint_limit));

  std::vector<std::future<tmc_manipulation_types::TimedRobotTrajectory>> trajectory_futures;
  for (const auto& trajectory_in : trajectories_in) {
    trajectory_futures.emplace_back(pool_->enqueue(&OptimizerPluginMultiThread::OptimizeImpl, this,
                                                   func_normal, std::ref(trajectory_in)));
    trajectory_futures.emplace_back(pool_->enqueue(&OptimizerPluginMultiThread::OptimizeImpl, this,
                                                   func_via_stop, std::ref(trajectory_in)));
  }

  for (auto& future : trajectory_futures) {
    future.wait();
    const auto trajectory = future.get();
    if (!trajectory.joint_trajectory.joint_names.empty() ||
        !trajectory.multi_dof_joint_trajectory.joint_names.empty()) {
      trajectories_out.emplace_back(trajectory);
    }
  }
  return !trajectories_out.empty();
}

tmc_manipulation_types::TimedRobotTrajectory OptimizerPluginMultiThread::OptimizeImpl(
    std::function<ITrajectoryFilterAdapter::Ptr(const InputTrajectory&)> optimize_func,
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in) {
  // It's a waste to convert to InputTrajectory, but joint names are needed for conversion, so it can't be helped
  auto trajectory_impl = optimize_func(InputTrajectory(trajectory_in));
  if (!trajectory_impl) {
    return tmc_manipulation_types::TimedRobotTrajectory();
  }

  tmc_manipulation_types::TimedRobotTrajectory optimized_trajectory;
  ConvertToRobotTrajectory(
      trajectory_impl,
      trajectory_in.joint_trajectory.joint_names,
      trajectory_in.multi_dof_joint_trajectory.joint_names.front(),
      common_.sampling_interval_sec(),
      optimized_trajectory);
  return optimized_trajectory;
}


OptimizerNode::OptimizerNode() : OptimizerNode(rclcpp::NodeOptions()) {}

OptimizerNode::OptimizerNode(const rclcpp::NodeOptions& options)
    : Node("optimizer", options) {}

void OptimizerNode::Initialize() {
  const auto node = shared_from_this();
  impl_.Initialize(node);

  server_ = rclcpp_action::create_server<tmc_planning_msgs::action::OptimizeRobotTrajectory>(
      this, "~/optimize",
      std::bind(&OptimizerNode::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OptimizerNode::CancelCallback, this, std::placeholders::_1),
      std::bind(&OptimizerNode::FeedbackSetupCallback, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse OptimizerNode::GoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tmc_planning_msgs::action::OptimizeRobotTrajectory::Goal> goal) {
  if (!ValidateRobotTrajectory(goal->robot_trajectory)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OptimizerNode::CancelCallback(const ServerGoalHandlePtr goal_handle) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OptimizerNode::FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
  std::thread{std::bind(&OptimizerNode::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void OptimizerNode::Execute(const ServerGoalHandlePtr goal_handle) {
  const auto goal = goal_handle->get_goal();

  tmc_manipulation_types::TimedRobotTrajectory trajectory_in;
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(goal->robot_trajectory, trajectory_in);

  auto interrupt_func = [goal_handle]() { return goal_handle->is_canceling(); };

  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  const auto opt_result = impl_.Optimize(trajectory_in, interrupt_func, trajectory_out);
  if (opt_result) {
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(trajectory_out, trajectory_msg);
    PublishSucceeded(goal_handle, trajectory_msg);
  } else {
    PublishAborted(goal_handle, goal->robot_trajectory);
  }
}

void OptimizerNode::PublishSucceeded(const ServerGoalHandlePtr goal_handle,
                                     const moveit_msgs::msg::RobotTrajectory& trajectory) {
  auto result = std::make_shared<tmc_planning_msgs::action::OptimizeRobotTrajectory::Result>();
  result->robot_trajectory = trajectory;
  goal_handle->succeed(result);
}

void OptimizerNode::PublishAborted(const ServerGoalHandlePtr goal_handle,
                                   const moveit_msgs::msg::RobotTrajectory& trajectory) {
  auto result = std::make_shared<tmc_planning_msgs::action::OptimizeRobotTrajectory::Result>();
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    return;
  } else {
    result->robot_trajectory = trajectory;
    goal_handle->abort(result);
  }
}

}  // namespace hsrb_quick_path_optimizer

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(hsrb_quick_path_optimizer::OptimizerPlugin, tmc_robot_local_planner::IOptimizer)
PLUGINLIB_EXPORT_CLASS(hsrb_quick_path_optimizer::OptimizerPluginMultiThread, tmc_robot_local_planner::IOptimizer)

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

#include <builtin_interfaces/msg/time.hpp>
#include <tmc_robot_local_planner/component_interfaces.hpp>

namespace hsrb_robot_local_planner_node {

class TimeStampPublisher {
 public:
  using Ptr = std::shared_ptr<TimeStampPublisher>;

  TimeStampPublisher(const rclcpp::Node::SharedPtr& node, const std::string& topic_name) {
    pub_ = node->create_publisher<builtin_interfaces::msg::Time>(topic_name, rclcpp::SystemDefaultsQoS());
  }

  void Publish(const rclcpp::Time& stamp) {
    builtin_interfaces::msg::Time msg = stamp;
    pub_->publish(msg);
  }

 private:
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_;
};

class PluginBase {
 public:
  explicit PluginBase(const std::string& plugin_name) : plugin_name_(plugin_name) {}
  virtual ~PluginBase() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) {
    initilized_stamp_pub_ = std::make_shared<TimeStampPublisher>(node, plugin_name_ + "/initialized_stamp");
    executed_stamp_pub_ = std::make_shared<TimeStampPublisher>(node, plugin_name_ + "/executed_stamp");
    clock_ = node->get_clock();

    initilized_stamp_pub_->Publish(clock_->now());
  }

  void Execute() {
    executed_stamp_pub_->Publish(clock_->now());
  }

 private:
  std::string plugin_name_;

  TimeStampPublisher::Ptr initilized_stamp_pub_;
  TimeStampPublisher::Ptr executed_stamp_pub_;
  rclcpp::Clock::SharedPtr clock_;
};


class GeneratorPlugin : public tmc_robot_local_planner::IGenerator {
 public:
  GeneratorPlugin() : base_("generator") {}
  virtual ~GeneratorPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override {
    base_.Initialize(node);
  }

  bool Generate(const tmc_robot_local_planner::Constraints& constraints,
                const tmc_manipulation_types::RobotState& initial_state,
                double normalized_velocity,
                const std::vector<std::string>& ignore_joints,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override {
    if (constraints.hard_joint_constraints.empty()) {
      return false;
    }
    if (initial_state.joint_state.name.empty()) {
      return false;
    }

    const auto goal_state = constraints.hard_joint_constraints[0]->Sample();

    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = goal_state.joint_state.name;
    trajectory.joint_trajectory.points.resize(1);
    trajectory.joint_trajectory.points[0].positions = goal_state.joint_state.position;
    trajectory.multi_dof_joint_trajectory.joint_names = goal_state.multi_dof_joint_state.names;
    trajectory.multi_dof_joint_trajectory.points.resize(1);
    trajectory.multi_dof_joint_trajectory.points[0].transforms = goal_state.multi_dof_joint_state.poses;
    trajectories_out.emplace_back(trajectory);

    base_.Execute();
    return true;
  };

 private:
  PluginBase base_;
};

class EvaluatorPlugin : public tmc_robot_local_planner::IEvaluator {
 public:
  EvaluatorPlugin() : base_("evaluator") {}
  virtual ~EvaluatorPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override {
    base_.Initialize(node);
  }

  bool Evaluate(const tmc_robot_local_planner::Constraints& constraints,
                const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override {
    if (trajectories_in.empty()) {
      return false;
    }

    trajectories_out = trajectories_in;
    base_.Execute();
    return true;
  }

 private:
  PluginBase base_;
};

class ValidatorPlugin : public tmc_robot_local_planner::IValidator {
 public:
  ValidatorPlugin() : base_("validator") {}
  virtual ~ValidatorPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override {
    base_.Initialize(node);
  }

  bool Validate(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override {
    if (trajectories_in.empty()) {
      return false;
    }

    trajectory_out = trajectories_in[0];
    base_.Execute();
    return true;
  }

 private:
  PluginBase base_;
};

class OptimizerPlugin : public tmc_robot_local_planner::IOptimizer {
 public:
  OptimizerPlugin() : base_("optimizer") {}
  virtual ~OptimizerPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override {
    base_.Initialize(node);
  }

  bool Optimize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in,
                std::function<bool()> interrupt,
                tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override {
    trajectory_out = trajectory_in;
    base_.Execute();
    return true;
  }
  bool Optimize(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override {
    trajectories_out = trajectories_in;
    base_.Execute();
    return true;
  }

 private:
  PluginBase base_;
};

}  // namespace hsrb_robot_local_planner_node

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(hsrb_robot_local_planner_node::GeneratorPlugin,
                       tmc_robot_local_planner::IGenerator)

PLUGINLIB_EXPORT_CLASS(hsrb_robot_local_planner_node::EvaluatorPlugin,
                       tmc_robot_local_planner::IEvaluator)

PLUGINLIB_EXPORT_CLASS(hsrb_robot_local_planner_node::ValidatorPlugin,
                       tmc_robot_local_planner::IValidator)

PLUGINLIB_EXPORT_CLASS(hsrb_robot_local_planner_node::OptimizerPlugin,
                       tmc_robot_local_planner::IOptimizer)

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
/// @file     robot_local_planner-test.cpp
/// @brief    HSRB_ROBOT_LOCAL_PLANNER_NODE test
/// @author   Satoru Onoda

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_utils/caching_subscriber.hpp>

#include "../src/hsrb_robot_local_planner_node/node.hpp"
#include "utils.hpp"

namespace {
constexpr double kEpsilon = 0.2;

template <typename ActionType>
class DummyBase {
 public:
  using Ptr = std::shared_ptr<DummyBase>;

  DummyBase(const rclcpp::Node::SharedPtr& node, const std::string& action_name)
      : node_(node), do_abort_(false), is_called_(false) {
    server_ = rclcpp_action::create_server<ActionType>(
        node, action_name,
        std::bind(&DummyBase::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyBase::CancelCallback, this, std::placeholders::_1),
        std::bind(&DummyBase::FeedbackSetupCallback, this, std::placeholders::_1));
  }
  void SetDoAbort(bool value) { do_abort_ = value; }
  bool IsActionCalled() const { return is_called_; }

 protected:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  using GoalType = typename ActionType::Goal;
  using ResultType = typename ActionType::Result;

  typename rclcpp_action::Server<ActionType>::SharedPtr server_;

  rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const GoalType> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
    std::thread{std::bind(&DummyBase::Execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void Execute(const ServerGoalHandlePtr goal_handle) {
    const auto goal = goal_handle->get_goal();
    const auto result = ExecuteImpl(goal);
    if (do_abort_) {
      goal_handle->abort(result);
    } else {
      goal_handle->succeed(result);
    }
    is_called_ = true;
  }

  virtual std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) = 0;

 private:
  rclcpp::Node::SharedPtr node_;
  bool do_abort_;
  bool is_called_;
};

class DummyGenerator : public DummyBase<tmc_planning_msgs::action::GenerateRobotTrajectories> {
 public:
  using Ptr = std::shared_ptr<DummyGenerator>;

  explicit DummyGenerator(const rclcpp::Node::SharedPtr& node) : DummyBase(node, "/generator/generate") {}
  virtual ~DummyGenerator() = default;

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = goal->initial_state.joint_state.name;
    trajectory.multi_dof_joint_trajectory.joint_names = goal->initial_state.multi_dof_joint_state.joint_names;

    std::vector<double> joint_diffs;
    for (auto i = 0; i < goal->initial_state.joint_state.name.size(); ++i) {
      joint_diffs.push_back(goal->constraints.hard_joint_constraints[0].min.joint_state.position[i] -
                            goal->initial_state.joint_state.position[i]);
    }
    // Since only X is tweaked in the test, process only X
    const double odom_x_diff =
        goal->constraints.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0].translation.x -
        goal->initial_state.multi_dof_joint_state.transforms[0].translation.x;

    constexpr double kTrajectoryDuration = 1.5;
    for (double time_from_start = 0.1; time_from_start < kTrajectoryDuration; time_from_start += 0.1) {
      trajectory_msgs::msg::JointTrajectoryPoint joint_point;
      const double progress = time_from_start / kTrajectoryDuration;
      for (auto i = 0; i < goal->initial_state.joint_state.name.size(); ++i) {
        joint_point.positions.push_back(goal->initial_state.joint_state.position[i] + progress * joint_diffs[i]);
      }
      joint_point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
      trajectory.joint_trajectory.points.push_back(joint_point);

      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint base_point;
      base_point.transforms = goal->initial_state.multi_dof_joint_state.transforms;
      base_point.transforms[0].translation.x += odom_x_diff * progress;
      base_point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
      trajectory.multi_dof_joint_trajectory.points.push_back(base_point);
    }

    auto result = std::make_shared<ResultType>();
    result->robot_trajectories.push_back(trajectory);
    return result;
  }
};

class DummyEvaluator : public DummyBase<tmc_planning_msgs::action::EvaluateRobotTrajectories> {
 public:
  using Ptr = std::shared_ptr<DummyEvaluator>;

  explicit DummyEvaluator(const rclcpp::Node::SharedPtr& node) : DummyBase(node, "evaluator/evaluate") {}
  virtual ~DummyEvaluator() = default;

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    auto result = std::make_shared<ResultType>();
    result->robot_trajectories = goal->robot_trajectories;
    return result;
  }
};

class DummyValidator : public DummyBase<tmc_planning_msgs::action::ValidateRobotTrajectories> {
 public:
  using Ptr = std::shared_ptr<DummyValidator>;

  explicit DummyValidator(const rclcpp::Node::SharedPtr& node) : DummyBase(node, "validator/validate") {}
  virtual ~DummyValidator() = default;

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    auto result = std::make_shared<ResultType>();
    result->robot_trajectory = goal->robot_trajectories[0];
    return result;
  }
};

class DummyOptimizer : public DummyBase<tmc_planning_msgs::action::OptimizeRobotTrajectory> {
 public:
  using Ptr = std::shared_ptr<DummyOptimizer>;

  explicit DummyOptimizer(const rclcpp::Node::SharedPtr& node) : DummyBase(node, "optimizer/optimize") {}
  virtual ~DummyOptimizer() = default;

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    auto result = std::make_shared<ResultType>();
    result->robot_trajectory = goal->robot_trajectory;
    return result;
  }
};

bool CheckTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                     const std::string& joint_name,
                     double check_value,
                     double error) {
  auto it = std::find(trajectory.joint_names.begin(),
                      trajectory.joint_names.end(), joint_name);
  if (it == trajectory.joint_names.end()) {
    return false;
  }
  auto index = std::distance(trajectory.joint_names.begin(), it);
  auto last_point = trajectory.points.back();
  EXPECT_NEAR(last_point.positions[index], check_value, error);
  return true;
}
}  // namespace

namespace hsrb_robot_local_planner_node {

class TrajectorySubscriber : public tmc_utils::CachingSubscriber<trajectory_msgs::msg::JointTrajectory> {
 public:
  using Ptr = std::shared_ptr<TrajectorySubscriber>;

  TrajectorySubscriber(const rclcpp::Node::SharedPtr& node, const std::string& topic_name)
      : CachingSubscriber(node, topic_name) {}
  virtual ~TrajectorySubscriber() = default;
};

class RobotLocalPlannerNodeWithActionTest : public ::testing::Test {
 public:
  void SetUp() override;
  void TearDown() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void SpinSomeLoop() {
    is_spin_interrupt_ = false;
    while (!is_spin_interrupt_) {
      SpinSome();
    }
  }

  std::shared_ptr<RobotLocalPlannerNodeWithAction> client_node_;
  std::thread client_thread_;
  bool is_client_interrupt_;
  bool is_spin_interrupt_;

  rclcpp::Node::SharedPtr server_node_;

  DummyGenerator::Ptr generator_;
  DummyEvaluator::Ptr evaluator_;
  DummyValidator::Ptr validator_;
  DummyOptimizer::Ptr optimizer_;

  TrajectorySubscriber::Ptr head_trajectory_;
  TrajectorySubscriber::Ptr arm_trajectory_;
  TrajectorySubscriber::Ptr hand_trajectory_;
  TrajectorySubscriber::Ptr base_trajectory_;
  void WaitFor(TrajectorySubscriber::Ptr trajectory);

  tmc_utils::CachingSubscriber<std_msgs::msg::Bool>::Ptr is_constraints_empty_;
  void WaitFor(bool is_constraints_empty);
  void WaitForTimeout(double timeout);

  tmc_utils::CachingSubscriber<tmc_planning_msgs::msg::RobotDisplacements>::Ptr displacements_cache_;
  RobotLocalPlannerStatusSubscriber::Ptr planner_status_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr base_state_publisher_;
  void PublishRobotState(double arm_flex_position = 0.0);

  rclcpp::Publisher<tmc_planning_msgs::msg::RobotLocalGoal>::SharedPtr constraints_publisher_;
  void PublishRobotLocalGoal(const tmc_planning_msgs::msg::RangeJointConstraint& rjc,
                             bool enable_arm, bool enable_head, bool enable_gripper, bool enable_base,
                             const std::string& id = "");

  tmc_planning_msgs::msg::RangeJointConstraint rjc_;
};

void RobotLocalPlannerNodeWithActionTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("robot_description", tmc_manipulation_tests::hsrb::GetUrdf()),
      rclcpp::Parameter("robot_description_kinematics", tmc_manipulation_tests::hsrb::GetUrdf()),
      rclcpp::Parameter("head_trajectory_controller.joints",
                        std::vector<std::string>({"head_pan_joint", "head_tilt_joint"})),
      rclcpp::Parameter("arm_trajectory_controller.joints",
                        std::vector<std::string>({"arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
                                                  "wrist_flex_joint", "wrist_roll_joint"})),
      rclcpp::Parameter("gripper_controller.joints", std::vector<std::string>({"hand_motor_joint"})),
      rclcpp::Parameter("omni_base_controller.joints", std::vector<std::string>({"odom_x", "odom_y", "odom_t"}))};
      // rclcpp::Parameter("joint_displacement_threshold", 0.0),
      // rclcpp::Parameter("link_displacement_threshold", 0.0),
      // rclcpp::Parameter("joint_stall_threshold", 0.0),
      // rclcpp::Parameter("link_stall_threshold", 0.0)};

  client_node_ = std::make_shared<RobotLocalPlannerNodeWithAction>(options);

  server_node_ = rclcpp::Node::make_shared("server");

  generator_ = std::make_shared<DummyGenerator>(server_node_);
  evaluator_ = std::make_shared<DummyEvaluator>(server_node_);
  validator_ = std::make_shared<DummyValidator>(server_node_);
  optimizer_ = std::make_shared<DummyOptimizer>(server_node_);

  head_trajectory_ = std::make_shared<TrajectorySubscriber>(server_node_,
                                                            "head_trajectory_controller/joint_trajectory");
  arm_trajectory_ = std::make_shared<TrajectorySubscriber>(server_node_, "arm_trajectory_controller/joint_trajectory");
  hand_trajectory_ = std::make_shared<TrajectorySubscriber>(server_node_, "gripper_controller/joint_trajectory");
  base_trajectory_ = std::make_shared<TrajectorySubscriber>(server_node_, "omni_base_controller/joint_trajectory");
  is_constraints_empty_ = std::make_shared<tmc_utils::CachingSubscriber<std_msgs::msg::Bool>>(
      server_node_, "hsrb_robot_local_planner/is_constraints_empty");
  displacements_cache_ = std::make_shared<tmc_utils::CachingSubscriber<tmc_planning_msgs::msg::RobotDisplacements>>(
      server_node_, "hsrb_robot_local_planner/displacements");
  planner_status_sub_ = std::make_shared<RobotLocalPlannerStatusSubscriber>(server_node_);

  joint_state_publisher_ = server_node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SystemDefaultsQoS());
  base_state_publisher_ = server_node_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      "base_trajectory_controller_state", rclcpp::SystemDefaultsQoS());
  constraints_publisher_ = server_node_->create_publisher<tmc_planning_msgs::msg::RobotLocalGoal>(
      "hsrb_robot_local_planner/constraints", rclcpp::SystemDefaultsQoS());

  rjc_ = GenerateRangeJointConstraint();

  is_client_interrupt_ = false;
  auto client_node_func = std::bind(
      static_cast<void(RobotLocalPlannerNodeBase::*)(std::function<bool()>)>(&RobotLocalPlannerNodeBase::Run),
      client_node_, std::placeholders::_1);
  client_thread_ = std::thread(client_node_func, [this]() { return is_client_interrupt_; });

  PublishRobotState();
}

void RobotLocalPlannerNodeWithActionTest::TearDown() {
  auto spin_thread = std::thread(std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSomeLoop, this));

  is_client_interrupt_ = true;
  client_thread_.join();

  is_spin_interrupt_ = true;
  spin_thread.join();
}

void RobotLocalPlannerNodeWithActionTest::WaitFor(TrajectorySubscriber::Ptr trajectory) {
  const auto timeout = server_node_->now() + rclcpp::Duration::from_seconds(10.0);
  while (rclcpp::ok()) {
    if (trajectory->IsSubscribed()) {
      break;
    }
    if (server_node_->now() > timeout) {
      FAIL();
    }
    SpinSome();
  }
}

void RobotLocalPlannerNodeWithActionTest::WaitFor(bool is_constraints_empty) {
  const auto timeout = server_node_->now() + rclcpp::Duration::from_seconds(10.0);
  while (rclcpp::ok()) {
    if (is_constraints_empty_->IsSubscribed() && is_constraints_empty_->GetValue().data == is_constraints_empty) {
      return;
    }
    if (server_node_->now() > timeout) {
      FAIL();
    }
    SpinSome();
  }
}

void RobotLocalPlannerNodeWithActionTest::WaitForTimeout(double timeout) {
  const auto end_time = server_node_->now() + rclcpp::Duration::from_seconds(timeout);
  while (rclcpp::ok()) {
    if (is_constraints_empty_->IsSubscribed() && is_constraints_empty_->GetValue().data) {
      FAIL();
      return;
    }
    if (server_node_->now() > end_time) {
      return;
    }
    SpinSome();
  }
}

void RobotLocalPlannerNodeWithActionTest::PublishRobotState(double arm_flex_position) {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"head_pan_joint", "head_tilt_joint", "arm_lift_joint", "arm_flex_joint",
                      "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint", "hand_motor_joint"};
  joint_state.position = {0.0, 0.0, 0.0, arm_flex_position, 0.0, 0.0, 0.0, 0.0};
  joint_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  control_msgs::msg::JointTrajectoryControllerState base_state;
  base_state.joint_names = {"odom_x", "odom_y", "odom_t"};
  base_state.desired.positions = {0.0, 0.0, 0.0};
  base_state.desired.velocities = {0.0, 0.0, 0.0};
  base_state.actual.positions = {0.0, 0.0, 0.0};
  base_state.actual.velocities = {0.0, 0.0, 0.0};

  // There is no way to detect the optimization node that has received the topic, so make a proper number of times.
  for (auto i = 0; i < 20; i++) {
    joint_state_publisher_->publish(joint_state);
    base_state_publisher_->publish(base_state);
    SpinSome();
  }
}

void RobotLocalPlannerNodeWithActionTest::PublishRobotLocalGoal(
    const tmc_planning_msgs::msg::RangeJointConstraint& rjc,
    bool enable_arm, bool enable_head, bool enable_gripper, bool enable_base,
    const std::string& id) {
  tmc_planning_msgs::msg::RobotLocalGoal goal;
  goal.id = id;
  goal.constraints.hard_joint_constraints = {rjc};
  goal.normalized_velocity = 0.5;
  goal.enable_arm = enable_arm;
  goal.enable_head = enable_head;
  goal.enable_gripper = enable_gripper;
  goal.enable_base = enable_base;
  constraints_publisher_->publish(goal);
}

TEST_F(RobotLocalPlannerNodeWithActionTest, AllTrajectoryPub) {
  PublishRobotLocalGoal(rjc_, true, true, true, true);
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::SUCCESS,
                                           std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSome, this)));

  WaitFor(false);
  WaitFor(true);
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::CONSTRAINTS_EMPTY,
                                           std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSome, this)));

  EXPECT_TRUE(head_trajectory_->IsSubscribed());
  EXPECT_TRUE(arm_trajectory_->IsSubscribed());
  EXPECT_TRUE(hand_trajectory_->IsSubscribed());
  EXPECT_TRUE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(arm_trajectory_->GetValue(), "arm_flex_joint", -1.57, kEpsilon));
}

TEST_F(RobotLocalPlannerNodeWithActionTest, DisplacementWithCurrentState) {
  // Overwrite the parameters and re -initialize
  is_client_interrupt_ = true;
  client_thread_.join();

  rclcpp::NodeOptions options = client_node_->get_node_options();
  options.parameter_overrides().push_back(rclcpp::Parameter("use_current_state_for_displacement", true));
  client_node_ = std::make_shared<RobotLocalPlannerNodeWithAction>(options);

  is_client_interrupt_ = false;
  auto client_node_func = std::bind(
      static_cast<void(RobotLocalPlannerNodeBase::*)(std::function<bool()>)>(&RobotLocalPlannerNodeBase::Run),
      client_node_, std::placeholders::_1);
  client_thread_ = std::thread(client_node_func, [this]() { return is_client_interrupt_; });

  PublishRobotState();

  // Since Jointstate is not updated, the operation should not be completed
  PublishRobotLocalGoal(rjc_, true, true, true, true);
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::SUCCESS,
                                           std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSome, this)));

  WaitForTimeout(7.0);

  // If you update Jointstate, the operation should be completed
  PublishRobotState(-1.57);
  WaitFor(true);
}

TEST_F(RobotLocalPlannerNodeWithActionTest, ArmTrajectoryPub) {
  PublishRobotLocalGoal(rjc_, true, false, false, false);

  WaitFor(false);
  WaitFor(true);

  EXPECT_FALSE(head_trajectory_->IsSubscribed());
  EXPECT_TRUE(arm_trajectory_->IsSubscribed());
  EXPECT_FALSE(hand_trajectory_->IsSubscribed());
  EXPECT_FALSE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(arm_trajectory_->GetValue(), "arm_flex_joint", -1.57, kEpsilon));
}

TEST_F(RobotLocalPlannerNodeWithActionTest, HeadTrajectoryPub) {
  rjc_.min.joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 0.0};
  rjc_.max = rjc_.min;
  PublishRobotLocalGoal(rjc_, false, true, false, false);

  WaitFor(false);
  WaitFor(true);

  EXPECT_TRUE(head_trajectory_->IsSubscribed());
  EXPECT_FALSE(arm_trajectory_->IsSubscribed());
  EXPECT_FALSE(hand_trajectory_->IsSubscribed());
  EXPECT_FALSE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(head_trajectory_->GetValue(), "head_pan_joint", 1.57, kEpsilon));
}

TEST_F(RobotLocalPlannerNodeWithActionTest, HandTrajectoryPub) {
  rjc_.min.joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  rjc_.max = rjc_.min;
  PublishRobotLocalGoal(rjc_, false, false, true, false);

  WaitFor(false);
  WaitFor(true);

  EXPECT_FALSE(head_trajectory_->IsSubscribed());
  EXPECT_FALSE(arm_trajectory_->IsSubscribed());
  EXPECT_TRUE(hand_trajectory_->IsSubscribed());
  EXPECT_FALSE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(hand_trajectory_->GetValue(), "hand_motor_joint", 1.0, kEpsilon));
}

TEST_F(RobotLocalPlannerNodeWithActionTest, BaseTrajectoryPub) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 1.0;
  transform.rotation.w = 1.0;
  rjc_.min.multi_dof_joint_state.transforms[0] = transform;
  rjc_.max = rjc_.min;
  PublishRobotLocalGoal(rjc_, false, false, false, true);

  WaitFor(base_trajectory_);

  // Default parameters that use acceleration to delete several orbit points
  double d = 0.5 * 0.5 * std::pow(0.2, 2);
  EXPECT_GT(base_trajectory_->GetValue().points[0].positions[0], d);

  WaitFor(true);

  EXPECT_FALSE(head_trajectory_->IsSubscribed());
  EXPECT_FALSE(arm_trajectory_->IsSubscribed());
  EXPECT_FALSE(hand_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(base_trajectory_->GetValue(), "odom_x", 1.0, kEpsilon));
}

TEST_F(RobotLocalPlannerNodeWithActionTest, StopTrajectoryPub) {
  PublishRobotLocalGoal(rjc_, true, true, true, true);

  WaitFor(head_trajectory_);
  WaitFor(arm_trajectory_);
  WaitFor(hand_trajectory_);
  WaitFor(base_trajectory_);

  // Throw an empty constraint while moving
  tmc_planning_msgs::msg::RobotLocalGoal goal;
  constraints_publisher_->publish(goal);

  const auto timeout = server_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (rclcpp::ok() && server_node_->now() < timeout) {
    SpinSome();
  }

  // The stop trajectory is issued, but the gripper does not work because it may drop the ingredient if you move it poorly.
  EXPECT_TRUE(head_trajectory_->IsSubscribed());
  EXPECT_TRUE(arm_trajectory_->IsSubscribed());
  EXPECT_TRUE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(head_trajectory_->GetValue().points.empty());
  EXPECT_TRUE(arm_trajectory_->GetValue().points.empty());
  EXPECT_TRUE(base_trajectory_->GetValue().points.empty());

  EXPECT_FALSE(hand_trajectory_->GetValue().points.empty());
}

TEST_F(RobotLocalPlannerNodeWithActionTest, ChangeConstraintsPub) {
  PublishRobotLocalGoal(rjc_, true, true, true, true, "first");

  WaitFor(head_trajectory_);

  EXPECT_TRUE(displacements_cache_->IsSubscribed());
  {
    const auto displacements = displacements_cache_->GetValue();
    EXPECT_EQ(displacements.id, "first");
  }

  // Throw a constraint of another command when moving
  rjc_.min.joint_state.position = {0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0};
  rjc_.max = rjc_.min;
  PublishRobotLocalGoal(rjc_, true, true, true, true, "second");

  WaitFor(true);

  {
    const auto displacements = displacements_cache_->GetValue();
    EXPECT_EQ(displacements.id, "second");
  }


  EXPECT_TRUE(head_trajectory_->IsSubscribed());
  EXPECT_TRUE(arm_trajectory_->IsSubscribed());
  EXPECT_TRUE(hand_trajectory_->IsSubscribed());
  EXPECT_TRUE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(arm_trajectory_->GetValue(), "arm_flex_joint", 0.0, kEpsilon));
  EXPECT_TRUE(CheckTrajectory(arm_trajectory_->GetValue(), "wrist_flex_joint", -1.57, kEpsilon));

  const auto& first_running_stamps = planner_status_sub_->ExtractStatusStamps(
      {{"first", tmc_planning_msgs::msg::ConstraintsStatus::RUNNING}});
  ASSERT_FALSE(first_running_stamps.empty());

  const auto& second_running_stamps = planner_status_sub_->ExtractStatusStamps(
      {{"first", tmc_planning_msgs::msg::ConstraintsStatus::PREEMPTED},
       {"second", tmc_planning_msgs::msg::ConstraintsStatus::RUNNING}});
  ASSERT_FALSE(second_running_stamps.empty());

  const auto& second_done_stamps = planner_status_sub_->ExtractStatusStamps(
      {{"second", tmc_planning_msgs::msg::ConstraintsStatus::SATISFIED}});
  ASSERT_FALSE(second_done_stamps.empty());

  EXPECT_LT(first_running_stamps.back(), second_running_stamps.front());
  EXPECT_LT(second_running_stamps.back(), second_done_stamps.front());
}

// Patterns that fail in the first PlanPath and succeed in the second time
TEST_F(RobotLocalPlannerNodeWithActionTest, FailFirstTimeSucceedSecondTime) {
  generator_->SetDoAbort(true);

  PublishRobotLocalGoal(rjc_, true, true, true, true);
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::GENERATION_FAILURE,
                                           std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSome, this)));
  const auto start = server_node_->now();

  const auto timeout = server_node_->now() + rclcpp::Duration::from_seconds(10.0);
  while (rclcpp::ok()) {
    if (generator_->IsActionCalled()) {
      generator_->SetDoAbort(false);
      break;
    }
    if (server_node_->now() > timeout) {
      FAIL();
    }
    SpinSome();
  }

  WaitFor(arm_trajectory_);
  EXPECT_GT((server_node_->now() - start).seconds(), 0.2);
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::SUCCESS,
                                           std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSome, this)));

  WaitFor(true);
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::CONSTRAINTS_EMPTY,
                                           std::bind(&RobotLocalPlannerNodeWithActionTest::SpinSome, this)));

  EXPECT_TRUE(head_trajectory_->IsSubscribed());
  EXPECT_TRUE(arm_trajectory_->IsSubscribed());
  EXPECT_TRUE(hand_trajectory_->IsSubscribed());
  EXPECT_TRUE(base_trajectory_->IsSubscribed());

  EXPECT_TRUE(CheckTrajectory(arm_trajectory_->GetValue(), "arm_flex_joint", -1.57, kEpsilon));
}

}  // namespace hsrb_robot_local_planner_node

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

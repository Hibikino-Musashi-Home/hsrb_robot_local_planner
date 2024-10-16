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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_utils/caching_subscriber.hpp>

#include "../src/hsrb_robot_local_planner_node/node.hpp"
#include "utils.hpp"


namespace hsrb_robot_local_planner_node {

class TimeStampSubscriber : public tmc_utils::CachingSubscriber<builtin_interfaces::msg::Time> {
 public:
  using Ptr = std::shared_ptr<TimeStampSubscriber>;

  TimeStampSubscriber(const rclcpp::Node::SharedPtr& node, const std::string& topic_name)
      : CachingSubscriber(node, topic_name) {}
  virtual ~TimeStampSubscriber() = default;
};

class RobotLocalPlannerNodeWithPluginTest : public ::testing::Test {
 public:
  void SetUp() override;
  void TearDown() override;

  void SpinSome() {
    rclcpp::spin_some(rlp_node_);
    rclcpp::spin_some(test_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void SpinSomeLoop() {
    is_spin_interrupt_ = false;
    while (!is_spin_interrupt_) {
      SpinSome();
    }
  }

  std::shared_ptr<RobotLocalPlannerNodeWithPlugin> rlp_node_;
  std::thread rlp_thread_;
  bool is_rlp_interrupt_;
  bool is_spin_interrupt_;

  rclcpp::Node::SharedPtr test_node_;

  TimeStampSubscriber::Ptr generator_initialized_stamp_;
  TimeStampSubscriber::Ptr generator_executed_stamp_;
  TimeStampSubscriber::Ptr evaluator_initialized_stamp_;
  TimeStampSubscriber::Ptr evaluator_executed_stamp_;
  TimeStampSubscriber::Ptr validator_initialized_stamp_;
  TimeStampSubscriber::Ptr validator_executed_stamp_;
  TimeStampSubscriber::Ptr optimizer_initialized_stamp_;
  TimeStampSubscriber::Ptr optimizer_executed_stamp_;
  void WaitFor(TimeStampSubscriber::Ptr stamp_sub, const rclcpp::Time& stamp);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr base_state_publisher_;
  void PublishRobotState();

  rclcpp::Publisher<tmc_planning_msgs::msg::RobotLocalGoal>::SharedPtr constraints_publisher_;
  void PublishRobotLocalGoal(const tmc_planning_msgs::msg::RangeJointConstraint& rjc);

  RobotLocalPlannerStatusSubscriber::Ptr planner_status_sub_;
};

void RobotLocalPlannerNodeWithPluginTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("robot_description_kinematics", tmc_manipulation_tests::hsrb::GetUrdf()),
      rclcpp::Parameter("generate_action", "hsrb_robot_local_planner_node/GeneratorPlugin"),
      rclcpp::Parameter("evaluate_action", "hsrb_robot_local_planner_node/EvaluatorPlugin"),
      rclcpp::Parameter("validate_action", "hsrb_robot_local_planner_node/ValidatorPlugin"),
      rclcpp::Parameter("optimize_action", "hsrb_robot_local_planner_node/OptimizerPlugin"),
      rclcpp::Parameter("head_trajectory_controller.joints",
                        std::vector<std::string>({"head_pan_joint", "head_tilt_joint"})),
      rclcpp::Parameter("arm_trajectory_controller.joints",
                        std::vector<std::string>({"arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
                                                  "wrist_flex_joint", "wrist_roll_joint"})),
      rclcpp::Parameter("gripper_controller.joints", std::vector<std::string>({"hand_motor_joint"})),
      rclcpp::Parameter("omni_base_controller.joints", std::vector<std::string>({"odom_x", "odom_y", "odom_t"}))};

  rlp_node_ = std::make_shared<RobotLocalPlannerNodeWithPlugin>(options);
  test_node_ = rclcpp::Node::make_shared("test");

  generator_initialized_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "generator/initialized_stamp");
  generator_executed_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "generator/executed_stamp");
  evaluator_initialized_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "evaluator/initialized_stamp");
  evaluator_executed_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "evaluator/executed_stamp");
  validator_initialized_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "validator/initialized_stamp");
  validator_executed_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "validator/executed_stamp");
  optimizer_initialized_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "optimizer/initialized_stamp");
  optimizer_executed_stamp_ = std::make_shared<TimeStampSubscriber>(test_node_, "optimizer/executed_stamp");

  planner_status_sub_ = std::make_shared<RobotLocalPlannerStatusSubscriber>(test_node_);

  joint_state_publisher_ = test_node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SystemDefaultsQoS());
  base_state_publisher_ = test_node_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      "base_trajectory_controller_state", rclcpp::SystemDefaultsQoS());
  constraints_publisher_ = test_node_->create_publisher<tmc_planning_msgs::msg::RobotLocalGoal>(
      "hsrb_robot_local_planner/constraints", rclcpp::SystemDefaultsQoS());

  const auto stamp = test_node_->now();

  is_rlp_interrupt_ = false;
  auto rlp_node_func = std::bind(
      static_cast<void(RobotLocalPlannerNodeBase::*)(std::function<bool()>)>(&RobotLocalPlannerNodeBase::Run),
      rlp_node_, std::placeholders::_1);
  rlp_thread_ = std::thread(rlp_node_func, [this]() { return is_rlp_interrupt_; });

  PublishRobotState();

  WaitFor(generator_initialized_stamp_, stamp);
  WaitFor(evaluator_initialized_stamp_, stamp);
  WaitFor(validator_initialized_stamp_, stamp);
  WaitFor(optimizer_initialized_stamp_, stamp);
}

void RobotLocalPlannerNodeWithPluginTest::TearDown() {
  auto spin_thread = std::thread(std::bind(&RobotLocalPlannerNodeWithPluginTest::SpinSomeLoop, this));

  is_rlp_interrupt_ = true;
  rlp_thread_.join();

  is_spin_interrupt_ = true;
  spin_thread.join();
}

void RobotLocalPlannerNodeWithPluginTest::WaitFor(TimeStampSubscriber::Ptr stamp_sub,
                                                  const rclcpp::Time& stamp) {
  const auto timeout = test_node_->now() + rclcpp::Duration::from_seconds(3.0);
  while (rclcpp::ok()) {
    if (stamp_sub->IsSubscribed() && (rclcpp::Time(stamp_sub->GetValue()) > stamp)) {
      break;
    }
    if (test_node_->now() > timeout) {
      FAIL();
    }
    SpinSome();
  }
}

void RobotLocalPlannerNodeWithPluginTest::PublishRobotState() {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"head_pan_joint", "head_tilt_joint", "arm_lift_joint", "arm_flex_joint",
                      "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint", "hand_motor_joint"};
  joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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

void RobotLocalPlannerNodeWithPluginTest::PublishRobotLocalGoal(
    const tmc_planning_msgs::msg::RangeJointConstraint& rjc) {
  tmc_planning_msgs::msg::RobotLocalGoal goal;
  goal.constraints.hard_joint_constraints = {rjc};
  goal.normalized_velocity = 0.5;
  goal.enable_arm = true;
  goal.enable_head = true;
  goal.enable_gripper = true;
  goal.enable_base = true;
  constraints_publisher_->publish(goal);
}

TEST_F(RobotLocalPlannerNodeWithPluginTest, Normal) {
  // Since there is a WITH_ACTION test, we only need to check the use of plugin (generated by setup) and use.
  const auto stamp = test_node_->now();

  PublishRobotLocalGoal(GenerateRangeJointConstraint());
  EXPECT_TRUE(planner_status_sub_->WaitFor(tmc_planning_msgs::msg::RobotLocalPlannerStatus::SUCCESS,
                                           std::bind(&RobotLocalPlannerNodeWithPluginTest::SpinSome, this)));
  // I want to put the test target on the left in a consistent, but I can't compare unless I bring rclpy :: time to the left.
  EXPECT_LT(stamp, planner_status_sub_->GetValue().header.stamp);
  EXPECT_GT(test_node_->now(), planner_status_sub_->GetValue().header.stamp);

  WaitFor(generator_executed_stamp_, stamp);
  WaitFor(evaluator_executed_stamp_, stamp);
  WaitFor(validator_executed_stamp_, stamp);
  WaitFor(optimizer_executed_stamp_, stamp);
}

}  // namespace hsrb_robot_local_planner_node

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

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
#include <tmc_robot_local_planner/range_joint_constraint.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>
#include <tmc_utils/caching_subscriber.hpp>

#include "../src/hsrb_robot_local_planner_node/utils.hpp"

namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

namespace hsrb_robot_local_planner_node {

class DisplacementCheckerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  rclcpp::Node::SharedPtr client_node_;
  tmc_utils::CachingSubscriber<tmc_planning_msgs::msg::RobotDisplacements>::Ptr displacements_cache_;

  rclcpp::Node::SharedPtr server_node_;
  std::shared_ptr<DisplacementChecker> displacement_checker_;
};

void DisplacementCheckerTest::SetUp() {
  client_node_ = rclcpp::Node::make_shared("client");
  displacements_cache_ = std::make_shared<tmc_utils::CachingSubscriber<tmc_planning_msgs::msg::RobotDisplacements>>(
      client_node_, "server/displacements");

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("robot_description", tmc_manipulation_tests::hsrb::GetUrdf())};
  server_node_ = rclcpp::Node::make_shared("server", options);

  displacement_checker_ = std::make_shared<DisplacementChecker>(server_node_);
}

TEST_F(DisplacementCheckerTest, DisplacementPublishment) {
  tmc_robot_local_planner::Constraints constraints;

  {
    tmc_manipulation_types::RobotState min;
    min.joint_state.name = {"head_pan_joint", "head_tilt_joint"};
    min.joint_state.position.resize(2);
    min.joint_state.position << -0.1, -0.2;
    min.multi_dof_joint_state.names = {"world_joint"};
    min.multi_dof_joint_state.poses = {
        Eigen::Translation3d(-0.1, 0.3, 0.0) * Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ())};

    tmc_manipulation_types::RobotState max;
    max.joint_state.name = {"head_pan_joint", "head_tilt_joint"};
    max.joint_state.position.resize(2);
    max.joint_state.position << 0.1, 0.2;
    max.multi_dof_joint_state.names = {"world_joint"};
    max.multi_dof_joint_state.poses = {
        Eigen::Translation3d(0.1, 0.5, 0.0) * Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ())};
    constraints.hard_joint_constraints.push_back(
        std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(min, max, 0));
  }
  {
    tmc_manipulation_types::RegionValues min;
    min << 0.0, 0.0, 0.0, 0.0, 0.0, 0.1;
    tmc_manipulation_types::RegionValues max;
    max << 0.0, 0.1, 0.2, 0.0, 0.0, 0.5;
    // HSR-B's joint position with zero joint position
    tmc_manipulation_types::TaskSpaceRegion tsr(
        Eigen::Translation3d(0.158, 0.078, 0.825) * Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0),
        Eigen::Affine3d::Identity(), min, max, "odom", "hand_palm_link");
    constraints.hard_link_constraints.push_back(std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0));
  }

  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state.name = {"head_pan_joint", "head_tilt_joint", "arm_lift_joint"};
  robot_state.joint_state.position.resize(3);
  robot_state.joint_state.position << 0.2, -0.5, 0.6;
  robot_state.multi_dof_joint_state.names = {"world_joint"};
  robot_state.multi_dof_joint_state.poses = {Eigen::Translation3d(-0.1, 0.3, 0.0) * Eigen::AngleAxisd::Identity()};


  displacement_checker_->UpdateConstraints(constraints);
  EXPECT_FALSE(displacement_checker_->ShouldComplete("constraints", robot_state));

  while (rclcpp::ok()) {
    SpinSome();
    if (displacements_cache_->IsSubscribed() && displacements_cache_->GetValue().id == "constraints") {
      break;
    }
  }
  ASSERT_TRUE(displacements_cache_->IsSubscribed());

  const auto displacements = displacements_cache_->GetValue();
  EXPECT_EQ(displacements.id, "constraints");
  EXPECT_NEAR(displacements.min_displacement, std::sqrt(0.1 * 0.1 + 0.3 * 0.3) + 0.1, kEpsilon);

  ASSERT_EQ(displacements.joint_displacements.size(), 1);
  EXPECT_EQ(displacements.joint_displacements[0].joint_names,
            std::vector<std::string>({"head_pan_joint", "head_tilt_joint"}));
  EXPECT_EQ(displacements.joint_displacements[0].joint_values.size(), 2);
  EXPECT_NEAR(displacements.joint_displacements[0].joint_values[0], 0.1, kEpsilon);
  EXPECT_NEAR(displacements.joint_displacements[0].joint_values[1], 0.3, kEpsilon);
  EXPECT_EQ(displacements.joint_displacements[0].base_names, std::vector<std::string>({"world_joint"}));
  EXPECT_EQ(displacements.joint_displacements[0].base_values.size(), 1);
  EXPECT_NEAR(displacements.joint_displacements[0].base_values[0], 0.1, kEpsilon);

  ASSERT_EQ(displacements.link_displacements.size(), 1);
  EXPECT_NEAR(displacements.link_displacements[0].xyz[0], 0.1, kEpsilon);
  EXPECT_NEAR(displacements.link_displacements[0].xyz[1], -0.3, kEpsilon);
  EXPECT_NEAR(displacements.link_displacements[0].xyz[2], 0.4, kEpsilon);
  EXPECT_NEAR(displacements.link_displacements[0].rpy[0], 0.0, kEpsilon);
  EXPECT_NEAR(displacements.link_displacements[0].rpy[1], 0.0, kEpsilon);
  EXPECT_NEAR(displacements.link_displacements[0].rpy[2], -0.1, kEpsilon);
}
}  // namespace hsrb_robot_local_planner_node


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

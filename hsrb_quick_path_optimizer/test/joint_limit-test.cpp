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
/// @brief Function group to obtain optimization parameters

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "../src/joint_limit.hpp"

namespace hsrb_quick_path_optimizer {

class JointLimitTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;

    node_ = rclcpp::Node::make_shared("test_node");
    node_->declare_parameter("joint_1.velocity", 1.0, descriptor);
    node_->declare_parameter("joint_2.velocity", 2.0, descriptor);
    node_->declare_parameter("odom_x.velocity", 3.0, descriptor);
    node_->declare_parameter("odom_y.velocity", 4.0, descriptor);
    node_->declare_parameter("odom_t.velocity", 5.0, descriptor);
    node_->declare_parameter("joint_1.acceleration", 1.1, descriptor);
    node_->declare_parameter("joint_2.acceleration", 2.1, descriptor);
    node_->declare_parameter("odom_x.acceleration", 3.1, descriptor);
    node_->declare_parameter("odom_y.acceleration", 4.1, descriptor);
    node_->declare_parameter("odom_t.acceleration", 5.1, descriptor);
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(JointLimitTest, GetVelocityLimitSuccess) {
  Eigen::VectorXd result;
  ASSERT_TRUE(GetVelocityLimit(node_, {"joint_1", "joint_2"}, result));

  ASSERT_EQ(5, result.size());
  EXPECT_DOUBLE_EQ(1.0, result[0]);
  EXPECT_DOUBLE_EQ(2.0, result[1]);
  EXPECT_DOUBLE_EQ(3.0, result[2]);
  EXPECT_DOUBLE_EQ(4.0, result[3]);
  EXPECT_DOUBLE_EQ(5.0, result[4]);
}

TEST_F(JointLimitTest, GetVelocityLimitNoOdom) {
  node_->undeclare_parameter("odom_t.velocity");

  Eigen::VectorXd result;
  ASSERT_FALSE(GetVelocityLimit(node_, {"joint_1", "joint_2"}, result));
}

TEST_F(JointLimitTest, GetVelocityLimitNoJoint) {
  node_->undeclare_parameter("joint_2.velocity");

  Eigen::VectorXd result;
  ASSERT_FALSE(GetVelocityLimit(node_, {"joint_1", "joint_2"}, result));
}

TEST_F(JointLimitTest, GetAccelerationLimitSuccess) {
  Eigen::VectorXd result;
  ASSERT_TRUE(GetAccelerationLimit(node_, {"joint_1", "joint_2"}, result));

  ASSERT_EQ(5, result.size());
  EXPECT_DOUBLE_EQ(1.1, result[0]);
  EXPECT_DOUBLE_EQ(2.1, result[1]);
  EXPECT_DOUBLE_EQ(3.1, result[2]);
  EXPECT_DOUBLE_EQ(4.1, result[3]);
  EXPECT_DOUBLE_EQ(5.1, result[4]);
}

TEST_F(JointLimitTest, GetAccelerationLimitNoOdom) {
  node_->undeclare_parameter("odom_t.acceleration");

  Eigen::VectorXd result;
  ASSERT_FALSE(GetAccelerationLimit(node_, {"joint_1", "joint_2"}, result));
}

TEST_F(JointLimitTest, GetAccelerationLimitNoJoint) {
  node_->undeclare_parameter("joint_2.acceleration");

  Eigen::VectorXd result;
  ASSERT_FALSE(GetAccelerationLimit(node_, {"joint_1", "joint_2"}, result));
}
}  // namespace hsrb_quick_path_optimizer

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

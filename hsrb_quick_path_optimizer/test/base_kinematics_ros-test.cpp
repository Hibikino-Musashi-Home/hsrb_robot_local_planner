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
/// @brief A parameter class test for bogie athletic calculation after HSR-B

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "../src/base_kinematics_ros.hpp"

namespace hsrb_quick_path_optimizer {

TEST(BaseKinematicsROSTest, BaseJointLimitsWithParam) {
  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter("max_caster_velocity", 1.0);
  node->declare_parameter("max_wheel_velocity", 2.0);
  node->declare_parameter("max_caster_acceleration", 3.0);
  node->declare_parameter("max_wheel_acceleration", 4.0);

  auto limits = BaseJointLimitsRos(node);

  EXPECT_DOUBLE_EQ(1.0, limits.caster_velocity);
  EXPECT_DOUBLE_EQ(2.0, limits.wheel_velocity);
  EXPECT_DOUBLE_EQ(3.0, limits.caster_acceleration);
  EXPECT_DOUBLE_EQ(4.0, limits.wheel_acceleration);
}

TEST(BaseKinematicsROSTest, BaseJointLimitsWithoutParam) {
  auto node = rclcpp::Node::make_shared("test_node");

  auto limits_ros = BaseJointLimitsRos(node);
  auto limits_base = BaseJointLimits();

  EXPECT_DOUBLE_EQ(limits_base.caster_velocity,
                   limits_ros.caster_velocity);
  EXPECT_DOUBLE_EQ(limits_base.wheel_velocity,
                   limits_ros.wheel_velocity);
  EXPECT_DOUBLE_EQ(limits_base.caster_acceleration,
                   limits_ros.caster_acceleration);
  EXPECT_DOUBLE_EQ(limits_base.wheel_acceleration,
                   limits_ros.wheel_acceleration);
}

TEST(BaseKinematicsROSTest, OmniBaseSizeWithParam) {
  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter("tread", 1.0);
  node->declare_parameter("caster_offset", 2.0);
  node->declare_parameter("wheel_radius", 3.0);

  auto size = OmniBaseSizeRos(node);

  EXPECT_DOUBLE_EQ(1.0, size.tread);
  EXPECT_DOUBLE_EQ(2.0, size.caster_offset);
  EXPECT_DOUBLE_EQ(3.0, size.wheel_radius);
}

TEST(BaseKinematicsROSTest, OmniBaseSizeWithoutParam) {
  auto node = rclcpp::Node::make_shared("test_node");

  auto size_ros = OmniBaseSizeRos(node);
  auto size_base = OmniBaseSize();

  EXPECT_DOUBLE_EQ(size_base.tread, size_ros.tread);
  EXPECT_DOUBLE_EQ(size_base.caster_offset, size_ros.caster_offset);
  EXPECT_DOUBLE_EQ(size_base.wheel_radius, size_ros.wheel_radius);
}
}  // namespace hsrb_quick_path_optimizer

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}

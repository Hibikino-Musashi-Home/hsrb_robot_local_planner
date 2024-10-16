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
/// @brief Athletic for acceleration calculation in bogies after HSR-B
#include <tuple>

#include <gtest/gtest.h>

#include "../src/base_kinematics.hpp"

namespace {
constexpr double kEpsilon = 0.01;
}  // namespace

namespace hsrb_quick_path_optimizer {

// [IN] Test input, [OUT] limit
using OneInputOneResultTuple = std::tuple<double, Eigen::Vector3d>;
// [IN] Test input 1, [IN] Test input 2, [IN] Test input 3, [OUT] Limit
using ThreeInputOneResultTuple =
    std::tuple<double, double, double, Eigen::Vector3d>;

// Speed ​​calculation that moves in arbitrary direction from a still state
class MoveDirectionWithStoppingVelocityTest
    : public ::testing::TestWithParam<OneInputOneResultTuple> {};

// Direction, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    MoveDirectionWithStoppingVelocityTest, MoveDirectionWithStoppingVelocityTest,
    ::testing::Values(
        std::make_tuple(0.0 * M_PI_4, Eigen::Vector3d(0.34, 0.00, 1.80)),
        std::make_tuple(1.0 * M_PI_4, Eigen::Vector3d(0.15, 0.15, 0.39)),
        std::make_tuple(2.0 * M_PI_4, Eigen::Vector3d(0.00, 0.28, 0.76)),
        std::make_tuple(3.0 * M_PI_4, Eigen::Vector3d(0.15, 0.15, 0.39)),
        std::make_tuple(4.0 * M_PI_4, Eigen::Vector3d(0.34, 0.00, 1.80)),
        std::make_tuple(-1.0 * M_PI_4, Eigen::Vector3d(0.15, 0.15, 0.39)),
        std::make_tuple(-2.0 * M_PI_4, Eigen::Vector3d(0.00, 0.28, 0.76)),
        std::make_tuple(-3.0 * M_PI_4, Eigen::Vector3d(0.15, 0.15, 0.39))));

TEST_P(MoveDirectionWithStoppingVelocityTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const auto result = base_kinematics.CalculateBaseMaxVelocity(0.0, std::get<0>(GetParam()), 0.0);

  EXPECT_NEAR(result[0], std::get<1>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<1>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<1>(GetParam())[2], kEpsilon);
}

// Speed ​​calculation according to the bogie turning axis
class BaseRollJointVelocityTest
    : public ::testing::TestWithParam<OneInputOneResultTuple> {};

// Bogie turning shaft angle, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    BaseRollJointVelocityTest, BaseRollJointVelocityTest,
    ::testing::Values(
        std::make_tuple(0.0 * M_PI_4, Eigen::Vector3d(0.34, 0.00, 1.80)),
        std::make_tuple(1.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39)),
        std::make_tuple(2.0 * M_PI_4, Eigen::Vector3d(0.28, 0.00, 0.76)),
        std::make_tuple(3.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39)),
        std::make_tuple(4.0 * M_PI_4, Eigen::Vector3d(0.34, 0.00, 1.80)),
        std::make_tuple(-1.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39)),
        std::make_tuple(-2.0 * M_PI_4, Eigen::Vector3d(0.28, 0.00, 0.76)),
        std::make_tuple(-3.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39))));

TEST_P(BaseRollJointVelocityTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const auto result = base_kinematics.CalculateBaseMaxVelocity(0.0, 0.0, std::get<0>(GetParam()));

  EXPECT_NEAR(result[0], std::get<1>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<1>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<1>(GetParam())[2], kEpsilon);
}

// Speed ​​calculation according to the Origin standard bogie rotation
class BaseRotationVelocityTest
    : public ::testing::TestWithParam<OneInputOneResultTuple> {};

// ORIGIN standard bogie posture YAW, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    BaseRotationVelocityTest, BaseRotationVelocityTest,
    ::testing::Values(
        std::make_tuple(0.0 * M_PI_4, Eigen::Vector3d(0.34, 0.00, 1.80)),
        std::make_tuple(1.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39)),
        std::make_tuple(2.0 * M_PI_4, Eigen::Vector3d(0.28, 0.00, 0.76)),
        std::make_tuple(3.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39)),
        std::make_tuple(4.0 * M_PI_4, Eigen::Vector3d(0.34, 0.00, 1.80)),
        std::make_tuple(-1.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39)),
        std::make_tuple(-2.0 * M_PI_4, Eigen::Vector3d(0.28, 0.00, 0.76)),
        std::make_tuple(-3.0 * M_PI_4, Eigen::Vector3d(0.22, 0.00, 0.39))));

TEST_P(BaseRotationVelocityTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const auto result = base_kinematics.CalculateBaseMaxVelocity(std::get<0>(GetParam()), 0.0, 0.0);

  EXPECT_NEAR(result[0], std::get<1>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<1>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<1>(GetParam())[2], kEpsilon);
}

// Acceleration calculation that moves in any direction from a still state
class MoveDirectionWithStoppingAccelerationTest
    : public ::testing::TestWithParam<OneInputOneResultTuple> {};

// Direction, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    MoveDirectionWithStoppingAccelerationTest, MoveDirectionWithStoppingAccelerationTest,
    ::testing::Values(
        std::make_tuple(0.0 * M_PI_4, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(1.0 * M_PI_4, Eigen::Vector3d(0.09, 0.09, 0.98)),
        std::make_tuple(2.0 * M_PI_4, Eigen::Vector3d(0.00, 0.17, 0.30)),
        std::make_tuple(3.0 * M_PI_4, Eigen::Vector3d(0.09, 0.09, 0.98)),
        std::make_tuple(4.0 * M_PI_4, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(-1.0 * M_PI_4, Eigen::Vector3d(0.09, 0.09, 0.98)),
        std::make_tuple(-2.0 * M_PI_4, Eigen::Vector3d(0.00, 0.17, 0.30)),
        std::make_tuple(-3.0 * M_PI_4, Eigen::Vector3d(0.09, 0.09, 0.98))));

TEST_P(MoveDirectionWithStoppingAccelerationTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const auto result = base_kinematics.CalculateBaseMaxAcceleration(
      0.0, std::get<0>(GetParam()), 0.0, Eigen::Vector3d::Zero());

  EXPECT_NEAR(result[0], std::get<1>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<1>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<1>(GetParam())[2], kEpsilon);
}

// Acceleration calculation according to the bogie turning axis
class BaseRollJointAccelerationTest
    : public ::testing::TestWithParam<OneInputOneResultTuple> {};

// Bogie turning shaft angle, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    BaseRollJointAccelerationTest, BaseRollJointAccelerationTest,
    ::testing::Values(
        std::make_tuple(0.0 * M_PI_4, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(1.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98)),
        std::make_tuple(2.0 * M_PI_4, Eigen::Vector3d(0.17, 0.00, 0.30)),
        std::make_tuple(3.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98)),
        std::make_tuple(4.0 * M_PI_4, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(-1.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98)),
        std::make_tuple(-2.0 * M_PI_4, Eigen::Vector3d(0.17, 0.00, 0.30)),
        std::make_tuple(-3.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98))));

TEST_P(BaseRollJointAccelerationTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const auto result = base_kinematics.CalculateBaseMaxAcceleration(
      0.0, 0.0, std::get<0>(GetParam()), Eigen::Vector3d::Zero());

  EXPECT_NEAR(result[0], std::get<1>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<1>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<1>(GetParam())[2], kEpsilon);
}

// Acceleration calculation according to Origin standard bogie rotation
class BaseRotationAccelerationTest
    : public ::testing::TestWithParam<OneInputOneResultTuple> {};

// ORIGIN standard bogie posture YAW, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    BaseRotationAccelerationTest, BaseRotationAccelerationTest,
    ::testing::Values(
        std::make_tuple(0.0 * M_PI_4, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(1.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98)),
        std::make_tuple(2.0 * M_PI_4, Eigen::Vector3d(0.17, 0.00, 0.30)),
        std::make_tuple(3.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98)),
        std::make_tuple(4.0 * M_PI_4, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(-1.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98)),
        std::make_tuple(-2.0 * M_PI_4, Eigen::Vector3d(0.17, 0.00, 0.30)),
        std::make_tuple(-3.0 * M_PI_4, Eigen::Vector3d(0.13, 0.00, 0.98))));

TEST_P(BaseRotationAccelerationTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const auto result = base_kinematics.CalculateBaseMaxAcceleration(
      std::get<0>(GetParam()), 0.0, 0.0, Eigen::Vector3d::Zero());

  EXPECT_NEAR(result[0], std::get<1>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<1>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<1>(GetParam())[2], kEpsilon);
}

// Acceleration calculation according to each axis speed
class JointVelocityInputTest
    : public ::testing::TestWithParam<ThreeInputOneResultTuple> {};

// Right wheel speed, left wheel speed, turning axis speed, expected calculation result group
INSTANTIATE_TEST_CASE_P(
    JointVelocityInputTest, JointVelocityInputTest,
    ::testing::Values(
        std::make_tuple(1.0, 1.0, 1.0, Eigen::Vector3d(0.15, 0.00, 1.44)),
        std::make_tuple(1.0, 1.0, 0.0, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(1.0, 1.0, -1.0, Eigen::Vector3d(0.15, 0.00, 1.44)),
        std::make_tuple(1.0, -1.0, 1.0, Eigen::Vector3d(0.17, 0.00, 1.80)),
        std::make_tuple(1.0, -1.0, 0.0, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(1.0, -1.0, -1.0, Eigen::Vector3d(0.23, 0.00, 1.80)),
        std::make_tuple(-1.0, 1.0, 1.0, Eigen::Vector3d(0.23, 0.00, 1.80)),
        std::make_tuple(-1.0, 1.0, 0.0, Eigen::Vector3d(0.20, 0.00, 1.80)),
        std::make_tuple(-1.0, 1.0, -1.0, Eigen::Vector3d(0.17, 0.00, 1.80))));

TEST_P(JointVelocityInputTest, TestCase) {
  auto base_kinematics = BaseKinematics(BaseJointLimits(), OmniBaseSize());
  const Eigen::Vector3d joint_velocity(std::get<0>(GetParam()),
                                       std::get<1>(GetParam()),
                                       std::get<2>(GetParam()));
  const auto result = base_kinematics.CalculateBaseMaxAcceleration(
      0.0, 0.0, 0.0, joint_velocity);

  EXPECT_NEAR(result[0], std::get<3>(GetParam())[0], kEpsilon);
  EXPECT_NEAR(result[1], std::get<3>(GetParam())[1], kEpsilon);
  EXPECT_NEAR(result[2], std::get<3>(GetParam())[2], kEpsilon);
}
}  // namespace hsrb_quick_path_optimizer

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

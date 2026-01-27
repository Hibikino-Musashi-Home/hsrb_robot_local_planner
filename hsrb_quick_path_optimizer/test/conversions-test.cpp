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
/// @brief Test of transformation functions for preprocessing and postprocessing of optimization

#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_timeopt/quick_trajectory_filter.hpp>

#include "../src/conversions.hpp"
#include "../src/trajectory_filter_adapter.hpp"

namespace {

constexpr double kEpsilon = 1.0e-6;

tmc_manipulation_types::TimedRobotTrajectory GenerateMultiDOFOnlyRobotTrajectory() {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.multi_dof_joint_trajectory.points.resize(2);
  trajectory.multi_dof_joint_trajectory.points[0].transforms.push_back(
      Eigen::Translation3d(9.0, 10.0, 0.0) * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  trajectory.multi_dof_joint_trajectory.points[0].velocities.resize(1);
  trajectory.multi_dof_joint_trajectory.points[0].velocities[0] << 11.0, 12.0, 0.0, 0.0, 0.0, 13.0;
  trajectory.multi_dof_joint_trajectory.points[1].transforms.push_back(
      Eigen::Translation3d(14.0, 15.0, 0.0) * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  trajectory.multi_dof_joint_trajectory.points[1].velocities.resize(1);
  trajectory.multi_dof_joint_trajectory.points[1].velocities[0] << 16.0, 17.0, 0.0, 0.0, 0.0, 18.0;
  return trajectory;
}

tmc_manipulation_types::TimedRobotTrajectory GenerateValidRobotTrajectory() {
  auto trajectory = GenerateMultiDOFOnlyRobotTrajectory();

  trajectory.joint_trajectory.points.resize(2);
  trajectory.joint_trajectory.points[0].positions.resize(2);
  trajectory.joint_trajectory.points[0].positions << 1.0, 2.0;
  trajectory.joint_trajectory.points[0].velocities.resize(2);
  trajectory.joint_trajectory.points[0].velocities << 3.0, 4.0;
  trajectory.joint_trajectory.points[1].positions.resize(2);
  trajectory.joint_trajectory.points[1].positions << 5.0, 6.0;
  trajectory.joint_trajectory.points[1].velocities.resize(2);
  trajectory.joint_trajectory.points[1].velocities << 7.0, 8.0;

  return trajectory;
}

}  // namespace

namespace hsrb_quick_path_optimizer {

TEST(ConversionsTest, ExtractInitialPositions) {
  Eigen::VectorXd result;
  ExtractInitialPositions(GenerateValidRobotTrajectory(), result);

  ASSERT_EQ(5, result.size());
  EXPECT_DOUBLE_EQ(1.0, result[0]);
  EXPECT_DOUBLE_EQ(2.0, result[1]);
  EXPECT_DOUBLE_EQ(9.0, result[2]);
  EXPECT_DOUBLE_EQ(10.0, result[3]);
  EXPECT_NEAR(0.2, result[4], kEpsilon);
}

TEST(ConversionsTest, ExtractInitialPositionsMultiDOFOnly) {
  Eigen::VectorXd result;
  ExtractInitialPositions(GenerateMultiDOFOnlyRobotTrajectory(), result);

  ASSERT_EQ(3, result.size());
  EXPECT_DOUBLE_EQ(9.0, result[0]);
  EXPECT_DOUBLE_EQ(10.0, result[1]);
  EXPECT_NEAR(0.2, result[2], kEpsilon);
}

TEST(ConversionsTest, ExtractInitialVelocities) {
  Eigen::VectorXd result;
  ExtractInitialVelocities(GenerateValidRobotTrajectory(), result);

  ASSERT_EQ(5, result.size());
  EXPECT_DOUBLE_EQ(3.0, result[0]);
  EXPECT_DOUBLE_EQ(4.0, result[1]);
  EXPECT_DOUBLE_EQ(11.0, result[2]);
  EXPECT_DOUBLE_EQ(12.0, result[3]);
  EXPECT_DOUBLE_EQ(13.0, result[4]);
}

TEST(ConversionsTest, ExtractInitialVelocitiesMultiDOFOnly) {
  Eigen::VectorXd result;
  ExtractInitialVelocities(GenerateMultiDOFOnlyRobotTrajectory(), result);

  ASSERT_EQ(3, result.size());
  EXPECT_DOUBLE_EQ(11.0, result[0]);
  EXPECT_DOUBLE_EQ(12.0, result[1]);
  EXPECT_DOUBLE_EQ(13.0, result[2]);
}

TEST(ConversionsTest, ConvertToWayPoints) {
  std::vector<Eigen::VectorXd> result;
  ConvertToWayPoints(GenerateValidRobotTrajectory(), result);

  ASSERT_EQ(2, result.size());

  ASSERT_EQ(5, result[0].size());
  EXPECT_DOUBLE_EQ(1.0, result[0][0]);
  EXPECT_DOUBLE_EQ(2.0, result[0][1]);
  EXPECT_DOUBLE_EQ(9.0, result[0][2]);
  EXPECT_DOUBLE_EQ(10.0, result[0][3]);
  EXPECT_NEAR(0.2, result[0][4], kEpsilon);

  ASSERT_EQ(5, result[1].size());
  EXPECT_DOUBLE_EQ(5.0, result[1][0]);
  EXPECT_DOUBLE_EQ(6.0, result[1][1]);
  EXPECT_DOUBLE_EQ(14.0, result[1][2]);
  EXPECT_DOUBLE_EQ(15.0, result[1][3]);
  EXPECT_NEAR(0.4, result[1][4], kEpsilon);
}

TEST(ConversionsTest, ConvertToWayPointsMultiDOFOnly) {
  std::vector<Eigen::VectorXd> result;
  ConvertToWayPoints(GenerateMultiDOFOnlyRobotTrajectory(), result);

  ASSERT_EQ(2, result.size());

  ASSERT_EQ(3, result[0].size());
  EXPECT_DOUBLE_EQ(9.0, result[0][0]);
  EXPECT_DOUBLE_EQ(10.0, result[0][1]);
  EXPECT_NEAR(0.2, result[0][2], kEpsilon);

  ASSERT_EQ(3, result[1].size());
  EXPECT_DOUBLE_EQ(14.0, result[1][0]);
  EXPECT_DOUBLE_EQ(15.0, result[1][1]);
  EXPECT_NEAR(0.4, result[1][2], kEpsilon);
}

TEST(ConversionsTest, ConvertToWayPointsOverPI) {
  auto input = GenerateValidRobotTrajectory();
  input.joint_trajectory.points.push_back(input.joint_trajectory.points[0]);
  input.multi_dof_joint_trajectory.points.push_back(input.multi_dof_joint_trajectory.points[0]);

  input.multi_dof_joint_trajectory.points[0].transforms[0] =
      Eigen::Translation3d(input.multi_dof_joint_trajectory.points[0].transforms[0].translation()) *
      Eigen::AngleAxisd(3.0, Eigen::Vector3d::UnitZ());
  input.multi_dof_joint_trajectory.points[1].transforms[0] =
      Eigen::Translation3d(input.multi_dof_joint_trajectory.points[1].transforms[0].translation()) *
      Eigen::AngleAxisd(-3.0, Eigen::Vector3d::UnitZ());
  input.multi_dof_joint_trajectory.points[2].transforms[0] =
      Eigen::Translation3d(input.multi_dof_joint_trajectory.points[2].transforms[0].translation()) *
      Eigen::AngleAxisd(3.0, Eigen::Vector3d::UnitZ());

  std::vector<Eigen::VectorXd> result;
  ConvertToWayPoints(input, result);

  ASSERT_EQ(3, result.size());

  ASSERT_EQ(5, result[0].size());
  EXPECT_NEAR(3.0, result[0][4], kEpsilon);

  ASSERT_EQ(5, result[1].size());
  EXPECT_NEAR(3.0 + 2.0 * (M_PI - 3.0), result[1][4], kEpsilon);

  ASSERT_EQ(5, result[2].size());
  EXPECT_NEAR(3.0, result[2][4], kEpsilon);
}

class QuickTrajectoryFilterMock : public ITrajectoryFilterAdapter {
 public:
  using Ptr = std::shared_ptr<QuickTrajectoryFilterMock>;

  MOCK_CONST_METHOD1(GetPosition, Eigen::VectorXd(double time_from_start));
  MOCK_CONST_METHOD1(GetVelocity, Eigen::VectorXd(double time_from_start));
  MOCK_CONST_METHOD0(GetDuration, double());
};

TEST(ConversionsTest, ConvertToRobotTrajectory) {
  QuickTrajectoryFilterMock::Ptr filter_mock = std::make_shared<QuickTrajectoryFilterMock>();

  std::vector<Eigen::VectorXd> positions(3, Eigen::VectorXd(5));
  positions[0] << 1.0, 2.0, 3.0, 4.0, 0.2;
  positions[1] << 11.0, 12.0, 13.0, 14.0, 0.4;
  positions[2] << 21.0, 22.0, 23.0, 24.0, 0.6;

  std::vector<Eigen::VectorXd> velocities(3, Eigen::VectorXd(5));
  velocities[0] << 5.0, 6.0, 7.0, 8.0, 9.0;
  velocities[1] << 15.0, 16.0, 17.0, 18.0, 19.0;
  velocities[2] << 25.0, 26.0, 27.0, 28.0, 29.0;

  EXPECT_CALL(*filter_mock, GetDuration())
      .WillRepeatedly(::testing::Return(1.1));
  EXPECT_CALL(*filter_mock, GetPosition(::testing::_))
      .WillOnce(::testing::Return(positions[0]))
      .WillOnce(::testing::Return(positions[1]))
      .WillOnce(::testing::Return(positions[2]));
  EXPECT_CALL(*filter_mock, GetVelocity(::testing::_))
      .WillOnce(::testing::Return(velocities[0]))
      .WillOnce(::testing::Return(velocities[1]))
      .WillOnce(::testing::Return(velocities[2]));

  tmc_manipulation_types::TimedRobotTrajectory result;
  ConvertToRobotTrajectory(filter_mock, {"joint_1", "joint_2"}, "world", 0.5, result);

  ASSERT_EQ(2, result.joint_trajectory.joint_names.size());
  EXPECT_EQ("joint_1", result.joint_trajectory.joint_names[0]);
  EXPECT_EQ("joint_2", result.joint_trajectory.joint_names[1]);
  ASSERT_EQ(3, result.joint_trajectory.points.size());

  ASSERT_EQ(2, result.joint_trajectory.points[0].positions.size());
  EXPECT_DOUBLE_EQ(1.0, result.joint_trajectory.points[0].positions[0]);
  EXPECT_DOUBLE_EQ(2.0, result.joint_trajectory.points[0].positions[1]);
  ASSERT_EQ(2, result.joint_trajectory.points[0].velocities.size());
  EXPECT_DOUBLE_EQ(5.0, result.joint_trajectory.points[0].velocities[0]);
  EXPECT_DOUBLE_EQ(6.0, result.joint_trajectory.points[0].velocities[1]);
  EXPECT_DOUBLE_EQ(0.5, result.joint_trajectory.points[0].time_from_start);

  ASSERT_EQ(2, result.joint_trajectory.points[1].positions.size());
  EXPECT_DOUBLE_EQ(11.0, result.joint_trajectory.points[1].positions[0]);
  EXPECT_DOUBLE_EQ(12.0, result.joint_trajectory.points[1].positions[1]);
  ASSERT_EQ(2, result.joint_trajectory.points[1].velocities.size());
  EXPECT_DOUBLE_EQ(15.0, result.joint_trajectory.points[1].velocities[0]);
  EXPECT_DOUBLE_EQ(16.0, result.joint_trajectory.points[1].velocities[1]);
  EXPECT_DOUBLE_EQ(1.0, result.joint_trajectory.points[1].time_from_start);

  ASSERT_EQ(2, result.joint_trajectory.points[2].positions.size());
  EXPECT_DOUBLE_EQ(21.0, result.joint_trajectory.points[2].positions[0]);
  EXPECT_DOUBLE_EQ(22.0, result.joint_trajectory.points[2].positions[1]);
  ASSERT_EQ(2, result.joint_trajectory.points[2].velocities.size());
  EXPECT_DOUBLE_EQ(25.0, result.joint_trajectory.points[2].velocities[0]);
  EXPECT_DOUBLE_EQ(26.0, result.joint_trajectory.points[2].velocities[1]);
  EXPECT_DOUBLE_EQ(1.1, result.joint_trajectory.points[2].time_from_start);

  ASSERT_EQ(1, result.multi_dof_joint_trajectory.joint_names.size());
  EXPECT_EQ("world", result.multi_dof_joint_trajectory.joint_names[0]);
  ASSERT_EQ(3, result.multi_dof_joint_trajectory.points.size());

  ASSERT_EQ(1, result.multi_dof_joint_trajectory.points[0].transforms.size());
  EXPECT_DOUBLE_EQ(3.0, result.multi_dof_joint_trajectory.points[0].transforms[0].translation().x());
  EXPECT_DOUBLE_EQ(4.0, result.multi_dof_joint_trajectory.points[0].transforms[0].translation().y());
  EXPECT_DOUBLE_EQ(0.2, result.multi_dof_joint_trajectory.points[0].transforms[0].linear().eulerAngles(0, 1, 2)[2]);
  ASSERT_EQ(1, result.multi_dof_joint_trajectory.points[0].velocities.size());
  EXPECT_DOUBLE_EQ(7.0, result.multi_dof_joint_trajectory.points[0].velocities[0][0]);
  EXPECT_DOUBLE_EQ(8.0, result.multi_dof_joint_trajectory.points[0].velocities[0][1]);
  EXPECT_DOUBLE_EQ(9.0, result.multi_dof_joint_trajectory.points[0].velocities[0][5]);

  ASSERT_EQ(1, result.multi_dof_joint_trajectory.points[1].transforms.size());
  EXPECT_DOUBLE_EQ(13.0, result.multi_dof_joint_trajectory.points[1].transforms[0].translation().x());
  EXPECT_DOUBLE_EQ(14.0, result.multi_dof_joint_trajectory.points[1].transforms[0].translation().y());
  EXPECT_DOUBLE_EQ(0.4, result.multi_dof_joint_trajectory.points[1].transforms[0].linear().eulerAngles(0, 1, 2)[2]);
  ASSERT_EQ(1, result.multi_dof_joint_trajectory.points[1].velocities.size());
  EXPECT_DOUBLE_EQ(17.0, result.multi_dof_joint_trajectory.points[1].velocities[0][0]);
  EXPECT_DOUBLE_EQ(18.0, result.multi_dof_joint_trajectory.points[1].velocities[0][1]);
  EXPECT_DOUBLE_EQ(19.0, result.multi_dof_joint_trajectory.points[1].velocities[0][5]);

  ASSERT_EQ(1, result.multi_dof_joint_trajectory.points[2].transforms.size());
  EXPECT_DOUBLE_EQ(23.0, result.multi_dof_joint_trajectory.points[2].transforms[0].translation().x());
  EXPECT_DOUBLE_EQ(24.0, result.multi_dof_joint_trajectory.points[2].transforms[0].translation().y());
  EXPECT_DOUBLE_EQ(0.6, result.multi_dof_joint_trajectory.points[2].transforms[0].linear().eulerAngles(0, 1, 2)[2]);
  ASSERT_EQ(1, result.multi_dof_joint_trajectory.points[2].velocities.size());
  EXPECT_DOUBLE_EQ(27.0, result.multi_dof_joint_trajectory.points[2].velocities[0][0]);
  EXPECT_DOUBLE_EQ(28.0, result.multi_dof_joint_trajectory.points[2].velocities[0][1]);
  EXPECT_DOUBLE_EQ(29.0, result.multi_dof_joint_trajectory.points[2].velocities[0][5]);
}

}  // namespace hsrb_quick_path_optimizer

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

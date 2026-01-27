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
/// @brief Transformation functions for preprocessing and postprocessing of optimization

#include "conversions.hpp"

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <angles/angles.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_robot_local_planner_utils/converter.hpp>

namespace {

constexpr uint32_t kBaseJointNum = 3;

Eigen::Vector3d Get2DPose(const Eigen::Affine3d& transform, double previous_theta) {
  auto base_pose = tmc_robot_local_planner_utils::Get2DPose(transform);
  auto diff_theta = angles::shortest_angular_distance(previous_theta, base_pose[2]);
  base_pose[2] = previous_theta + diff_theta;
  return base_pose;
}

void ConvertPositions(const Eigen::VectorXd& joint_positions,
                      const Eigen::Affine3d& base_pose,
                      double previous_theta,
                      Eigen::VectorXd& dst_positions) {
  // Joint axis + 3-axis of the cart
  dst_positions.resize(joint_positions.size() + kBaseJointNum);
  dst_positions << joint_positions, Get2DPose(base_pose, previous_theta);
}

void ConvertPositions(const Eigen::VectorXd& joint_positions,
                      const Eigen::Affine3d& base_pose,
                      Eigen::VectorXd& dst_positions) {
  ConvertPositions(joint_positions, base_pose, 0.0, dst_positions);
}

}  // namespace

namespace hsrb_quick_path_optimizer {

// Extract initial position, exclusive for HSR-B and later
void ExtractInitialPositions(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                             Eigen::VectorXd& dst_positions) {
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(0);
  if (!trajectory.joint_trajectory.points.empty()) {
    joint_positions = trajectory.joint_trajectory.points[0].positions;
  }
  ConvertPositions(
      joint_positions,
      trajectory.multi_dof_joint_trajectory.points[0].transforms[0],
      dst_positions);
}

// Extract initial velocity, exclusive for HSR-B and later
void ExtractInitialVelocities(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                              Eigen::VectorXd& dst_velocities) {
  // Joint axis + 3-axis of the cart
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(0);
  if (trajectory.joint_trajectory.points.size() > 0) {
    joint_velocities = trajectory.joint_trajectory.points[0].velocities;
  }
  dst_velocities.resize(joint_velocities.size() + kBaseJointNum);
  dst_velocities << joint_velocities,
                    tmc_robot_local_planner_utils::Get2DTwist(
                        trajectory.multi_dof_joint_trajectory.points[0].velocities[0]);
}

// Convert trajectory to type for input to optimization
void ConvertToWayPoints(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                        std::vector<Eigen::VectorXd>& dst_way_points) {
  if (trajectory.joint_trajectory.points.empty()) {
    dst_way_points.resize(trajectory.multi_dof_joint_trajectory.points.size());

    double previous_theta = 0.0;
    for (uint32_t i = 0; i < dst_way_points.size(); ++i) {
      ConvertPositions(
          Eigen::VectorXd::Zero(0),
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0],
          previous_theta, dst_way_points[i]);
      previous_theta = dst_way_points[i][dst_way_points[i].size() - 1];
    }
  } else {
    dst_way_points.resize(trajectory.joint_trajectory.points.size());

    double previous_theta = 0.0;
    for (uint32_t i = 0; i < dst_way_points.size(); ++i) {
      ConvertPositions(
          trajectory.joint_trajectory.points[i].positions,
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0],
          previous_theta, dst_way_points[i]);
      previous_theta = dst_way_points[i][dst_way_points[i].size() - 1];
    }
  }
}

// Extract optimization results
void SampleTrajectoryPoint(
    const ITrajectoryFilterAdapter::Ptr& filter,
    double time_from_start,
    tmc_manipulation_types::TimedJointTrajectoryPoint& dst_joint_point,
    tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint& dst_base_point) {
  const auto positions = filter->GetPosition(time_from_start);
  const auto velocities = filter->GetVelocity(time_from_start);

  // Assumed to be stored in the order of joint axis, 3-axis of the cart
  auto joint_num = positions.size() - kBaseJointNum;
  dst_joint_point.time_from_start = time_from_start;
  dst_joint_point.positions = positions.head(joint_num);
  dst_joint_point.velocities = velocities.head(joint_num);

  dst_base_point.time_from_start = dst_joint_point.time_from_start;
  dst_base_point.transforms.push_back(tmc_robot_local_planner_utils::GetTransform(positions.tail(kBaseJointNum)));
  dst_base_point.velocities.push_back(tmc_robot_local_planner_utils::GetTwist(velocities.tail(kBaseJointNum)));
}

// Extract optimization results at every sampling_step[sec] and convert to ROS message
void ConvertToRobotTrajectory(const ITrajectoryFilterAdapter::Ptr& filter,
                              const std::vector<std::string>& joint_names,
                              const std::string& base_joint_name,
                              double sampling_step,
                              tmc_manipulation_types::TimedRobotTrajectory& dst_trajctory) {
  dst_trajctory.joint_trajectory.joint_names = joint_names;
  dst_trajctory.multi_dof_joint_trajectory.joint_names = {base_joint_name};

  // 0 seconds is not played back, so do not sample
  const double duration = filter->GetDuration();
  double time_from_start = sampling_step;
  while (time_from_start < duration) {
    tmc_manipulation_types::TimedJointTrajectoryPoint joint_point;
    tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint base_point;
    SampleTrajectoryPoint(filter, time_from_start, joint_point, base_point);

    dst_trajctory.joint_trajectory.points.push_back(joint_point);
    dst_trajctory.multi_dof_joint_trajectory.points.push_back(base_point);

    time_from_start += sampling_step;
  }
  tmc_manipulation_types::TimedJointTrajectoryPoint joint_point;
  tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint base_point;
  SampleTrajectoryPoint(filter, duration, joint_point, base_point);

  dst_trajctory.joint_trajectory.points.push_back(joint_point);
  dst_trajctory.multi_dof_joint_trajectory.points.push_back(base_point);
}

}  // namespace hsrb_quick_path_optimizer

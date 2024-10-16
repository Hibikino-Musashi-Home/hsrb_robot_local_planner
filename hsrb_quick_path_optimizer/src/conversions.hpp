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
/// @brief Profile for optimization and conversion functions for post -processing
#ifndef HSRB_QUICK_PATH_OPTIMIZER_CONVERSIONS_HPP_
#define HSRB_QUICK_PATH_OPTIMIZER_CONVERSIONS_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_timeopt/quick_trajectory_filter.hpp>

#include "trajectory_filter_adapter.hpp"

namespace hsrb_quick_path_optimizer {

// The consistency check of the input orbit is performed separately

// Extracting the initial position, only for HSR-B
void ExtractInitialPositions(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                             Eigen::VectorXd& dst_positions);

// Extracting the initial speed, exclusively after HSR-B
void ExtractInitialVelocities(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                              Eigen::VectorXd& dst_velocities);

// Convert trajectory to molds for optimization
void ConvertToWayPoints(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                        std::vector<Eigen::VectorXd>& dst_way_points);

// Remove the optimization result for each Sampling_step [SEC] and convert it to a ROS message.
void ConvertToRobotTrajectory(const ITrajectoryFilterAdapter::Ptr& filter,
                              const std::vector<std::string>& joint_names,
                              const std::string& base_joint_name,
                              double sampling_step,
                              tmc_manipulation_types::TimedRobotTrajectory& dst_trajctory);

}  // namespace hsrb_quick_path_optimizer
#endif  // HSRB_QUICK_PATH_OPTIMIZER_CONVERSIONS_HPP_

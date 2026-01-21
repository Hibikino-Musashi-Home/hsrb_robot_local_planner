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
/// @brief Kinematics for acceleration calculation in carts from HSR-B onwards

#include "base_kinematics.hpp"

#include <cmath>

#include <algorithm>
#include <limits>
#include <tuple>
#include <vector>

namespace {
// Number of steps for exploration
// With 50, it becomes approximately 0.1[msec] on Core i5-8350U CPU 1.70GHz
constexpr int32_t kSteps = 50;

void GenerateCandidates(double limit, std::vector<std::tuple<double, double>>& dst_candidates) {
  // Since only the edge of the limit needs to be explored, prepare the exploration range in advance
  for (int32_t i = -kSteps + 1; i < kSteps; ++i) {
    const double value = limit * static_cast<double>(i) / kSteps;
    dst_candidates.push_back(std::make_tuple(limit, value));
    dst_candidates.push_back(std::make_tuple(-limit, value));
    dst_candidates.push_back(std::make_tuple(value, limit));
    dst_candidates.push_back(std::make_tuple(value, -limit));
  }
  dst_candidates.push_back(std::make_tuple(limit, limit));
  dst_candidates.push_back(std::make_tuple(limit, -limit));
  dst_candidates.push_back(std::make_tuple(-limit, limit));
  dst_candidates.push_back(std::make_tuple(-limit, -limit));
}
}  // namespace

namespace hsrb_quick_path_optimizer {

using hsrb_base_controllers::kJointIDLeftWheel;
using hsrb_base_controllers::kJointIDRightWheel;
using hsrb_base_controllers::kJointIDSteer;

// Initialize with HSR-B parameters
BaseJointLimits::BaseJointLimits() {
  this->caster_velocity = 1.8;
  this->wheel_velocity = 8.5;
  this->caster_acceleration = 1.8;
  this->wheel_acceleration = 5.0;
}

// Initialize with HSR-B parameters
OmniBaseSize::OmniBaseSize() {
  this->tread = 0.266;
  this->caster_offset = 0.11;
  this->wheel_radius = 0.04;
}

// Constructor
BaseKinematics::BaseKinematics(const BaseJointLimits& joint_limits,
                               const OmniBaseSize& omni_base_size)
    : joint_limits_(joint_limits), omni_base_size_(omni_base_size) {
  GenerateCandidates(joint_limits_.wheel_acceleration, acceleration_candidates_);
  GenerateCandidates(joint_limits_.wheel_velocity, velocity_candidates_);
}

struct JacobianImpl {
  double j11;
  double j12;
  double j21;
  double j22;

  double dj11;
  double dj12;
  double dj21;
  double dj22;

  double c1;
  double c2;

  double wheel_rate;

  JacobianImpl(const OmniBaseSize& size, double base_roll_joint)
      : JacobianImpl(size, base_roll_joint, Eigen::Vector3d::Zero()) {}

  JacobianImpl(const OmniBaseSize& size, double base_roll_joint, const Eigen::Vector3d& joint_velocities) {
    // Calculate necessary elements of the Jacobian
    // For the cart mechanism of HSR, refer to the following paper
    // https://www.jstage.jst.go.jp/article/jrsj/27/3/27_3_314/_pdf
    // The Jacobian is also summarized in doc/base_kinematics.pdf
    const double r = size.wheel_radius;
    const double s = size.caster_offset;
    const double w = size.tread;
    const double cos_v = std::cos(base_roll_joint);
    const double sin_v = std::sin(base_roll_joint);
    j11 = r * cos_v * 0.5 - r * s * sin_v / w;
    j12 = r * cos_v * 0.5 + r * s * sin_v / w;
    j21 = r * sin_v * 0.5 + r * s * cos_v / w;
    j22 = r * sin_v * 0.5 - r * s * cos_v / w;

    wheel_rate = r / w;

    dj11 = (-r * sin_v * 0.5 - r * s * cos_v / w) * joint_velocities[kJointIDSteer];
    dj12 = (-r * sin_v * 0.5 + r * s * cos_v / w) * joint_velocities[kJointIDSteer];
    dj21 = (r * cos_v * 0.5 - r * s * sin_v / w) * joint_velocities[kJointIDSteer];
    dj22 = (r * cos_v * 0.5 + r * s * sin_v / w) * joint_velocities[kJointIDSteer];
    c1 = dj11 * joint_velocities[kJointIDRightWheel] + dj12 * joint_velocities[kJointIDLeftWheel];
    c2 = dj21 * joint_velocities[kJointIDRightWheel] + dj22 * joint_velocities[kJointIDLeftWheel];
  }
};

Eigen::Vector3d BaseKinematics::CalculateBaseMaxVelocity(
    double origin_to_base_yaw, double origin_to_target_direction, double base_roll_joint) const {
  const auto jacobian = JacobianImpl(omni_base_size_, base_roll_joint);

  // Search for wheel speed closest to the target direction
  const auto direction_local = origin_to_target_direction - origin_to_base_yaw;
  double angle_distance = std::numeric_limits<double>::max();
  Eigen::Vector3d joint_velocities;
  for (const auto& point : velocity_candidates_) {
    const double ddx = jacobian.j11 * std::get<0>(point) + jacobian.j12 * std::get<1>(point);
    const double ddy = jacobian.j21 * std::get<0>(point) + jacobian.j22 * std::get<1>(point);
    const double direction = std::atan2(ddy, ddx);
    const double diff =
        std::fabs(angles::shortest_angular_distance(direction_local, direction));
    if (diff < angle_distance) {
      angle_distance = diff;
      joint_velocities[kJointIDRightWheel] = std::get<0>(point);
      joint_velocities[kJointIDLeftWheel] = std::get<1>(point);
    }
  }
  // Derivation of turning axis speed
  double yaw_velocity = joint_velocities[kJointIDRightWheel] * jacobian.wheel_rate -
                        joint_velocities[kJointIDLeftWheel] * jacobian.wheel_rate;
  // Receive the intended direction of the yaw axis as input, and implement branching accordingly
  // However, since it seems to work with just this for HSR, let's try it
  yaw_velocity =
      std::min(std::fabs(yaw_velocity - joint_limits_.caster_velocity),
               std::fabs(yaw_velocity + joint_limits_.caster_velocity));

  // Return to origin coordinates and output
  const Eigen::Matrix3d origin_to_base =
      Eigen::AngleAxisd(origin_to_base_yaw, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Vector3d base_velocities_local;
  base_velocities_local <<
      jacobian.j11 * joint_velocities[kJointIDRightWheel] + jacobian.j12 * joint_velocities[kJointIDLeftWheel],
      jacobian.j21 * joint_velocities[kJointIDRightWheel] + jacobian.j22 * joint_velocities[kJointIDLeftWheel],
      yaw_velocity;
  return (origin_to_base * base_velocities_local).cwiseAbs();
}


// Calculate the maximum acceleration when moving towards the target direction, considering the current cart state
Eigen::Vector3d BaseKinematics::CalculateBaseMaxAcceleration(
    double origin_to_base_yaw, double origin_to_target_direction,
    double base_roll_joint, const Eigen::Vector3d& joint_velocities) const {
  const auto jacobian = JacobianImpl(omni_base_size_, base_roll_joint, joint_velocities);

  // Search for wheel acceleration closest to the target direction
  const auto direction_local = origin_to_target_direction - origin_to_base_yaw;
  double angle_distance = std::numeric_limits<double>::max();
  Eigen::Vector3d joint_acceleration;
  for (const auto& point : acceleration_candidates_) {
    const double ddx = jacobian.j11 * std::get<0>(point) + jacobian.j12 * std::get<1>(point) + jacobian.c1;
    const double ddy = jacobian.j21 * std::get<0>(point) + jacobian.j22 * std::get<1>(point) + jacobian.c2;
    const double direction = std::atan2(ddy, ddx);
    const double diff =
        std::fabs(angles::shortest_angular_distance(direction_local, direction));
    if (diff < angle_distance) {
      angle_distance = diff;
      joint_acceleration[kJointIDRightWheel] = std::get<0>(point);
      joint_acceleration[kJointIDLeftWheel] = std::get<1>(point);
    }
  }
  // Derivation of turning axis acceleration
  double yaw_acceleration = joint_acceleration[kJointIDRightWheel] * jacobian.wheel_rate -
                            joint_acceleration[kJointIDLeftWheel] * jacobian.wheel_rate;
  // Receive the intended direction of the yaw axis as input, and implement branching accordingly
  // However, since it seems to work with just this for HSR, let's try it
  yaw_acceleration =
      std::min(std::fabs(yaw_acceleration - joint_limits_.caster_acceleration),
               std::fabs(yaw_acceleration + joint_limits_.caster_acceleration));

  // Return to origin coordinates and output
  const Eigen::Matrix3d origin_to_base =
      Eigen::AngleAxisd(origin_to_base_yaw, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Vector3d base_accelerations_local;
  base_accelerations_local <<
      jacobian.j11 * joint_acceleration[kJointIDRightWheel] +
          jacobian.j12 * joint_acceleration[kJointIDLeftWheel] + jacobian.c1,
      jacobian.j21 * joint_acceleration[kJointIDRightWheel] +
          jacobian.j22 * joint_acceleration[kJointIDLeftWheel] + jacobian.c2,
      yaw_acceleration;
  return (origin_to_base * base_accelerations_local).cwiseAbs();
}

}  // namespace hsrb_quick_path_optimizer

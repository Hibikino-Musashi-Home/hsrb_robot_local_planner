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
#ifndef HSRB_QUICK_PATH_OPTIMIZER_BASE_KINEMATICS_
#define HSRB_QUICK_PATH_OPTIMIZER_BASE_KINEMATICS_

#include <memory>
#include <tuple>
#include <vector>

#include <hsrb_base_controllers/twin_caster_drive.hpp>

namespace hsrb_quick_path_optimizer {

// Bogie speed / acceleration limit
struct BaseJointLimits {
  // Speed ​​limit of turning axis [M/S]
  double caster_velocity;
  // Wheel speed limit [M/S
  double wheel_velocity;
  // Acceleration limit of turning axis [M/S^2]
  double caster_acceleration;
  // Wheel acceleration limit [M/S^2]
  double wheel_acceleration;

  // Initialize with HSR-B parameters
  BaseJointLimits();
};

// Dimensions of bogie
struct OmniBaseSize : public hsrb_base_controllers::OmniBaseSize {
  // Initialize with HSR-B parameters
  OmniBaseSize();
};

// Bog motor for acceleration calculation
class BaseKinematics {
 public:
  using Ptr = std::shared_ptr<BaseKinematics>;

  // constructor
  BaseKinematics(const BaseJointLimits& joint_limits,
                 const OmniBaseSize& omni_base_size);
  virtual ~BaseKinematics() = default;

  // Calculate the maximum speed when going in the target direction considering the current bogie state
  // @param[in] ORIGIN_TO_BASE_YAW ORIGIN's reference yaw component [RAD]
  // @param[in] ORIGIN_TO_TARGET_DIRECTION ORIGIN Speed ​​direction [RAD]
  // @param[in] Base_roll_Joint Table Rotary Alarm [RAD]
  // @return Eigen :: Vector3d Origin Maximum speed x, y, yaw
  Eigen::Vector3d CalculateBaseMaxVelocity(
      double origin_to_base_yaw, double origin_to_target_direction, double base_roll_joint) const;

  // Calculate the maximum acceleration when going in the target direction considering the current bogie state
  // @param[in] ORIGIN_TO_BASE_YAW ORIGIN's reference yaw component [RAD]
  // @param[in] ORIGIN_TO_TARGET_DIRECTION ORIGIN Against acceleration direction [RAD]
  // @param[in] Base_roll_Joint Table Rotary Alarm [RAD]
  // @param[in] Joint_velocities speed of right wheels, left wheels, turning axis speed [RAD/S
  // @return Eigen :: Vector3d Origin Maximum acceleration x, y, yaw
  Eigen::Vector3d CalculateBaseMaxAcceleration(
      double origin_to_base_yaw, double origin_to_target_direction,
      double base_roll_joint, const Eigen::Vector3d& joint_velocities) const;

 private:
  BaseJointLimits joint_limits_;
  OmniBaseSize omni_base_size_;

  using Point = std::tuple<double, double>;
  std::vector<Point> acceleration_candidates_;
  std::vector<Point> velocity_candidates_;
};

}  // namespace hsrb_quick_path_optimizer

#endif  // HSRB_QUICK_PATH_OPTIMIZER_BASE_KINEMATICS_

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

#include "base_kinematics_ros.hpp"

#include <string>

#include <tmc_utils/parameters.hpp>

namespace hsrb_quick_path_optimizer {

BaseJointLimitsRos::BaseJointLimitsRos(const rclcpp::Node::SharedPtr& node)
    : BaseJointLimits() {
  this->caster_velocity = tmc_utils::GetParameter(node, "max_caster_velocity", this->caster_velocity);
  this->wheel_velocity = tmc_utils::GetParameter(node, "max_wheel_velocity", this->wheel_velocity);
  this->caster_acceleration = tmc_utils::GetParameter(node, "max_caster_acceleration", this->caster_acceleration);
  this->wheel_acceleration = tmc_utils::GetParameter(node, "max_wheel_acceleration", this->wheel_acceleration);
}

OmniBaseSizeRos::OmniBaseSizeRos(const rclcpp::Node::SharedPtr& node)
    : OmniBaseSize() {
  this->tread = tmc_utils::GetParameter(node, "tread", this->tread);
  this->caster_offset = tmc_utils::GetParameter(node, "caster_offset", this->caster_offset);
  this->wheel_radius = tmc_utils::GetParameter(node, "wheel_radius", this->wheel_radius);
}

}  // namespace hsrb_quick_path_optimizer

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
#ifndef HSRB_QUICK_PATH_OPTIMIZER_TRAJECTORY_FILTER_ADAPTER_HPP_
#define HSRB_QUICK_PATH_OPTIMIZER_TRAJECTORY_FILTER_ADAPTER_HPP_

#include <memory>
#include <tmc_timeopt/quick_trajectory_filter.hpp>

namespace hsrb_quick_path_optimizer {

class ITrajectoryFilterAdapter {
 public:
  using Ptr = std::shared_ptr<ITrajectoryFilterAdapter>;

  virtual ~ITrajectoryFilterAdapter() = default;

  // Get the joint position in Time_from_start
  virtual Eigen::VectorXd GetPosition(double time_from_start) const = 0;

  // Time_from_start gets joint speed
  virtual Eigen::VectorXd GetVelocity(double time_from_start) const = 0;

  // Get the track playback time
  virtual double GetDuration() const = 0;
};

class TrajectoryFilterAdapter : public ITrajectoryFilterAdapter {
 public:
  explicit TrajectoryFilterAdapter(const std::shared_ptr<tmc_timeopt::ITrajectoryFilter>& filter_impl)
      : filter_impl_(filter_impl) {}

  // Get the joint position in Time_from_start
  Eigen::VectorXd GetPosition(double time_from_start) const {
    return filter_impl_->GetPosition(time_from_start);
  }

  // Time_from_start gets joint speed
  Eigen::VectorXd GetVelocity(double time_from_start) const {
    return filter_impl_->GetVelocity(time_from_start);
  }

  // Get the track playback time
  double GetDuration() const {
    return filter_impl_->GetDuration();
  }

 private:
  std::shared_ptr<tmc_timeopt::ITrajectoryFilter> filter_impl_;
};

}  // namespace hsrb_quick_path_optimizer
#endif  // HSRB_QUICK_PATH_OPTIMIZER_TRAJECTORY_FILTER_ADAPTER_HPP_

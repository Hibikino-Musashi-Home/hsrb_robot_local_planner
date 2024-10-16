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
/// @file     arrival_percentage_publisher.hpp
/// @brief    publish for arrival percentage
/// @author   Satoru Onoda
#ifndef HSRB_ROBOT_LOCAL_PLANNER_NODE_ARRIVAL_PERCENTAGE_PUBLISHER_HPP
#define HSRB_ROBOT_LOCAL_PLANNER_NODE_ARRIVAL_PERCENTAGE_PUBLISHER_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace hsrb_robot_local_planner_node {
class ArrivalPercentagePublisher {
 public:
  using Ptr = std::shared_ptr<ArrivalPercentagePublisher>;

  explicit ArrivalPercentagePublisher(rclcpp::Node::SharedPtr node) {
    arrival_percentage_pub_ = node->create_publisher<std_msgs::msg::Float64>(
        "~/arrival_percentage", rclcpp::SystemDefaultsQoS());
  }
  ~ArrivalPercentagePublisher() = default;

  void Publish(double arrival_rate) {
    std_msgs::msg::Float64 arrival_rate_msg;
    arrival_rate_msg.data = arrival_rate;
    arrival_percentage_pub_->publish(arrival_rate_msg);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arrival_percentage_pub_;
};
}  // namespace hsrb_robot_local_planner_node

#endif  // HSRB_ROBOT_LOCAL_PLANNER_NODE_ARRIVAL_PERCENTAGE_PUBLISHER_HPP

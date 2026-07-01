// Copyright 2026 cmc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <flexiv/amr/vendor/seer/seer_amr.h>

#include <chrono>
#include <flexiv_amr_msgs/msg/amr_local_motion.hpp>
#include <flexiv_amr_msgs/srv/amr_emergency_stop.hpp>
#include <flexiv_amr_msgs/srv/amr_stop_open_loop_motion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace flexiv_amr::driver {

class FlexivAmrDriverNode : public rclcpp::Node
{
public:
    FlexivAmrDriverNode();
    ~FlexivAmrDriverNode() override;

private:
    static constexpr int kDefaultDurationMs = 500;
    static constexpr int kMaximumDurationMs = 5000;
    static constexpr auto kMinimumCommandInterval = std::chrono::milliseconds(100);

    bool ConnectIfNeeded();
    void DisconnectAfterFailure();
    void OnMotion(const flexiv_amr_msgs::msg::AmrLocalMotion::SharedPtr message);
    void OnStopMotion(const flexiv_amr_msgs::srv::AmrStopOpenLoopMotion::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrStopOpenLoopMotion::Response::SharedPtr response);
    void OnEmergencyStop(const flexiv_amr_msgs::srv::AmrEmergencyStop::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrEmergencyStop::Response::SharedPtr response);

    std::string amr_ip_;
    int duration_;
    std::chrono::steady_clock::time_point last_command_time_;
    bool connected_ {false};
    bool motion_command_sent_ {false};
    std::unique_ptr<flexiv::amr::seer::SeerAmrClient> amr_;
    rclcpp::Subscription<flexiv_amr_msgs::msg::AmrLocalMotion>::SharedPtr motion_subscription_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrStopOpenLoopMotion>::SharedPtr stop_motion_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrEmergencyStop>::SharedPtr emergency_stop_service_;
};

} // namespace flexiv_amr::driver

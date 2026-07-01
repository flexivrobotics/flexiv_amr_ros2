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

#include "flexiv_amr_driver/flexiv_amr_driver_node.hpp"

#include <cmath>
#include <functional>
#include <rclcpp/qos.hpp>
#include <stdexcept>

namespace flexiv_amr::driver {

FlexivAmrDriverNode::FlexivAmrDriverNode()
: Node("flexiv_amr_driver")
, amr_ip_(declare_parameter<std::string>("amr_ip", ""))
, duration_(declare_parameter<int>("duration", kDefaultDurationMs))
, last_command_time_(std::chrono::steady_clock::time_point::min())
{
    const auto topic = declare_parameter<std::string>("topic", "/flexiv/amr/control/motion");
    const auto stop_motion_service_name
        = declare_parameter<std::string>("stop_motion_service", "/flexiv/amr/control/stop_motion");
    const auto emergency_stop_service_name = declare_parameter<std::string>(
        "emergency_stop_service", "/flexiv/amr/control/emergency_stop");
    const auto connect_timeout_ms = declare_parameter<int>("connect_timeout_ms", 3000);
    const auto send_timeout_ms = declare_parameter<int>("send_timeout_ms", 3000);
    const auto recv_timeout_ms = declare_parameter<int>("recv_timeout_ms", 3000);

    if (amr_ip_.empty()) {
        throw std::invalid_argument("The 'amr_ip' parameter must not be empty");
    }
    if (topic.empty()) {
        throw std::invalid_argument("The 'topic' parameter must not be empty");
    }
    if (stop_motion_service_name.empty()) {
        throw std::invalid_argument("The 'stop_motion_service' parameter must not be empty");
    }
    if (emergency_stop_service_name.empty()) {
        throw std::invalid_argument("The 'emergency_stop_service' parameter must not be empty");
    }
    if (duration_ < 0 || duration_ > kMaximumDurationMs) {
        throw std::invalid_argument("The 'duration' parameter must be between 0 and 5000 ms");
    }
    if (connect_timeout_ms <= 0 || send_timeout_ms <= 0 || recv_timeout_ms <= 0) {
        throw std::invalid_argument("SEER timeout parameters must be greater than zero");
    }

    flexiv::amr::seer::SeerAmrOptions options;
    options.host = amr_ip_;
    options.auto_connect = false;
    options.auto_reconnect = true;
    options.time_out.connect_timeout_ms = connect_timeout_ms;
    options.time_out.send_timeout_ms = send_timeout_ms;
    options.time_out.recv_timeout_ms = recv_timeout_ms;
    amr_ = std::make_unique<flexiv::amr::seer::SeerAmrClient>(options);

    motion_subscription_
        = create_subscription<flexiv_amr_msgs::msg::AmrLocalMotion>(topic, rclcpp::SensorDataQoS(),
            std::bind(&FlexivAmrDriverNode::OnMotion, this, std::placeholders::_1));
    stop_motion_service_
        = create_service<flexiv_amr_msgs::srv::AmrStopOpenLoopMotion>(stop_motion_service_name,
            [this](const flexiv_amr_msgs::srv::AmrStopOpenLoopMotion::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrStopOpenLoopMotion::Response::SharedPtr response) {
                OnStopMotion(request, response);
            });
    emergency_stop_service_
        = create_service<flexiv_amr_msgs::srv::AmrEmergencyStop>(emergency_stop_service_name,
            [this](const flexiv_amr_msgs::srv::AmrEmergencyStop::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrEmergencyStop::Response::SharedPtr response) {
                OnEmergencyStop(request, response);
            });

    RCLCPP_WARN(get_logger(),
        "Open-loop motion commands cancel any active navigation task; duration is %ld ms",
        static_cast<long>(duration_));
    if (duration_ == 0) {
        RCLCPP_WARN(get_logger(),
            "A zero duration keeps the AMR moving until an explicit stop command is sent");
    }
}

FlexivAmrDriverNode::~FlexivAmrDriverNode()
{
    if (!amr_) {
        return;
    }

    if (connected_ && motion_command_sent_) {
        try {
            if (!amr_->StopOpenLoopMotion()) {
                RCLCPP_ERROR(get_logger(), "SEER AMR rejected the cleanup stop command");
            }
        } catch (const std::exception& error) {
            RCLCPP_ERROR(
                get_logger(), "Failed to stop open-loop motion during cleanup: %s", error.what());
        }
    }

    if (connected_) {
        try {
            amr_->Disconnect();
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "Failed to disconnect cleanly: %s", error.what());
        }
    }
}

bool FlexivAmrDriverNode::ConnectIfNeeded()
{
    if (connected_) {
        return true;
    }

    try {
        RCLCPP_INFO(get_logger(), "Connecting to SEER AMR at %s", amr_ip_.c_str());
        amr_->Connect();
        connected_ = true;
        RCLCPP_INFO(get_logger(), "Connected to SEER AMR");
        return true;
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to SEER AMR: %s", error.what());
        return false;
    }
}

void FlexivAmrDriverNode::DisconnectAfterFailure()
{
    connected_ = false;
    try {
        amr_->Disconnect();
    } catch (const std::exception&) {
        // Preserve the original command failure in the log.
    }
}

void FlexivAmrDriverNode::OnMotion(const flexiv_amr_msgs::msg::AmrLocalMotion::SharedPtr message)
{
    if (!std::isfinite(message->vx) || !std::isfinite(message->vy) || !std::isfinite(message->w)) {
        RCLCPP_ERROR(get_logger(), "Ignoring motion command with a non-finite velocity");
        return;
    }
    if (!ConnectIfNeeded()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_command_time_ != std::chrono::steady_clock::time_point::min()
        && now - last_command_time_ < kMinimumCommandInterval) {
        RCLCPP_DEBUG(get_logger(), "Ignoring motion command received faster than 10 Hz");
        return;
    }
    last_command_time_ = now;

    flexiv::amr::seer::SeerOpenLoopMotionCommand command;
    command.vx = message->vx;
    command.vy = message->vy;
    command.w = message->w;
    command.duration = duration_;

    try {
        if (!amr_->ExecuteOpenLoopMotion(command)) {
            RCLCPP_ERROR(get_logger(), "SEER AMR rejected the open-loop motion command");
            return;
        }
        motion_command_sent_ = true;
        RCLCPP_DEBUG(get_logger(),
            "Sent open-loop motion: vx=%.3f m/s, vy=%.3f m/s, w=%.3f rad/s, duration=%ld ms",
            message->vx, message->vy, message->w, static_cast<long>(duration_));
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Open-loop motion command failed: %s", error.what());
        DisconnectAfterFailure();
    }
}

void FlexivAmrDriverNode::OnStopMotion(
    const flexiv_amr_msgs::srv::AmrStopOpenLoopMotion::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrStopOpenLoopMotion::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->StopOpenLoopMotion();
        response->message = response->success
                                ? "Stop open-loop motion command accepted"
                                : "SEER AMR rejected the stop open-loop motion command";
        if (response->success) {
            motion_command_sent_ = false;
        }
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrDriverNode::OnEmergencyStop(
    const flexiv_amr_msgs::srv::AmrEmergencyStop::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrEmergencyStop::Response::SharedPtr response)
{
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->SetSoftEmergencyStop(request->status);
        response->message = response->success
                                ? (request->status ? "Software emergency stop activated"
                                                   : "Software emergency stop released")
                                : "SEER AMR rejected the software emergency-stop command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

} // namespace flexiv_amr::driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<flexiv_amr::driver::FlexivAmrDriverNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("flexiv_amr_driver"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

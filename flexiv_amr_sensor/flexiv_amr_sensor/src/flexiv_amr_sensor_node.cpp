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

#include "flexiv_amr_sensor/flexiv_amr_sensor_node.hpp"

#include <flexiv/amr/exceptions.h>
#include <stdexcept>


namespace {

bool RequiresReconnect(flexiv::amr::AmrErrorCode code)
{
    using flexiv::amr::AmrErrorCode;
    switch (code) {
        case AmrErrorCode::kConnectTimeout:
        case AmrErrorCode::kSendTimeout:
        case AmrErrorCode::kRecvTimeout:
        case AmrErrorCode::kResolveFailed:
        case AmrErrorCode::kConnectFailed:
        case AmrErrorCode::kSendFailed:
        case AmrErrorCode::kRecvFailed:
        case AmrErrorCode::kDisconnected:
        case AmrErrorCode::kInvalidHeader:
        case AmrErrorCode::kInvalidFrame:
            return true;
        default:
            return false;
    }
}

} // namespace

namespace flexiv_amr::sensor {

FlexivAmrSensorNode::FlexivAmrSensorNode()
: Node("flexiv_amr_sensor")
, amr_ip_(declare_parameter<std::string>("amr_ip", ""))
, reconnect_interval_(declare_parameter<double>("reconnect_interval_sec", 2.0))
, next_connect_attempt_(std::chrono::steady_clock::time_point::min())
{
    const auto connect_timeout_ms = declare_parameter<int>("connect_timeout_ms", 3000);
    const auto send_timeout_ms = declare_parameter<int>("send_timeout_ms", 3000);
    const auto recv_timeout_ms = declare_parameter<int>("recv_timeout_ms", 3000);

    if (amr_ip_.empty()) {
        throw std::invalid_argument("The 'amr_ip' parameter must not be empty");
    }
    if (reconnect_interval_.count() <= 0.0) {
        throw std::invalid_argument(
            "The 'reconnect_interval_sec' parameter must be greater than zero");
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

    AddSensor(std::make_shared<Lidar>(*this));
    AddSensor(std::make_shared<Imu>(*this));
    AddSensor(std::make_shared<Encoder>(*this));
    AddSensor(std::make_shared<Ultrasonic>(*this));
    AddSensor(std::make_shared<SensorInfo>(*this));
    AddSensor(std::make_shared<Speed>(*this));
    AddSensor(std::make_shared<Battery>(*this));
    AddSensor(std::make_shared<Pose>(*this));
    AddSensor(std::make_shared<Info>(*this));
    AddSensor(std::make_shared<RunInfo>(*this));
    AddSensor(std::make_shared<BlockStatus>(*this));

    RCLCPP_INFO(get_logger(), "Started AMR sensor node with %zu enabled sensor publisher(s)",
        timers_.size());
}

FlexivAmrSensorNode::~FlexivAmrSensorNode()
{
    try {
        if (connected_) {
            amr_->Disconnect();
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN(get_logger(), "Failed to disconnect cleanly: %s", error.what());
    }
}

bool FlexivAmrSensorNode::ConnectIfNeeded()
{
    if (connected_) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_connect_attempt_) {
        return false;
    }

    try {
        RCLCPP_INFO(get_logger(), "Connecting to SEER AMR at %s", amr_ip_.c_str());
        amr_->Connect();
        connected_ = true;
        RCLCPP_INFO(get_logger(), "Connected to SEER AMR");
        return true;
    } catch (const std::exception& error) {
        next_connect_attempt_ = now
                                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    reconnect_interval_);
        RCLCPP_ERROR(get_logger(), "Failed to connect to SEER AMR: %s", error.what());
        return false;
    }
}

void FlexivAmrSensorNode::PollSensor(const std::function<void()>& callback)
{
    if (!ConnectIfNeeded()) {
        return;
    }

    try {
        callback();
    } catch (const flexiv::amr::AmrException& error) {
        if (!RequiresReconnect(error.code())) {
            RCLCPP_WARN(get_logger(), "Sensor polling skipped: %s", error.what());
            return;
        }
        connected_ = false;
        next_connect_attempt_ = std::chrono::steady_clock::now()
                                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    reconnect_interval_);
        try {
            amr_->Disconnect();
        } catch (const std::exception&) {
        }
        RCLCPP_ERROR(get_logger(), "Sensor polling failed: %s", error.what());
    } catch (const std::exception& error) {
        connected_ = false;
        next_connect_attempt_ = std::chrono::steady_clock::now()
                                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    reconnect_interval_);
        try {
            amr_->Disconnect();
        } catch (const std::exception&) {
        }
        RCLCPP_ERROR(get_logger(), "Sensor polling failed: %s", error.what());
    } catch (...) {
        connected_ = false;
        next_connect_attempt_ = std::chrono::steady_clock::now()
                                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    reconnect_interval_);
        try {
            amr_->Disconnect();
        } catch (...) {
        }
        RCLCPP_ERROR(get_logger(), "Sensor polling failed with an unknown exception");
    }
}

} // namespace flexiv_amr::sensor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<flexiv_amr::sensor::FlexivAmrSensorNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("flexiv_amr_sensor"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

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

#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class SensorInfo
{
public:
    explicit SensorInfo(rclcpp::Node& node)
    : enable_(node.declare_parameter<bool>("amr_sensor_info.enable", true))
    , topic_(
          node.declare_parameter<std::string>("amr_sensor_info.topic", "/flexiv/amr/sensor/info"))
    , publish_rate_(node.declare_parameter<double>("amr_sensor_info.publish_rate", 1.0))
    , qos_depth_(node.declare_parameter<int>("amr_sensor_info.qos_depth", 5))
    , depth_cameras_(node.declare_parameter<std::vector<std::string>>(
          "amr_sensor_info.depth_cameras", std::vector<std::string> {"camera"}))
    {
        if (topic_.empty()) {
            throw std::invalid_argument("amr_sensor_info.topic must not be empty");
        }
        if (publish_rate_ <= 0.0) {
            throw std::invalid_argument("amr_sensor_info.publish_rate must be greater than zero");
        }
        if (qos_depth_ <= 0) {
            throw std::invalid_argument("amr_sensor_info.qos_depth must be greater than zero");
        }
        if (depth_cameras_.empty()) {
            throw std::invalid_argument(
                "amr_sensor_info.depth_cameras must contain at least one camera");
        }
        if (enable_) {
            publisher_
                = node.create_publisher<std_msgs::msg::String>(topic_, SensorQos(qos_depth_));
        }
    }

    bool enabled() const { return enable_; }
    double publish_rate() const { return publish_rate_; }

    void Publish(rclcpp::Node&, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetSensorInfo(depth_cameras_);
        std_msgs::msg::String message;
        message.data = data.data.dump();
        publisher_->publish(message);
    }

private:
    bool enable_;
    std::string topic_;
    double publish_rate_;
    int qos_depth_;
    std::vector<std::string> depth_cameras_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

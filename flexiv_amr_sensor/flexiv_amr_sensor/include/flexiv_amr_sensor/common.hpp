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

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

namespace flexiv_amr::sensor {

struct SensorConfig
{
    bool enable {true};
    std::string topic;
    std::string frame_id;
    double publish_rate {1.0};
    int qos_depth {5};
};

inline SensorConfig DeclareSensorConfig(rclcpp::Node& node, const std::string& prefix,
    const std::string& default_topic, const std::string& default_frame_id,
    const double default_publish_rate)
{
    SensorConfig config;
    config.enable = node.declare_parameter<bool>(prefix + ".enable", true);
    config.topic = node.declare_parameter<std::string>(prefix + ".topic", default_topic);
    config.frame_id = node.declare_parameter<std::string>(prefix + ".frame_id", default_frame_id);
    config.publish_rate
        = node.declare_parameter<double>(prefix + ".publish_rate", default_publish_rate);
    config.qos_depth = node.declare_parameter<int>(prefix + ".qos_depth", 5);

    if (config.topic.empty()) {
        throw std::invalid_argument(prefix + ".topic must not be empty");
    }
    if (config.publish_rate <= 0.0) {
        throw std::invalid_argument(prefix + ".publish_rate must be greater than zero");
    }
    if (config.qos_depth <= 0) {
        throw std::invalid_argument(prefix + ".qos_depth must be greater than zero");
    }
    return config;
}

inline rclcpp::QoS SensorQos(const int depth)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
}

} // namespace flexiv_amr::sensor

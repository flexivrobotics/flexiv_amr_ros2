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

#include <flexiv_amr_msgs/msg/amr_ultrasonic.hpp>
#include <flexiv_amr_msgs/msg/amr_ultrasonic_node.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Ultrasonic
{
public:
    explicit Ultrasonic(rclcpp::Node& node)
    : config_(DeclareSensorConfig(
        node, "amr_ultrasonic", "/flexiv/amr/sensor/ultrasonic/raw", "base_link", 5.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrUltrasonic>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetUltrasonicData();
        flexiv_amr_msgs::msg::AmrUltrasonic message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.nodes.reserve(data.ultrasonic_nodes.size());
        for (const auto& source : data.ultrasonic_nodes) {
            flexiv_amr_msgs::msg::AmrUltrasonicNode ultrasonic_node;
            ultrasonic_node.id = source.id;
            ultrasonic_node.distance_m = source.dist;
            ultrasonic_node.valid = source.valid;
            message.nodes.push_back(ultrasonic_node);
        }
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrUltrasonic>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

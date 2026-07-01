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

#include <flexiv_amr_msgs/msg/amr_encoder.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Encoder
{
public:
    explicit Encoder(rclcpp::Node& node)
    : config_(
        DeclareSensorConfig(node, "amr_encoder", "/flexiv/amr/sensor/encoder/raw", "base_link", 5.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrEncoder>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetEncoderData();
        flexiv_amr_msgs::msg::AmrEncoder message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.motor_encoder_json.reserve(data.motor_encoders.size());
        for (const auto& encoder : data.motor_encoders) {
            message.motor_encoder_json.push_back(encoder.dump());
        }
        message.legacy_encoder_pulses = data.legacy_encoder_pulses;
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrEncoder>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

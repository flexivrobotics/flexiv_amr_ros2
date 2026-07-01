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

#include <flexiv_amr_msgs/msg/amr_speed.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Speed
{
public:
    explicit Speed(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_speed", "/flexiv/amr/speed", "AMR", 10.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrSpeed>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetAmrSpeed();
        flexiv_amr_msgs::msg::AmrSpeed message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.actual_velocity.x = data.vx;
        message.actual_velocity.y = data.vy;
        message.actual_velocity.z = data.w;
        message.received_velocity.x = data.r_vx;
        message.received_velocity.y = data.r_vy;
        message.received_velocity.z = data.r_w;
        message.steer = data.steer;
        message.spin = data.spin;
        message.received_steer = data.r_steer;
        message.received_spin = data.r_spin;
        message.steer_angles = data.steer_angles;
        message.received_steer_angles = data.r_steer_angles;
        message.is_stop = data.is_stop;
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrSpeed>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

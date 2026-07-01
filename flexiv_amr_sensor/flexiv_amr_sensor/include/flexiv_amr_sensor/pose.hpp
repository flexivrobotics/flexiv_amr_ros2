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

#include <flexiv_amr_msgs/msg/amr_pose.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Pose
{
public:
    explicit Pose(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_pose", "/flexiv/amr/pose", "base_link", 5.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrPose>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetAmrPose();
        flexiv_amr_msgs::msg::AmrPose message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.x = data.x;
        message.y = data.y;
        message.angle = data.angle;
        message.confidence = data.confidence;
        message.localization_method = data.localization_method;
        message.current_station = data.current_station;
        message.last_station = data.last_station;
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrPose>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

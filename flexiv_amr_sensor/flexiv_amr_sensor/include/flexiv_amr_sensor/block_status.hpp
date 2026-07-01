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

#include <flexiv_amr_msgs/msg/amr_block_status.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class BlockStatus
{
public:
    explicit BlockStatus(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_block_status", "/flexiv/amr/block/status", "base_link", 1.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrBlockStatus>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetBlockStatus();
        flexiv_amr_msgs::msg::AmrBlockStatus message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.blocked = data.blocked;
        message.block_reason = data.block_reason;
        message.block_x = data.block_x;
        message.block_y = data.block_y;
        message.block_id = data.block_id;
        message.slowed = data.slowed;
        message.slow_reason = data.slow_reason;
        message.slow_x = data.slow_x;
        message.slow_y = data.slow_y;
        message.slow_id = data.slow_id;
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrBlockStatus>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

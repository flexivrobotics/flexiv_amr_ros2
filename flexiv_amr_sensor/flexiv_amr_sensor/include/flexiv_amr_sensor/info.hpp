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

#include <flexiv_amr_msgs/msg/amr_info.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Info
{
public:
    explicit Info(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_info", "/flexiv/amr/info", "AMR", 1.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrInfo>(
                config_.topic, rclcpp::QoS(rclcpp::KeepLast(config_.qos_depth)).transient_local());
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetAmrInfo();
        flexiv_amr_msgs::msg::AmrInfo message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.amr_id = data.amr_id;
        message.amr_name = data.amr_name;
        message.amr_note = data.amr_note;
        message.robokit_version = data.robokit_version;
        message.amr_model_name = data.amr_model_name;
        message.dsp_version = data.dsp_version;
        message.gyro_version = data.gyro_version;
        message.map_version = data.map_version;
        message.model_version = data.model_version;
        message.netprotocol_version = data.netprotocol_version;
        message.modbus_version = data.modbus_version;
        message.current_map = data.current_map;
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrInfo>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

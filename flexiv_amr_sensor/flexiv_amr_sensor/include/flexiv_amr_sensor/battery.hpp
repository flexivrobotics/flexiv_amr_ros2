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

#include <flexiv_amr_msgs/msg/amr_battery.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Battery
{
public:
    explicit Battery(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_battery", "/flexiv/amr/battery/status", "base_link", 1.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrBattery>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetBatteryStatus();
        flexiv_amr_msgs::msg::AmrBattery message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.battery_level = data.battery_level;
        message.battery_temp = data.battery_temp;
        message.voltage = data.voltage;
        message.current = data.current;
        message.max_charge_voltage = data.max_charge_voltage;
        message.max_charge_current = data.max_charge_current;
        message.charging = data.charging;
        message.manual_charge = data.manual_charge;
        message.auto_charge = data.auto_charge;
        message.battery_cycle = data.battery_cycle;
        message.battery_user_data = data.battery_user_data;
        message.extra = data.extra;
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrBattery>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

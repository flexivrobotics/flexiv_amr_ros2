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

#include <flexiv_amr_msgs/msg/amr_imu.hpp>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Imu
{
public:
    explicit Imu(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_imu", "/flexiv/amr/sensor/imu/raw", "imu_link", 10.0))
    {
        if (config_.enable) {
            publisher_ = node.create_publisher<flexiv_amr_msgs::msg::AmrImu>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto data = amr.GetImuData();
        flexiv_amr_msgs::msg::AmrImu message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        message.euler_rad.x = data.roll;
        message.euler_rad.y = data.pitch;
        message.euler_rad.z = data.yaw;
        message.acceleration_adc.x = data.acc_x;
        message.acceleration_adc.y = data.acc_y;
        message.acceleration_adc.z = data.acc_z;
        message.angular_velocity_adc.x = data.rot_x;
        message.angular_velocity_adc.y = data.rot_y;
        message.angular_velocity_adc.z = data.rot_z;
        message.angular_velocity_offset_adc.x = static_cast<double>(data.rot_off_x);
        message.angular_velocity_offset_adc.y = static_cast<double>(data.rot_off_y);
        message.angular_velocity_offset_adc.z = static_cast<double>(data.rot_off_z);
        publisher_->publish(message);
    }

private:
    SensorConfig config_;
    rclcpp::Publisher<flexiv_amr_msgs::msg::AmrImu>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

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

#include <flexiv_amr_msgs/msg/amr_imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <string>
#include <vector>

namespace flexiv_amr::imu {

class FlexivAmrImuNode : public rclcpp::Node
{
public:
    FlexivAmrImuNode();

private:
    void OnRawImu(const flexiv_amr_msgs::msg::AmrImu::SharedPtr message);
    void ValidateParameters() const;
    void ConfigureRawToImuRotation(const std::vector<double>& rpy);
    geometry_msgs::msg::Vector3 TransformVector(double x, double y, double z) const;

    std::string raw_imu_topic_;
    std::string imu_topic_;
    bool subtract_angular_velocity_offset_ {true};

    std::array<double, 4> raw_to_imu_quaternion_ {0.0, 0.0, 0.0, 1.0};
    std::array<double, 9> raw_to_imu_rotation_ {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::array<double, 9> orientation_covariance_ {};
    std::array<double, 9> angular_velocity_covariance_ {};
    std::array<double, 9> linear_acceleration_covariance_ {};

    rclcpp::Subscription<flexiv_amr_msgs::msg::AmrImu>::SharedPtr raw_imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

} // namespace flexiv_amr::imu

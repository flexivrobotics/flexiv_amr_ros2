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

#include <flexiv_amr_msgs/msg/amr_encoder.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace flexiv_amr::wheel_odom {

class FlexivAmrWheelOdom : public rclcpp::Node
{
public:
    FlexivAmrWheelOdom();

private:
    void ValidateParameters(const std::vector<double>& pose_covariance,
        const std::vector<double>& twist_covariance) const;
    void EncoderCallback(const flexiv_amr_msgs::msg::AmrEncoder::SharedPtr message);
    static double NormalizeAngle(double angle);
    geometry_msgs::msg::Quaternion CreateYawQuaternion() const;
    void PublishOdometry(
        const rclcpp::Time& stamp, double linear_velocity, double angular_velocity);

    std::string input_topic_;
    std::string odom_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    int left_encoder_index_ {0};
    int right_encoder_index_ {1};
    double left_pulses_per_meter_ {1.0};
    double right_pulses_per_meter_ {1.0};
    double wheel_separation_ {0.5716};
    double left_direction_ {1.0};
    double right_direction_ {1.0};

    bool initialized_ {false};
    std::int64_t previous_left_pulses_ {0};
    std::int64_t previous_right_pulses_ {0};
    rclcpp::Time previous_stamp_ {0, 0, RCL_ROS_TIME};
    double x_ {0.0};
    double y_ {0.0};
    double yaw_ {0.0};
    std::array<double, 36> pose_covariance_ {};
    std::array<double, 36> twist_covariance_ {};

    rclcpp::Subscription<flexiv_amr_msgs::msg::AmrEncoder>::SharedPtr encoder_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

} // namespace flexiv_amr::wheel_odom

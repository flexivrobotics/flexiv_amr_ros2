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

#include "flexiv_amr_wheel_odom/flexiv_amr_wheel_odom.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>

namespace flexiv_amr::wheel_odom {

FlexivAmrWheelOdom::FlexivAmrWheelOdom()
: Node("flexiv_amr_wheel_odom")
{
    input_topic_ = declare_parameter<std::string>("input_topic", "/flexiv/amr/sensor/encoder/raw");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel_odom");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    left_encoder_index_ = declare_parameter<int>("left_encoder_index", 0);
    right_encoder_index_ = declare_parameter<int>("right_encoder_index", 1);
    left_pulses_per_meter_ = declare_parameter<double>("left_pulses_per_meter", 1.0);
    right_pulses_per_meter_ = declare_parameter<double>("right_pulses_per_meter", 1.0);
    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.5716);
    left_direction_ = declare_parameter<double>("left_direction", 1.0);
    right_direction_ = declare_parameter<double>("right_direction", 1.0);

    const auto pose_covariance = declare_parameter<std::vector<double>>(
        "pose_covariance", {0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03});
    const auto twist_covariance = declare_parameter<std::vector<double>>(
        "twist_covariance", {0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03});

    ValidateParameters(pose_covariance, twist_covariance);
    std::copy(pose_covariance.begin(), pose_covariance.end(), pose_covariance_.begin());
    std::copy(twist_covariance.begin(), twist_covariance.end(), twist_covariance_.begin());

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    encoder_subscription_ = create_subscription<flexiv_amr_msgs::msg::AmrEncoder>(input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&FlexivAmrWheelOdom::EncoderCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Started wheel odometry: %s -> %s, left_index=%d, right_index=%d",
        input_topic_.c_str(), odom_topic_.c_str(), left_encoder_index_, right_encoder_index_);
}

void FlexivAmrWheelOdom::ValidateParameters(
    const std::vector<double>& pose_covariance, const std::vector<double>& twist_covariance) const
{
    if (input_topic_.empty() || odom_topic_.empty()) {
        throw std::invalid_argument("input_topic and odom_topic must not be empty");
    }
    if (odom_frame_id_.empty() || base_frame_id_.empty()) {
        throw std::invalid_argument("odom_frame_id and base_frame_id must not be empty");
    }
    if (left_encoder_index_ < 0 || right_encoder_index_ < 0) {
        throw std::invalid_argument("encoder indexes must be non-negative");
    }
    if (left_pulses_per_meter_ == 0.0 || right_pulses_per_meter_ == 0.0) {
        throw std::invalid_argument("pulses_per_meter parameters must not be zero");
    }
    if (wheel_separation_ <= 0.0) {
        throw std::invalid_argument("wheel_separation must be greater than zero");
    }
    if (pose_covariance.size() != 36 || twist_covariance.size() != 36) {
        throw std::invalid_argument("pose_covariance and twist_covariance must contain 36 values");
    }
}

void FlexivAmrWheelOdom::EncoderCallback(const flexiv_amr_msgs::msg::AmrEncoder::SharedPtr message)
{
    const auto left_index = static_cast<std::size_t>(left_encoder_index_);
    const auto right_index = static_cast<std::size_t>(right_encoder_index_);
    if (message->legacy_encoder_pulses.size() <= left_index
        || message->legacy_encoder_pulses.size() <= right_index) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Encoder message has %zu legacy pulse value(s), but indexes %d and %d are configured",
            message->legacy_encoder_pulses.size(), left_encoder_index_, right_encoder_index_);
        return;
    }

    const auto stamp = rclcpp::Time(message->header.stamp);
    const auto left_pulses = message->legacy_encoder_pulses[left_index];
    const auto right_pulses = message->legacy_encoder_pulses[right_index];

    if (!initialized_) {
        previous_left_pulses_ = left_pulses;
        previous_right_pulses_ = right_pulses;
        previous_stamp_ = stamp;
        initialized_ = true;
        PublishOdometry(stamp, 0.0, 0.0);
        return;
    }

    const auto dt = (stamp - previous_stamp_).seconds();
    if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Ignoring encoder message with non-increasing timestamp");
        previous_left_pulses_ = left_pulses;
        previous_right_pulses_ = right_pulses;
        previous_stamp_ = stamp;
        return;
    }

    const auto delta_left_pulses = left_pulses - previous_left_pulses_;
    const auto delta_right_pulses = right_pulses - previous_right_pulses_;
    previous_left_pulses_ = left_pulses;
    previous_right_pulses_ = right_pulses;
    previous_stamp_ = stamp;

    const auto delta_left
        = left_direction_ * static_cast<double>(delta_left_pulses) / left_pulses_per_meter_;
    const auto delta_right
        = right_direction_ * static_cast<double>(delta_right_pulses) / right_pulses_per_meter_;
    const auto delta_s = 0.5 * (delta_right + delta_left);
    const auto delta_theta = (delta_right - delta_left) / wheel_separation_;

    x_ += delta_s * std::cos(yaw_ + 0.5 * delta_theta);
    y_ += delta_s * std::sin(yaw_ + 0.5 * delta_theta);
    yaw_ = NormalizeAngle(yaw_ + delta_theta);

    PublishOdometry(stamp, delta_s / dt, delta_theta / dt);
}

double FlexivAmrWheelOdom::NormalizeAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Quaternion FlexivAmrWheelOdom::CreateYawQuaternion() const
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(0.5 * yaw_);
    quaternion.w = std::cos(0.5 * yaw_);
    return quaternion;
}

void FlexivAmrWheelOdom::PublishOdometry(
    const rclcpp::Time& stamp, double linear_velocity, double angular_velocity)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = base_frame_id_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = CreateYawQuaternion();
    odometry.pose.covariance = pose_covariance_;
    odometry.twist.twist.linear.x = linear_velocity;
    odometry.twist.twist.angular.z = angular_velocity;
    odometry.twist.covariance = twist_covariance_;
    odom_publisher_->publish(odometry);
}

} // namespace flexiv_amr::wheel_odom

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<flexiv_amr::wheel_odom::FlexivAmrWheelOdom>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("flexiv_amr_wheel_odom"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

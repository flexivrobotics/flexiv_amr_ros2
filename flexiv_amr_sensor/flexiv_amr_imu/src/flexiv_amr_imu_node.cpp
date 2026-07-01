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

#include "flexiv_amr_imu/flexiv_amr_imu_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <stdexcept>

namespace {

bool CopyCovariance(const std::vector<double>& source, std::array<double, 9>& destination)
{
    if (source.empty()) {
        destination.fill(0.0);
        return true;
    }

    if (source.size() != destination.size()) {
        return false;
    }

    std::copy(source.begin(), source.end(), destination.begin());
    return true;
}

} // namespace

namespace flexiv_amr::imu {

FlexivAmrImuNode::FlexivAmrImuNode()
: Node("flexiv_amr_imu")
{
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", "/flexiv/amr/sensor/imu/raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    subtract_angular_velocity_offset_
        = declare_parameter<bool>("subtract_angular_velocity_offset", true);
    const auto raw_to_imu_rpy = declare_parameter<std::vector<double>>(
        "raw_to_imu_rpy", std::vector<double> {0.0, 0.0, 0.0});

    const auto orientation_covariance
        = declare_parameter<std::vector<double>>("orientation_covariance", std::vector<double> {});
    const auto angular_velocity_covariance = declare_parameter<std::vector<double>>(
        "angular_velocity_covariance", std::vector<double> {});
    const auto linear_acceleration_covariance = declare_parameter<std::vector<double>>(
        "linear_acceleration_covariance", std::vector<double> {});

    if (!CopyCovariance(orientation_covariance, orientation_covariance_)) {
        throw std::runtime_error("orientation_covariance must contain exactly 9 values");
    }
    if (!CopyCovariance(angular_velocity_covariance, angular_velocity_covariance_)) {
        throw std::runtime_error("angular_velocity_covariance must contain exactly 9 values");
    }
    if (!CopyCovariance(linear_acceleration_covariance, linear_acceleration_covariance_)) {
        throw std::runtime_error("linear_acceleration_covariance must contain exactly 9 values");
    }
    ConfigureRawToImuRotation(raw_to_imu_rpy);

    ValidateParameters();

    imu_publisher_
        = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SystemDefaultsQoS());
    raw_imu_subscription_
        = create_subscription<flexiv_amr_msgs::msg::AmrImu>(raw_imu_topic_, rclcpp::SensorDataQoS(),
            [this](const flexiv_amr_msgs::msg::AmrImu::SharedPtr message) { OnRawImu(message); });

    RCLCPP_INFO(get_logger(), "Publishing standard IMU on '%s' from raw IMU topic '%s'",
        imu_topic_.c_str(), raw_imu_topic_.c_str());
}

void FlexivAmrImuNode::ValidateParameters() const
{
    if (raw_imu_topic_.empty()) {
        throw std::runtime_error("raw_imu_topic must be non-empty");
    }
    if (imu_topic_.empty()) {
        throw std::runtime_error("imu_topic must be non-empty");
    }
}

void FlexivAmrImuNode::ConfigureRawToImuRotation(const std::vector<double>& rpy)
{
    if (rpy.size() != 3) {
        throw std::runtime_error("raw_to_imu_rpy must contain exactly 3 values");
    }

    tf2::Quaternion correction;
    correction.setRPY(rpy[0], rpy[1], rpy[2]);
    correction.normalize();

    raw_to_imu_quaternion_ = {correction.x(), correction.y(), correction.z(), correction.w()};

    const tf2::Matrix3x3 rotation(correction);
    raw_to_imu_rotation_ = {rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0],
        rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2]};
}

geometry_msgs::msg::Vector3 FlexivAmrImuNode::TransformVector(double x, double y, double z) const
{
    geometry_msgs::msg::Vector3 transformed;
    transformed.x
        = raw_to_imu_rotation_[0] * x + raw_to_imu_rotation_[1] * y + raw_to_imu_rotation_[2] * z;
    transformed.y
        = raw_to_imu_rotation_[3] * x + raw_to_imu_rotation_[4] * y + raw_to_imu_rotation_[5] * z;
    transformed.z
        = raw_to_imu_rotation_[6] * x + raw_to_imu_rotation_[7] * y + raw_to_imu_rotation_[8] * z;
    return transformed;
}

void FlexivAmrImuNode::OnRawImu(const flexiv_amr_msgs::msg::AmrImu::SharedPtr message)
{
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = message->header.stamp;
    imu.header.frame_id = message->header.frame_id;

    tf2::Quaternion raw_orientation;
    raw_orientation.setRPY(message->euler_rad.x, message->euler_rad.y, message->euler_rad.z);
    raw_orientation.normalize();

    const tf2::Quaternion raw_to_imu(raw_to_imu_quaternion_[0], raw_to_imu_quaternion_[1],
        raw_to_imu_quaternion_[2], raw_to_imu_quaternion_[3]);
    tf2::Quaternion orientation = raw_orientation * raw_to_imu.inverse();
    orientation.normalize();

    imu.orientation.x = orientation.x();
    imu.orientation.y = orientation.y();
    imu.orientation.z = orientation.z();
    imu.orientation.w = orientation.w();
    imu.orientation_covariance = orientation_covariance_;

    const double angular_velocity_x
        = message->angular_velocity_adc.x
          - (subtract_angular_velocity_offset_ ? message->angular_velocity_offset_adc.x : 0.0);
    const double angular_velocity_y
        = message->angular_velocity_adc.y
          - (subtract_angular_velocity_offset_ ? message->angular_velocity_offset_adc.y : 0.0);
    const double angular_velocity_z
        = message->angular_velocity_adc.z
          - (subtract_angular_velocity_offset_ ? message->angular_velocity_offset_adc.z : 0.0);
    const auto angular_velocity
        = TransformVector(angular_velocity_x, angular_velocity_y, angular_velocity_z);

    imu.angular_velocity.x = angular_velocity.x;
    imu.angular_velocity.y = angular_velocity.y;
    imu.angular_velocity.z = angular_velocity.z;
    imu.angular_velocity_covariance = angular_velocity_covariance_;

    const auto linear_acceleration = TransformVector(
        message->acceleration_adc.x, message->acceleration_adc.y, message->acceleration_adc.z);
    imu.linear_acceleration.x = linear_acceleration.x;
    imu.linear_acceleration.y = linear_acceleration.y;
    imu.linear_acceleration.z = linear_acceleration.z;
    imu.linear_acceleration_covariance = linear_acceleration_covariance_;

    imu_publisher_->publish(imu);
}

} // namespace flexiv_amr::imu

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flexiv_amr::imu::FlexivAmrImuNode>());
    rclcpp::shutdown();
    return 0;
}

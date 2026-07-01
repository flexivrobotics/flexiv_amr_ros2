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
#include <flexiv/amr/vendor/seer/seer_data.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "flexiv_amr_sensor/common.hpp"

namespace flexiv_amr::sensor {

class Lidar
{
public:
    explicit Lidar(rclcpp::Node& node)
    : config_(DeclareSensorConfig(node, "amr_lidar", "/flexiv/amr/sensor/lidar/point", "AMR", 10.0))
    , angle_unit_(node.declare_parameter<std::string>("amr_lidar.angle_unit", "auto"))
    , distance_scale_(node.declare_parameter<double>("amr_lidar.distance_scale", 1.0))
    {
        if (angle_unit_ != "auto" && angle_unit_ != "rad" && angle_unit_ != "deg") {
            throw std::invalid_argument("amr_lidar.angle_unit must be auto, rad, or deg");
        }
        if (distance_scale_ <= 0.0) {
            throw std::invalid_argument("amr_lidar.distance_scale must be greater than zero");
        }
        if (config_.enable) {
            publisher_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
                config_.topic, SensorQos(config_.qos_depth));
        }
    }

    bool enabled() const { return config_.enable; }
    double publish_rate() const { return config_.publish_rate; }

    void Publish(rclcpp::Node& node, flexiv::amr::seer::SeerAmrClient& amr)
    {
        const auto points = LaserDataToPoints(amr.GetLaserData(false));

        sensor_msgs::msg::PointCloud2 message;
        message.header.stamp = node.now();
        message.header.frame_id = config_.frame_id;
        sensor_msgs::PointCloud2Modifier modifier(message);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
            sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> x(message, "x");
        sensor_msgs::PointCloud2Iterator<float> y(message, "y");
        sensor_msgs::PointCloud2Iterator<float> z(message, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
        for (const auto& point : points) {
            *x = point[0];
            *y = point[1];
            *z = point[2];
            *intensity = point[3];
            ++x;
            ++y;
            ++z;
            ++intensity;
        }
        message.is_dense = true;
        publisher_->publish(message);
    }

private:
    using Point = std::array<float, 4>;

    double AngleToRadians(double value, const std::vector<double>& reference_values) const
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTau = 2.0 * kPi;
        if (angle_unit_ == "rad") {
            return value;
        }
        if (angle_unit_ == "deg") {
            return value * kPi / 180.0;
        }

        double max_abs_angle = 0.0;
        for (const double angle : reference_values) {
            max_abs_angle = std::max(max_abs_angle, std::abs(angle));
        }
        return max_abs_angle > kTau + 1e-3 ? value * kPi / 180.0 : value;
    }

    std::vector<Point> LaserDataToPoints(const flexiv::amr::seer::SeerLaserData& laser_data) const
    {
        std::vector<Point> points;
        for (const auto& laser : laser_data.lasers) {
            std::vector<double> reference_angles {
                laser.device_info.min_angle,
                laser.device_info.max_angle,
                laser.install_info.yaw,
            };
            const auto sample_count = std::min<std::size_t>(laser.beams.size(), 10);
            reference_angles.reserve(reference_angles.size() + sample_count);
            for (std::size_t index = 0; index < sample_count; ++index) {
                reference_angles.push_back(laser.beams[index].angle);
            }

            const double yaw = AngleToRadians(laser.install_info.yaw, reference_angles);
            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);
            const double min_range = laser.device_info.min_range * distance_scale_;
            const double max_range = laser.device_info.max_range * distance_scale_;

            for (const auto& beam : laser.beams) {
                if (!beam.valid) {
                    continue;
                }
                const double distance = beam.dist * distance_scale_;
                if (distance <= 0.0 || (min_range > 0.0 && distance < min_range)
                    || (max_range > 0.0 && distance > max_range)) {
                    continue;
                }

                const double angle = AngleToRadians(beam.angle, reference_angles);
                const double local_x = distance * std::cos(angle);
                const double local_y = distance * std::sin(angle);
                const double x = laser.install_info.x + local_x * cos_yaw - local_y * sin_yaw;
                const double y = laser.install_info.y + local_x * sin_yaw + local_y * cos_yaw;
                points.push_back({
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(laser.install_info.z),
                    static_cast<float>(beam.rssi),
                });
            }
        }
        return points;
    }

    SensorConfig config_;
    std::string angle_unit_;
    double distance_scale_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

} // namespace flexiv_amr::sensor

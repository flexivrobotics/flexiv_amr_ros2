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

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "flexiv_amr_sensor/battery.hpp"
#include "flexiv_amr_sensor/block_status.hpp"
#include "flexiv_amr_sensor/encoder.hpp"
#include "flexiv_amr_sensor/imu.hpp"
#include "flexiv_amr_sensor/info.hpp"
#include "flexiv_amr_sensor/lidar.hpp"
#include "flexiv_amr_sensor/pose.hpp"
#include "flexiv_amr_sensor/run_info.hpp"
#include "flexiv_amr_sensor/sensor_info.hpp"
#include "flexiv_amr_sensor/speed.hpp"
#include "flexiv_amr_sensor/ultrasonic.hpp"

namespace flexiv_amr::sensor {

class FlexivAmrSensorNode : public rclcpp::Node
{
public:
    FlexivAmrSensorNode();
    ~FlexivAmrSensorNode() override;

private:
    template <typename SensorT>
    void AddSensor(const std::shared_ptr<SensorT>& sensor)
    {
        if (!sensor->enabled()) {
            return;
        }
        timers_.push_back(create_wall_timer(
            std::chrono::duration<double>(1.0 / sensor->publish_rate()),
            [this, sensor]() { PollSensor([this, sensor]() { sensor->Publish(*this, *amr_); }); }));
    }

    bool ConnectIfNeeded();
    void PollSensor(const std::function<void()>& callback);

    std::string amr_ip_;
    std::chrono::duration<double> reconnect_interval_;
    std::chrono::steady_clock::time_point next_connect_attempt_;
    bool connected_ {false};
    std::unique_ptr<flexiv::amr::seer::SeerAmrClient> amr_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

} // namespace flexiv_amr::sensor

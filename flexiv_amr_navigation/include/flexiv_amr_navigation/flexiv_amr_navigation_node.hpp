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

#include <flexiv_amr_msgs/srv/amr_cancel_navigation.hpp>
#include <flexiv_amr_msgs/srv/amr_cancel_relocation.hpp>
#include <flexiv_amr_msgs/srv/amr_fixed_path_navigation.hpp>
#include <flexiv_amr_msgs/srv/amr_location_status.hpp>
#include <flexiv_amr_msgs/srv/amr_navigation_status.hpp>
#include <flexiv_amr_msgs/srv/amr_pause_navigation.hpp>
#include <flexiv_amr_msgs/srv/amr_relocation.hpp>
#include <flexiv_amr_msgs/srv/amr_resume_navigation.hpp>
#include <flexiv_amr_msgs/srv/amr_rotation.hpp>
#include <flexiv_amr_msgs/srv/amr_specified_path_navigation.hpp>
#include <flexiv_amr_msgs/srv/amr_task_status.hpp>
#include <flexiv_amr_msgs/srv/amr_translation.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace flexiv_amr::navigation {

class FlexivAmrnavigationNode : public rclcpp::Node
{
public:
    FlexivAmrnavigationNode();
    ~FlexivAmrnavigationNode() override;

private:
    bool ConnectIfNeeded();
    void DisconnectAfterFailure();
    void OnTranslation(const flexiv_amr_msgs::srv::AmrTranslation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrTranslation::Response::SharedPtr response);
    void OnRotation(const flexiv_amr_msgs::srv::AmrRotation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrRotation::Response::SharedPtr response);
    void OnTaskStatus(const flexiv_amr_msgs::srv::AmrTaskStatus::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrTaskStatus::Response::SharedPtr response);
    void OnNavigationStatus(
        const flexiv_amr_msgs::srv::AmrNavigationStatus::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrNavigationStatus::Response::SharedPtr response);
    void OnFixedPathNavigation(
        const flexiv_amr_msgs::srv::AmrFixedPathNavigation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrFixedPathNavigation::Response::SharedPtr response);
    void OnSpecifiedPathNavigation(
        const flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation::Response::SharedPtr response);
    void OnPauseNavigation(
        const flexiv_amr_msgs::srv::AmrPauseNavigation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrPauseNavigation::Response::SharedPtr response);
    void OnResumeNavigation(
        const flexiv_amr_msgs::srv::AmrResumeNavigation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrResumeNavigation::Response::SharedPtr response);
    void OnCancelNavigation(
        const flexiv_amr_msgs::srv::AmrCancelNavigation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrCancelNavigation::Response::SharedPtr response);
    void OnRelocation(const flexiv_amr_msgs::srv::AmrRelocation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrRelocation::Response::SharedPtr response);
    void OnCancelRelocation(
        const flexiv_amr_msgs::srv::AmrCancelRelocation::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrCancelRelocation::Response::SharedPtr response);
    void OnLocationStatus(const flexiv_amr_msgs::srv::AmrLocationStatus::Request::SharedPtr request,
        flexiv_amr_msgs::srv::AmrLocationStatus::Response::SharedPtr response);

    std::string amr_ip_;
    bool connected_ {false};
    std::unique_ptr<flexiv::amr::seer::SeerAmrClient> amr_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrTranslation>::SharedPtr translation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrRotation>::SharedPtr rotation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrTaskStatus>::SharedPtr task_status_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrNavigationStatus>::SharedPtr
        navigation_status_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrFixedPathNavigation>::SharedPtr
        fixed_path_navigation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation>::SharedPtr
        specified_path_navigation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrPauseNavigation>::SharedPtr pause_navigation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrResumeNavigation>::SharedPtr
        resume_navigation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrCancelNavigation>::SharedPtr
        cancel_navigation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrRelocation>::SharedPtr relocation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrCancelRelocation>::SharedPtr
        cancel_relocation_service_;
    rclcpp::Service<flexiv_amr_msgs::srv::AmrLocationStatus>::SharedPtr location_status_service_;
};

} // namespace flexiv_amr::navigation

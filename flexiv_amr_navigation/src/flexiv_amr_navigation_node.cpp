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

#include "flexiv_amr_navigation/flexiv_amr_navigation_node.hpp"

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace flexiv_amr::navigation {
namespace {

template <typename T>
bool HasValueAt(const std::vector<T>& values, std::size_t index)
{
    return index < values.size();
}

bool ValidateOptionalArraySize(std::size_t expected_size, std::size_t actual_size,
    const std::string& field_name, std::string& error)
{
    if (actual_size == 0 || actual_size == expected_size) {
        return true;
    }

    error = "Field '" + field_name + "' must be empty or match source_ids size";
    return false;
}

void ApplyOptionalPathFields(flexiv::amr::seer::SeerPathNaviCommand& command,
    const std::string& task_id, bool use_angle, double angle, const std::string& method,
    bool use_max_speed, double max_speed, bool use_max_wspeed, double max_wspeed, bool use_max_acc,
    double max_acc, bool use_max_wacc, double max_wacc, bool use_duration, double duration,
    bool use_orientation, double orientation, bool use_spin, bool spin)
{
    if (!task_id.empty()) {
        command.task_id = task_id;
    }
    if (use_angle) {
        command.angle = angle;
    }
    if (!method.empty()) {
        command.method = method;
    }
    if (use_max_speed) {
        command.max_speed = max_speed;
    }
    if (use_max_wspeed) {
        command.max_wspeed = max_wspeed;
    }
    if (use_max_acc) {
        command.max_acc = max_acc;
    }
    if (use_max_wacc) {
        command.max_wacc = max_wacc;
    }
    if (use_duration) {
        command.duration = duration;
    }
    if (use_orientation) {
        command.orientation = orientation;
    }
    if (use_spin) {
        command.spin = spin;
    }
}

} // namespace

FlexivAmrnavigationNode::FlexivAmrnavigationNode()
: Node("flexiv_amr_navigation")
, amr_ip_(declare_parameter<std::string>("amr_ip", ""))
{
    const auto translation_service_name
        = declare_parameter<std::string>("translation_service", "/flexiv/amr/navigation/translate");
    const auto rotation_service_name
        = declare_parameter<std::string>("rotation_service", "/flexiv/amr/navigation/rotate");
    const auto task_status_service_name = declare_parameter<std::string>(
        "task_status_service", "/flexiv/amr/navigation/task_status");
    const auto navigation_status_service_name = declare_parameter<std::string>(
        "navigation_status_service", "/flexiv/amr/navigation/status");
    const auto fixed_path_navigation_service_name = declare_parameter<std::string>(
        "fixed_path_navigation_service", "/flexiv/amr/navigation/fixed_path");
    const auto specified_path_navigation_service_name = declare_parameter<std::string>(
        "specified_path_navigation_service", "/flexiv/amr/navigation/specified_path");
    const auto pause_navigation_service_name = declare_parameter<std::string>(
        "pause_navigation_service", "/flexiv/amr/navigation/pause");
    const auto resume_navigation_service_name = declare_parameter<std::string>(
        "resume_navigation_service", "/flexiv/amr/navigation/resume");
    const auto cancel_navigation_service_name = declare_parameter<std::string>(
        "cancel_navigation_service", "/flexiv/amr/navigation/cancel");
    const auto relocation_service_name
        = declare_parameter<std::string>("relocation_service", "/flexiv/amr/control/reloc");
    const auto cancel_relocation_service_name = declare_parameter<std::string>(
        "cancel_relocation_service", "/flexiv/amr/control/cancel_reloc");
    const auto location_status_service_name = declare_parameter<std::string>(
        "location_status_service", "/flexiv/amr/control/location_status");
    const auto connect_timeout_ms = declare_parameter<int>("connect_timeout_ms", 3000);
    const auto send_timeout_ms = declare_parameter<int>("send_timeout_ms", 3000);
    const auto recv_timeout_ms = declare_parameter<int>("recv_timeout_ms", 3000);

    if (amr_ip_.empty()) {
        throw std::invalid_argument("The 'amr_ip' parameter must not be empty");
    }
    if (translation_service_name.empty() || rotation_service_name.empty()
        || task_status_service_name.empty() || navigation_status_service_name.empty()
        || fixed_path_navigation_service_name.empty()
        || specified_path_navigation_service_name.empty() || pause_navigation_service_name.empty()
        || resume_navigation_service_name.empty() || cancel_navigation_service_name.empty()
        || relocation_service_name.empty() || cancel_relocation_service_name.empty()
        || location_status_service_name.empty()) {
        throw std::invalid_argument("Navigation and relocation service names must not be empty");
    }
    if (connect_timeout_ms <= 0 || send_timeout_ms <= 0 || recv_timeout_ms <= 0) {
        throw std::invalid_argument("SEER timeout parameters must be greater than zero");
    }

    flexiv::amr::seer::SeerAmrOptions options;
    options.host = amr_ip_;
    options.auto_connect = false;
    options.auto_reconnect = true;
    options.time_out.connect_timeout_ms = connect_timeout_ms;
    options.time_out.send_timeout_ms = send_timeout_ms;
    options.time_out.recv_timeout_ms = recv_timeout_ms;
    amr_ = std::make_unique<flexiv::amr::seer::SeerAmrClient>(options);

    translation_service_
        = create_service<flexiv_amr_msgs::srv::AmrTranslation>(translation_service_name,
            [this](const flexiv_amr_msgs::srv::AmrTranslation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrTranslation::Response::SharedPtr response) {
                OnTranslation(request, response);
            });
    rotation_service_ = create_service<flexiv_amr_msgs::srv::AmrRotation>(rotation_service_name,
        [this](const flexiv_amr_msgs::srv::AmrRotation::Request::SharedPtr request,
            flexiv_amr_msgs::srv::AmrRotation::Response::SharedPtr response) {
            OnRotation(request, response);
        });
    task_status_service_
        = create_service<flexiv_amr_msgs::srv::AmrTaskStatus>(task_status_service_name,
            [this](const flexiv_amr_msgs::srv::AmrTaskStatus::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrTaskStatus::Response::SharedPtr response) {
                OnTaskStatus(request, response);
            });
    navigation_status_service_
        = create_service<flexiv_amr_msgs::srv::AmrNavigationStatus>(navigation_status_service_name,
            [this](const flexiv_amr_msgs::srv::AmrNavigationStatus::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrNavigationStatus::Response::SharedPtr response) {
                OnNavigationStatus(request, response);
            });
    fixed_path_navigation_service_ = create_service<flexiv_amr_msgs::srv::AmrFixedPathNavigation>(
        fixed_path_navigation_service_name,
        [this](const flexiv_amr_msgs::srv::AmrFixedPathNavigation::Request::SharedPtr request,
            flexiv_amr_msgs::srv::AmrFixedPathNavigation::Response::SharedPtr response) {
            OnFixedPathNavigation(request, response);
        });
    specified_path_navigation_service_
        = create_service<flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation>(
            specified_path_navigation_service_name,
            [this](
                const flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation::Response::SharedPtr response) {
                OnSpecifiedPathNavigation(request, response);
            });
    pause_navigation_service_
        = create_service<flexiv_amr_msgs::srv::AmrPauseNavigation>(pause_navigation_service_name,
            [this](const flexiv_amr_msgs::srv::AmrPauseNavigation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrPauseNavigation::Response::SharedPtr response) {
                OnPauseNavigation(request, response);
            });
    resume_navigation_service_
        = create_service<flexiv_amr_msgs::srv::AmrResumeNavigation>(resume_navigation_service_name,
            [this](const flexiv_amr_msgs::srv::AmrResumeNavigation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrResumeNavigation::Response::SharedPtr response) {
                OnResumeNavigation(request, response);
            });
    cancel_navigation_service_
        = create_service<flexiv_amr_msgs::srv::AmrCancelNavigation>(cancel_navigation_service_name,
            [this](const flexiv_amr_msgs::srv::AmrCancelNavigation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrCancelNavigation::Response::SharedPtr response) {
                OnCancelNavigation(request, response);
            });
    relocation_service_
        = create_service<flexiv_amr_msgs::srv::AmrRelocation>(relocation_service_name,
            [this](const flexiv_amr_msgs::srv::AmrRelocation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrRelocation::Response::SharedPtr response) {
                OnRelocation(request, response);
            });
    cancel_relocation_service_
        = create_service<flexiv_amr_msgs::srv::AmrCancelRelocation>(cancel_relocation_service_name,
            [this](const flexiv_amr_msgs::srv::AmrCancelRelocation::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrCancelRelocation::Response::SharedPtr response) {
                OnCancelRelocation(request, response);
            });
    location_status_service_
        = create_service<flexiv_amr_msgs::srv::AmrLocationStatus>(location_status_service_name,
            [this](const flexiv_amr_msgs::srv::AmrLocationStatus::Request::SharedPtr request,
                flexiv_amr_msgs::srv::AmrLocationStatus::Response::SharedPtr response) {
                OnLocationStatus(request, response);
            });

    RCLCPP_INFO(get_logger(),
        "Navigation and relocation services ready: '%s', '%s', '%s', '%s', '%s', '%s', '%s', "
        "'%s', '%s', '%s', '%s', '%s'",
        translation_service_name.c_str(), rotation_service_name.c_str(),
        task_status_service_name.c_str(), navigation_status_service_name.c_str(),
        fixed_path_navigation_service_name.c_str(), specified_path_navigation_service_name.c_str(),
        pause_navigation_service_name.c_str(), resume_navigation_service_name.c_str(),
        cancel_navigation_service_name.c_str(), relocation_service_name.c_str(),
        cancel_relocation_service_name.c_str(), location_status_service_name.c_str());
}

FlexivAmrnavigationNode::~FlexivAmrnavigationNode()
{
    if (connected_ && amr_) {
        try {
            amr_->Disconnect();
        } catch (const std::exception& error) {
            RCLCPP_WARN(get_logger(), "Failed to disconnect cleanly: %s", error.what());
        }
    }
}

bool FlexivAmrnavigationNode::ConnectIfNeeded()
{
    if (connected_) {
        return true;
    }

    try {
        RCLCPP_INFO(get_logger(), "Connecting to SEER AMR at %s", amr_ip_.c_str());
        amr_->Connect();
        connected_ = true;
        RCLCPP_INFO(get_logger(), "Connected to SEER AMR");
        return true;
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to SEER AMR: %s", error.what());
        return false;
    }
}

void FlexivAmrnavigationNode::DisconnectAfterFailure()
{
    connected_ = false;
    try {
        amr_->Disconnect();
    } catch (const std::exception&) {
        // Preserve the original command failure in the service response.
    }
}

void FlexivAmrnavigationNode::OnTranslation(
    const flexiv_amr_msgs::srv::AmrTranslation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrTranslation::Response::SharedPtr response)
{
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    flexiv::amr::seer::SeerTranslationCommand command;
    command.dist = request->dist;
    command.vx = request->vx;
    command.vy = request->vy;
    command.mode = request->mode;

    try {
        response->success = amr_->ExecuteTranslation(command);
        response->message = response->success ? "Translation command accepted"
                                              : "SEER AMR rejected the translation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnRotation(
    const flexiv_amr_msgs::srv::AmrRotation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrRotation::Response::SharedPtr response)
{
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    flexiv::amr::seer::SeerRotationCommand command;
    command.angle = request->angle;
    command.vw = request->vw;
    command.mode = request->mode;

    try {
        response->success = amr_->ExecuteRotation(command);
        response->message = response->success ? "Rotation command accepted"
                                              : "SEER AMR rejected the rotation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnTaskStatus(
    const flexiv_amr_msgs::srv::AmrTaskStatus::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrTaskStatus::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        const auto status = amr_->GetTaskStatus();
        response->success = true;
        response->message = "Task status received";
        response->percentage = status.percentage;
        response->distance = status.distance;
        response->closest_target = status.closest_target;
        response->source_name = status.source_name;
        response->target_name = status.target_name;
        response->info = status.info;
        response->task_ids.reserve(status.task_states.size());
        response->statuses.reserve(status.task_states.size());
        response->types.reserve(status.task_states.size());
        for (const auto& task_state : status.task_states) {
            response->task_ids.push_back(task_state.task_id);
            response->statuses.push_back(task_state.status);
            response->types.push_back(task_state.type);
        }
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnNavigationStatus(
    const flexiv_amr_msgs::srv::AmrNavigationStatus::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrNavigationStatus::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        const auto status = amr_->GetNavigationStatus();
        response->success = true;
        response->message = "Navigation status received";
        response->task_status = status.task_status;
        response->task_type = status.task_type;
        response->target_id = status.target_id;
        response->finished_path = status.finished_path;
        response->unfinished_path = status.unfinished_path;
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnFixedPathNavigation(
    const flexiv_amr_msgs::srv::AmrFixedPathNavigation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrFixedPathNavigation::Response::SharedPtr response)
{
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    flexiv::amr::seer::SeerPathNaviCommand command;
    command.source_id = request->source_id;
    command.id = request->target_id;
    ApplyOptionalPathFields(command, request->task_id, request->use_angle, request->angle,
        request->method, request->use_max_speed, request->max_speed, request->use_max_wspeed,
        request->max_wspeed, request->use_max_acc, request->max_acc, request->use_max_wacc,
        request->max_wacc, request->use_duration, request->duration, request->use_orientation,
        request->orientation, request->use_spin, request->spin);

    try {
        response->success = amr_->ExecuteFixedPathNavigation(command);
        response->message = response->success
                                ? "Fixed path navigation command accepted"
                                : "SEER AMR rejected the fixed path navigation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnSpecifiedPathNavigation(
    const flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrSpecifiedPathNavigation::Response::SharedPtr response)
{
    const auto task_count = request->source_ids.size();
    if (task_count == 0) {
        response->success = false;
        response->message = "source_ids must not be empty";
        return;
    }
    if (request->target_ids.size() != task_count || request->task_ids.size() != task_count) {
        response->success = false;
        response->message = "target_ids and task_ids must match source_ids size";
        return;
    }

    std::string error;
    if (!ValidateOptionalArraySize(task_count, request->use_angles.size(), "use_angles", error)
        || !ValidateOptionalArraySize(task_count, request->angles.size(), "angles", error)
        || !ValidateOptionalArraySize(task_count, request->methods.size(), "methods", error)
        || !ValidateOptionalArraySize(
            task_count, request->use_max_speeds.size(), "use_max_speeds", error)
        || !ValidateOptionalArraySize(task_count, request->max_speeds.size(), "max_speeds", error)
        || !ValidateOptionalArraySize(
            task_count, request->use_max_wspeeds.size(), "use_max_wspeeds", error)
        || !ValidateOptionalArraySize(task_count, request->max_wspeeds.size(), "max_wspeeds", error)
        || !ValidateOptionalArraySize(
            task_count, request->use_max_accs.size(), "use_max_accs", error)
        || !ValidateOptionalArraySize(task_count, request->max_accs.size(), "max_accs", error)
        || !ValidateOptionalArraySize(
            task_count, request->use_max_waccs.size(), "use_max_waccs", error)
        || !ValidateOptionalArraySize(task_count, request->max_waccs.size(), "max_waccs", error)
        || !ValidateOptionalArraySize(
            task_count, request->use_durations.size(), "use_durations", error)
        || !ValidateOptionalArraySize(task_count, request->durations.size(), "durations", error)
        || !ValidateOptionalArraySize(
            task_count, request->use_orientations.size(), "use_orientations", error)
        || !ValidateOptionalArraySize(
            task_count, request->orientations.size(), "orientations", error)
        || !ValidateOptionalArraySize(task_count, request->use_spins.size(), "use_spins", error)
        || !ValidateOptionalArraySize(task_count, request->spins.size(), "spins", error)) {
        response->success = false;
        response->message = error;
        return;
    }

    std::vector<flexiv::amr::seer::SeerPathNaviCommand> commands;
    commands.reserve(task_count);
    for (std::size_t index = 0; index < task_count; ++index) {
        if (request->source_ids[index].empty() || request->target_ids[index].empty()
            || request->task_ids[index].empty()) {
            response->success = false;
            response->message
                = "source_ids, target_ids, and task_ids must not contain empty values";
            return;
        }

        flexiv::amr::seer::SeerPathNaviCommand command;
        command.source_id = request->source_ids[index];
        command.id = request->target_ids[index];
        ApplyOptionalPathFields(command, request->task_ids[index],
            HasValueAt(request->use_angles, index) && request->use_angles[index],
            HasValueAt(request->angles, index) ? request->angles[index] : 0.0,
            HasValueAt(request->methods, index) ? request->methods[index] : "",
            HasValueAt(request->use_max_speeds, index) && request->use_max_speeds[index],
            HasValueAt(request->max_speeds, index) ? request->max_speeds[index] : 0.0,
            HasValueAt(request->use_max_wspeeds, index) && request->use_max_wspeeds[index],
            HasValueAt(request->max_wspeeds, index) ? request->max_wspeeds[index] : 0.0,
            HasValueAt(request->use_max_accs, index) && request->use_max_accs[index],
            HasValueAt(request->max_accs, index) ? request->max_accs[index] : 0.0,
            HasValueAt(request->use_max_waccs, index) && request->use_max_waccs[index],
            HasValueAt(request->max_waccs, index) ? request->max_waccs[index] : 0.0,
            HasValueAt(request->use_durations, index) && request->use_durations[index],
            HasValueAt(request->durations, index) ? request->durations[index] : 0.0,
            HasValueAt(request->use_orientations, index) && request->use_orientations[index],
            HasValueAt(request->orientations, index) ? request->orientations[index] : 0.0,
            HasValueAt(request->use_spins, index) && request->use_spins[index],
            HasValueAt(request->spins, index) && request->spins[index]);
        commands.emplace_back(std::move(command));
    }

    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->ExecuteSpecifiedPathNavigation(commands);
        response->message = response->success
                                ? "Specified path navigation command accepted"
                                : "SEER AMR rejected the specified path navigation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnPauseNavigation(
    const flexiv_amr_msgs::srv::AmrPauseNavigation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrPauseNavigation::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->PauseNavigation();
        response->message = response->success ? "Pause navigation command accepted"
                                              : "SEER AMR rejected the pause navigation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnResumeNavigation(
    const flexiv_amr_msgs::srv::AmrResumeNavigation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrResumeNavigation::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->ResumeNavigation();
        response->message = response->success ? "Resume navigation command accepted"
                                              : "SEER AMR rejected the resume navigation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnCancelNavigation(
    const flexiv_amr_msgs::srv::AmrCancelNavigation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrCancelNavigation::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->CancelNavigation();
        response->message = response->success ? "Cancel navigation command accepted"
                                              : "SEER AMR rejected the cancel navigation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnRelocation(
    const flexiv_amr_msgs::srv::AmrRelocation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrRelocation::Response::SharedPtr response)
{
    if (!std::isfinite(request->x) || !std::isfinite(request->y)
        || (request->use_angle && !std::isfinite(request->angle))
        || (request->use_length && !std::isfinite(request->length))) {
        response->success = false;
        response->message = "Relocation coordinates, angle, and length must be finite";
        return;
    }

    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    flexiv::amr::seer::SeerRelocationRequest relocation;
    relocation.is_auto = request->is_auto;
    if (!request->is_auto) {
        relocation.x = request->x;
        relocation.y = request->y;
        if (request->use_angle) {
            relocation.angle = request->angle;
        }
        if (request->use_length) {
            relocation.length = request->length;
        }
        if (request->home) {
            relocation.home = true;
        }
    }

    try {
        response->success = amr_->PerformRelocation(relocation);
        response->message = response->success ? "Relocation command accepted"
                                              : "SEER AMR rejected the relocation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnCancelRelocation(
    const flexiv_amr_msgs::srv::AmrCancelRelocation::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrCancelRelocation::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->success = amr_->CancelRelocation();
        response->message = response->success ? "Cancel relocation command accepted"
                                              : "SEER AMR rejected the cancel relocation command";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

void FlexivAmrnavigationNode::OnLocationStatus(
    const flexiv_amr_msgs::srv::AmrLocationStatus::Request::SharedPtr request,
    flexiv_amr_msgs::srv::AmrLocationStatus::Response::SharedPtr response)
{
    (void)request;
    if (!ConnectIfNeeded()) {
        response->success = false;
        response->message = "Failed to connect to SEER AMR";
        return;
    }

    try {
        response->reloc_status = amr_->GetLocationStatus();
        response->success = true;
        response->message = "Location status received";
    } catch (const std::exception& error) {
        response->success = false;
        response->message = error.what();
        DisconnectAfterFailure();
    }
}

} // namespace flexiv_amr::navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<flexiv_amr::navigation::FlexivAmrnavigationNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("flexiv_amr_navigation"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

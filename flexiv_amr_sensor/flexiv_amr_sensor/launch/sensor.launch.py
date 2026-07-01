# Copyright 2026 cmc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_sensor_config():
    parameter_file = os.path.join(
        get_package_share_directory("flexiv_amr_sensor"),
        "config",
        "sensors.yaml",
    )
    with open(parameter_file, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def _flatten_sensor_config(config):
    parameters = {}
    for node_name, node_config in config.items():
        ros_parameters = node_config.get("ros__parameters", {})
        for key, value in ros_parameters.items():
            parameters[f"{node_name}.{key}"] = value
    return parameters


def generate_launch_description() -> LaunchDescription:
    config = _load_sensor_config()
    sensor_parameters = _flatten_sensor_config(config)
    amr_ip = LaunchConfiguration("amr_ip")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "amr_ip",
                default_value="192.168.192.5",
                description="SEER AMR IPv4 address or hostname",
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="flexiv_amr_sensor",
                executable="flexiv_amr_sensor",
                name="flexiv_amr_sensor",
                output="screen",
                parameters=[sensor_parameters, {"amr_ip": amr_ip}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )

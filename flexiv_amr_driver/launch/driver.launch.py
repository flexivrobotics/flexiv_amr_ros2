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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "amr_ip",
                default_value="192.168.192.5",
                description="SEER AMR IPv4 address or hostname",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value="500",
                description="Open-loop command duration in milliseconds",
            ),
            Node(
                package="flexiv_amr_driver",
                executable="flexiv_amr_driver_node",
                name="flexiv_amr_driver",
                output="screen",
                parameters=[
                    {
                        "amr_ip": LaunchConfiguration("amr_ip"),
                        "duration": ParameterValue(
                            LaunchConfiguration("duration"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )

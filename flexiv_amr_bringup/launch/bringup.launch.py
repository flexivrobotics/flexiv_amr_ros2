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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    parameter_file = os.path.join(
        get_package_share_directory("flexiv_amr_bringup"),
        "config",
        "launch_params.yaml",
    )
    with open(parameter_file, "r", encoding="utf-8") as file:
        launch_params = yaml.safe_load(file) or {}

    amr_ip = str(launch_params.get("amr_ip", "")).strip()
    if not amr_ip:
        raise RuntimeError("The bringup parameter 'amr_ip' must not be empty")

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("flexiv_amr_sensor"), "launch", "sensor.launch.py"]
            )
        ),
        launch_arguments={"amr_ip": amr_ip}.items(),
    )
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("flexiv_amr_driver"), "launch", "driver.launch.py"]
            )
        ),
        launch_arguments={"amr_ip": amr_ip}.items(),
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_amr_navigation"),
                    "launch",
                    "navigation.launch.py",
                ]
            )
        ),
        launch_arguments={"amr_ip": amr_ip}.items(),
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_amr_description"),
                    "launch",
                    "display.launch.py",
                ]
            )
        ),
        launch_arguments={"config_file": parameter_file}.items(),
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[
            {
                "target_frame": "base_link",
                "transform_tolerance": 0.01,
                "min_height": -10.0,
                "max_height": 10.0,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00436,
                "scan_time": 0.1,
                "range_min": 0.05,
                "range_max": 10.0,
                "use_inf": True,
            }
        ],
        remappings=[
            ("cloud_in", "/flexiv/amr/sensor/lidar/point"),
            ("scan", "/scan"),
        ],
    )

    wheel_odom_node = Node(
        package="flexiv_amr_wheel_odom",
        executable="flexiv_amr_wheel_odom",
        name="flexiv_amr_wheel_odom",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_amr_wheel_odom"),
                    "config",
                    "wheel_odom.yaml",
                ]
            )
        ],
        output="screen",
    )

    imu_node = Node(
        package="flexiv_amr_imu",
        executable="flexiv_amr_imu_node",
        name="flexiv_amr_imu",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_amr_imu"),
                    "config",
                    "imu.yaml",
                ]
            )
        ],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("flexiv_amr_bringup"),
                    "config",
                    "ekf.yaml",
                ]
            )
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sensor_launch,  # if you run with ros2 bag,you can comment this line to avoid the conflict of sensor driver
            driver_launch,
            navigation_launch,
            description_launch,
            pointcloud_to_laserscan_node,
            wheel_odom_node,
            imu_node,
            ekf_node,
        ]
    )

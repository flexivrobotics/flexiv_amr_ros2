# Flexiv AMR ROS 2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)

ROS 2 packages for integrating a Flexiv Autonomous Mobile Robot (AMR) with the ROS ecosystem. The stack wraps [`flexiv_amr_sdk`](../flexiv_amr_sdk), providing SEER TCP communication, sensor publishing, open-loop motion control, navigation and relocation services, wheel odometry, standard IMU conversion, `robot_localization` fusion, and an RViz-ready FMR-300 robot description.

## References

- [`flexiv_amr_sdk`](../flexiv_amr_sdk) provides the C++ interface to the AMR controller.
- [ROS 2 Humble documentation](https://docs.ros.org/en/humble/index.html) provides the ROS 2 installation and development guides.
- [`robot_localization`](https://docs.ros.org/en/humble/p/robot_localization/) provides the EKF used to fuse wheel odometry and IMU yaw.

## Compatibility

| **Supported OS** | **Supported ROS 2 distribution** |
| ---------------- | -------------------------------- |
| Ubuntu 22.04     | [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) |

Other Ubuntu releases and ROS 2 distributions may work, but are not currently tested.

## Packages

| Package | Description |
| ------- | ----------- |
| `flexiv_amr_bringup` | Top-level launch and shared configuration, including `robot_localization`. |
| `flexiv_amr_description` | FMR-300/Rizon robot description and RViz display. |
| `flexiv_amr_driver` | Open-loop motion driver, stop-motion service, and emergency-stop service. |
| `flexiv_amr_msgs` | AMR messages and service definitions. |
| `flexiv_amr_navigation` | Navigation, relocation, and status services. |
| `flexiv_amr_sensor` | Raw AMR sensor and status publishers backed by `flexiv_amr_sdk`. |
| `flexiv_amr_imu` | Converts `flexiv_amr_msgs/msg/AmrImu` to standard `sensor_msgs/msg/Imu`. |
| `flexiv_amr_wheel_odom` | Differential-drive wheel odometry from encoder pulses. |

> [!NOTE]
> `flexiv_amr_navigation` retains its existing package-name spelling for compatibility.

## Getting Started

This repository is developed and validated with ROS 2 Humble on Ubuntu 22.04. The AMR-facing packages require a built installation of `flexiv_amr_sdk`.

1. Install ROS 2 Humble, then the required ROS packages:

   ```bash
   sudo apt update
   sudo apt install -y \
     python3-colcon-common-extensions \
     python3-rosdep \
     ros-humble-joint-state-publisher \
     ros-humble-joint-state-publisher-gui \
     ros-humble-robot-state-publisher \
     ros-humble-rviz2 \
     ros-humble-pointcloud-to-laserscan \
     ros-humble-robot-localization \
   ```

2. Build and install [`flexiv_amr_sdk`](../flexiv_amr_sdk). Set `AMR_SDK_PREFIX` to its installation prefix.

3. Install ROS dependencies and build this workspace:

   ```bash
   cd ~/flexiv_amr_ros2
   source /opt/ros/humble/setup.bash

   rosdep update
   rosdep install --from-paths . --ignore-src --skip-keys flexiv_amr_sdk \
     --rosdistro humble -r -y

   colcon build --symlink-install \
     --cmake-args -DCMAKE_PREFIX_PATH="$AMR_SDK_PREFIX"
   source install/setup.bash
   ```

> [!IMPORTANT]
> Source ROS 2 and the workspace in every new terminal:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/flexiv_amr_ros2/install/setup.bash
> ```

## Usage

Configure the AMR IP address and robot-description options in [`flexiv_amr_bringup/config/launch_params.yaml`](flexiv_amr_bringup/config/launch_params.yaml). Then launch the complete stack:

```bash
ros2 launch flexiv_amr_bringup bringup.launch.py
```

The bringup launch starts:

- `flexiv_amr_sensor` for raw AMR sensors and status topics.
- `flexiv_amr_driver` for open-loop local motion control.
- `flexiv_amr_navigation` for navigation, relocation, and status services.
- `flexiv_amr_description` for robot state publishing and RViz visualization.
- `pointcloud_to_laserscan` to convert `/flexiv/amr/sensor/lidar/point` to `/scan`.
- `flexiv_amr_wheel_odom` to publish `/wheel_odom`.
- `flexiv_amr_imu` to publish `/imu`.
- `robot_localization/ekf_node` to fuse wheel odometry and IMU yaw.

The wheel odometry node only publishes `nav_msgs/msg/Odometry` on `/wheel_odom`; it does not publish TF. The EKF is responsible for the fused odometry output and TF according to [`flexiv_amr_bringup/config/ekf.yaml`](flexiv_amr_bringup/config/ekf.yaml).

> [!WARNING]
> Ensure the workstation can reach the configured AMR IP and that the SEER TCP services are enabled before launching. Open-loop motion cancels the currently active navigation task.

## Main Topics

| Topic | Type | Producer | Notes |
| ----- | ---- | -------- | ----- |
| `/flexiv/amr/sensor/lidar/point` | `sensor_msgs/msg/PointCloud2` | `flexiv_amr_sensor` | Raw AMR lidar point cloud. |
| `/scan` | `sensor_msgs/msg/LaserScan` | `pointcloud_to_laserscan` | Converted from the AMR point cloud. |
| `/flexiv/amr/sensor/imu/raw` | `flexiv_amr_msgs/msg/AmrImu` | `flexiv_amr_sensor` | Raw AMR IMU message. |
| `/imu` | `sensor_msgs/msg/Imu` | `flexiv_amr_imu` | Standard IMU message; uses the raw message `header.frame_id`. |
| `/flexiv/amr/sensor/encoder/raw` | `flexiv_amr_msgs/msg/AmrEncoder` | `flexiv_amr_sensor` | Raw encoder pulses. |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `flexiv_amr_wheel_odom` | Differential-drive wheel odometry, no TF. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `robot_localization` | EKF output from `/wheel_odom` and `/imu`. |
| `/flexiv/amr/control/motion` | `flexiv_amr_msgs/msg/AmrLocalMotion` | User command | Open-loop local motion command. |

Other AMR status topics, such as battery, pose, run info, and block status, are configured in [`flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml`](flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml).

## Configuration

| File | Purpose |
| ---- | ------- |
| [`flexiv_amr_bringup/config/launch_params.yaml`](flexiv_amr_bringup/config/launch_params.yaml) | AMR IP and robot-description options. |
| [`flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml`](flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml) | Raw sensor topic names, frame IDs, publish rates, and enable flags. |
| [`flexiv_amr_sensor/flexiv_amr_wheel_odom/config/wheel_odom.yaml`](flexiv_amr_sensor/flexiv_amr_wheel_odom/config/wheel_odom.yaml) | Encoder topic, `/wheel_odom` topic, wheel separation, encoder scale, direction, and odometry frames. |
| [`flexiv_amr_sensor/flexiv_amr_imu/config/imu.yaml`](flexiv_amr_sensor/flexiv_amr_imu/config/imu.yaml) | Raw IMU topic, `/imu` topic, raw-to-IMU axis correction, angular-velocity offset handling, and IMU covariances. |
| [`flexiv_amr_bringup/config/ekf.yaml`](flexiv_amr_bringup/config/ekf.yaml) | `robot_localization` input selection, frames, process noise, and initial estimate covariance. |

For a two-wheel differential AMR, tune `left_pulses_per_meter`, `right_pulses_per_meter`, `wheel_separation`, `left_direction`, and `right_direction` in `wheel_odom.yaml`. If the raw IMU axes do not match the robot frame, tune `raw_to_imu_rpy` in `imu.yaml`; do not fix raw sensor-axis errors by changing unrelated URDF joints unless the physical mounting transform is actually wrong.

Measurement noise for EKF fusion comes from the covariance fields in the input messages. For IMU, tune the covariance arrays in `imu.yaml`. For wheel odometry, tune `pose_covariance` and `twist_covariance` through the `flexiv_amr_wheel_odom` node parameters if the defaults are not suitable.

## Services

The driver and navigation nodes expose ROS services for AMR control. Common service names include:

- `/flexiv/amr/control/stop_motion`
- `/flexiv/amr/control/emergency_stop`
- `/flexiv/amr/navigation/translate`
- `/flexiv/amr/navigation/rotate`
- `/flexiv/amr/navigation/task_status`
- `/flexiv/amr/navigation/status`
- `/flexiv/amr/navigation/fixed_path`
- `/flexiv/amr/navigation/specified_path`
- `/flexiv/amr/navigation/pause`
- `/flexiv/amr/navigation/resume`
- `/flexiv/amr/navigation/cancel`
- `/flexiv/amr/control/reloc`
- `/flexiv/amr/control/cancel_reloc`
- `/flexiv/amr/control/location_status`

## License

This project is distributed under the [Apache License 2.0](https://opensource.org/licenses/Apache-2.0).

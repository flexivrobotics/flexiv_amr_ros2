# Flexiv AMR ROS 2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)

For ROS 2 users to easily work with Flexiv Autonomous Mobile Robots (AMR), the APIs of [`flexiv_amr_sdk`](https://github.com/zlin-flexiv/flexiv_amr_sdk) are wrapped into ROS packages in `flexiv_amr_ros2`. Key functionalities like SEER TCP communication, raw sensor publishing, open-loop motion control, navigation and relocation services, wheel odometry, IMU conversion, `robot_localization` fusion, and RViz visualization are supported.

## References

[`flexiv_amr_sdk`](https://github.com/zlin-flexiv/flexiv_amr_sdk) provides the C++ interface to the AMR controller.

[ROS 2 Humble documentation](https://docs.ros.org/en/humble/index.html) provides the ROS 2 installation and development guides.

[robot_localization](https://docs.ros.org/en/humble/p/robot_localization/) provides the EKF used to fuse wheel odometry and IMU yaw.

## Compatibility

| Supported OS | Supported ROS 2 distribution |
| ------------ | ---------------------------- |
| Ubuntu 22.04 | [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) |

This project was developed for ROS 2 Humble on Ubuntu 22.04. Other versions of Ubuntu and ROS 2 may work, but are not officially supported.

### Packages

| Package | Description |
| ------- | ----------- |
| `flexiv_amr_bringup` | Top-level launch and shared configuration, including `robot_localization`. |
| `flexiv_amr_description` | FMR-300 robot description and RViz display. |
| `flexiv_amr_driver` | Open-loop motion driver, stop-motion service, and emergency-stop service. |
| `flexiv_amr_msgs` | AMR messages and service definitions. |
| `flexiv_amr_navigation` | Navigation, relocation, and status services. |
| `flexiv_amr_sensor` | Raw AMR sensor and status publishers backed by `flexiv_amr_sdk`. |
| `flexiv_amr_imu` | Converts `flexiv_amr_msgs/msg/AmrImu` to standard `sensor_msgs/msg/Imu`. |
| `flexiv_amr_wheel_odom` | Differential-drive wheel odometry from encoder pulses. |

## Getting Started

This project requires ROS 2 Humble and a built installation of [`flexiv_amr_sdk`](https://github.com/zlin-flexiv/flexiv_amr_sdk).

1. Install [ROS 2 Humble via Debian Packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. Install `colcon` and additional ROS packages:

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
     ros-humble-robot-localization
   ```

3. Setup workspace:

   ```bash
   mkdir -p ~/flexiv_amr_ros2_ws/src
   cd ~/flexiv_amr_ros2_ws/src
   git clone https://github.com/zlin-flexiv/flexiv_amr_sdk.git
   git clone https://github.com/zlin-flexiv/flexiv_amr_ros2.git
   ```

4. Build and install `flexiv_amr_sdk`.

   Choose a directory for installing `flexiv_amr_sdk`. For example, use `~/flexiv_amr_sdk_install` as the installation prefix. Follow the SDK repository build instructions, then set:

   ```bash
   export AMR_SDK_PREFIX=~/flexiv_amr_sdk_install
   ```

5. Install ROS dependencies:

   ```bash
   cd ~/flexiv_amr_ros2_ws
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install --from-paths src --ignore-src --skip-keys flexiv_amr_sdk \
     --rosdistro humble -r -y
   ```

6. Build and source the workspace:

   ```bash
   cd ~/flexiv_amr_ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install \
     --cmake-args -DCMAKE_PREFIX_PATH="$AMR_SDK_PREFIX"
   source install/setup.bash
   ```

> [!IMPORTANT]
> Remember to source the setup files whenever a new terminal is opened:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/flexiv_amr_ros2_ws/install/setup.bash
> ```

## Usage

> [!NOTE]
> The instructions below are a quick reference for running the AMR ROS 2 stack.

The prerequisites of using ROS 2 with the AMR are enabling the SEER TCP services on the robot and establishing network connection between the workstation PC and the AMR.

Configure the AMR IP address in [`flexiv_amr_bringup/config/launch_params.yaml`](flexiv_amr_bringup/config/launch_params.yaml):

```yaml
amr_ip: 192.168.192.5
mobile_base: FMR300
```

The launch file to start the complete AMR stack is `bringup.launch.py`. It starts the AMR sensor node, driver node, navigation node, robot description, point-cloud conversion, wheel odometry, IMU conversion, EKF fusion, and RViz visualization.

### Example Commands

1. Start the complete AMR stack:

   ```bash
   ros2 launch flexiv_amr_bringup bringup.launch.py
   ```

2. Start only the robot description and RViz:

   ```bash
   ros2 launch flexiv_amr_description display.launch.py
   ```

3. Send an open-loop local motion command:

   ```bash
   ros2 topic pub --once /flexiv/amr/control/motion flexiv_amr_msgs/msg/AmrLocalMotion \
     "{vx: 0.1, vy: 0.0, w: 0.0}"
   ```

4. Stop open-loop motion:

   ```bash
   ros2 service call /flexiv/amr/control/stop_motion flexiv_amr_msgs/srv/AmrStopOpenLoopMotion "{}"
   ```

> [!WARNING]
> Open-loop motion commands may cancel the currently active navigation task. Ensure the workstation can reach the configured AMR IP and that the SEER TCP services are enabled before launching.

### Robot Description

The default robot description is the FMR-300 mobile base. The root frame is `base_link`, and the description no longer includes the Rizon arm links or meshes.

The RViz configuration is provided by:

```bash
flexiv_amr_description/rviz/flexiv_amr.default.rviz
```

### AMR States

The AMR sensor node publishes raw sensor and status feedback to ROS topics. Common topics include:

| Topic | Type | Notes |
| ----- | ---- | ----- |
| `/flexiv/amr/sensor/lidar/point` | `sensor_msgs/msg/PointCloud2` | Raw AMR lidar point cloud. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Converted from the AMR point cloud. |
| `/flexiv/amr/sensor/imu/raw` | `flexiv_amr_msgs/msg/AmrImu` | Raw AMR IMU message. |
| `/imu` | `sensor_msgs/msg/Imu` | Standard IMU message converted from the raw AMR IMU message. |
| `/flexiv/amr/sensor/encoder/raw` | `flexiv_amr_msgs/msg/AmrEncoder` | Raw encoder pulses. |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | Differential-drive wheel odometry. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF output from `/wheel_odom` and `/imu`. |
| `/flexiv/amr/control/motion` | `flexiv_amr_msgs/msg/AmrLocalMotion` | Open-loop local motion command. |

Other AMR status topics, such as battery, pose, run info, and block status, are configured in [`flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml`](flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml).

### Services

The driver and navigation nodes expose ROS services for AMR control:

| Service | Description |
| ------- | ----------- |
| `/flexiv/amr/control/stop_motion` | Stop open-loop local motion. |
| `/flexiv/amr/control/emergency_stop` | Trigger software emergency stop. |
| `/flexiv/amr/navigation/translate` | Request translation navigation. |
| `/flexiv/amr/navigation/rotate` | Request rotation navigation. |
| `/flexiv/amr/navigation/task_status` | Query navigation task status. |
| `/flexiv/amr/navigation/status` | Query navigation status. |
| `/flexiv/amr/navigation/fixed_path` | Request fixed-path navigation. |
| `/flexiv/amr/navigation/specified_path` | Request specified-path navigation. |
| `/flexiv/amr/navigation/pause` | Pause navigation. |
| `/flexiv/amr/navigation/resume` | Resume navigation. |
| `/flexiv/amr/navigation/cancel` | Cancel navigation. |
| `/flexiv/amr/control/reloc` | Request relocation. |
| `/flexiv/amr/control/cancel_reloc` | Cancel relocation. |
| `/flexiv/amr/control/location_status` | Query relocation status. |

### Configuration

| File | Purpose |
| ---- | ------- |
| [`flexiv_amr_bringup/config/launch_params.yaml`](flexiv_amr_bringup/config/launch_params.yaml) | AMR IP and robot-description selection. |
| [`flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml`](flexiv_amr_sensor/flexiv_amr_sensor/config/sensors.yaml) | Raw sensor topic names, frame IDs, publish rates, and enable flags. |
| [`flexiv_amr_sensor/flexiv_amr_wheel_odom/config/wheel_odom.yaml`](flexiv_amr_sensor/flexiv_amr_wheel_odom/config/wheel_odom.yaml) | Encoder topic, odometry topic, wheel separation, encoder scale, direction, and odometry frames. |
| [`flexiv_amr_sensor/flexiv_amr_imu/config/imu.yaml`](flexiv_amr_sensor/flexiv_amr_imu/config/imu.yaml) | Raw IMU topic, output IMU topic, raw-to-IMU axis correction, angular-velocity offset handling, and IMU covariances. |
| [`flexiv_amr_bringup/config/ekf.yaml`](flexiv_amr_bringup/config/ekf.yaml) | `robot_localization` input selection, frames, process noise, and initial estimate covariance. |

The wheel odometry node publishes `nav_msgs/msg/Odometry` on `/wheel_odom`; it does not publish TF. The EKF is responsible for the fused odometry output and TF according to [`flexiv_amr_bringup/config/ekf.yaml`](flexiv_amr_bringup/config/ekf.yaml).

For a two-wheel differential AMR, tune `left_pulses_per_meter`, `right_pulses_per_meter`, `wheel_separation`, `left_direction`, and `right_direction` in `wheel_odom.yaml`. If the raw IMU axes do not match the robot frame, tune `raw_to_imu_rpy` in `imu.yaml`.

## License

This project is distributed under the [Apache License 2.0](https://opensource.org/licenses/Apache-2.0).

# QRB ROS Robot Base Control

## Overview

QRB ROS Robot Base control package provide ROS interfaces  to control AMR robot base.

It will init robot base, sync time with MCB, provide ROS interfaces for control robot base, publish robot base state with ROS topics and provide test tools for robot base control. All the data between MCB and RBx side through [QRC protocol](https://github.com/qualcomm-qrb-ros/robot_base_qrc).

## Packages

| Name                            | Descriptions                                                                      |
| ------------------------------- | --------------------------------------------------------------------------------- |
| qrb_robot_base_manager          | The C/C++ library to control robot base. it communicates with MCB using `libqrc`. |
| qrb_ros_robot_base              | ROS 2 API to control robot base, it calls `qrb_robot_base_manager`.                   |
| qrb_ros_robot_base_msgs         | ROS 2 interfaces definition.                                                      |
| qrb_ros_robot_base_keyboard     | Keyboard command line tools to control Robot base with ROS 2 messages.             |
| qrb_ros_robot_base_urdf         | Robot base URDF model description and Rviz launch scripts.                         |

## Code Sync and Build

Currently, we only support build with QCLINUX SDK.

Download QCLINUX SDK follow this document: [download-the-prebuilt-robotics-image](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/download-the-prebuilt-robotics-image_3_1.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

Set up the cross-compile environment:

```bash
cd <qirp_decompressed_workspace>/qirp-sdk
source setup.sh
```

Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

```bash
mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
```

Clone this repository and dependencies under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

```bash
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws
git clone https://github.com/qualcomm-qrb-ros/libqrc.git
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_robot_base.git
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_interfaces.git
```

Build projects

```bash
export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

colcon build --merge-install --cmake-args \
   -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
   -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
   -DCMAKE_STAGING_PREFIX=$(pwd)/install \
   -DCMAKE_PREFIX_PATH=$(pwd)/install/share -DBUILD_TESTING=OFF \
   -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DQRC_RB3=ON
```

## Install and Run

Push to the device and install

```
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install
tar czvf qrb_ros_robot_base.tar.gz include lib share
scp qrb_ros_robot_base.tar.gz root@[ip-addr]:/opt/
ssh root@[ip-addr]
(ssh) tar -zxf /opt/qrb_ros_robot_base.tar.gz -C /opt/qcom/qirp-sdk/usr/
```

Run robot base manager

```bash
ssh root@[ip-addr]
source /opt/qcom/qirp-sdk/qirp-setup.sh
ros2 launch qrb_ros_robot_base robot_base.launch.py
```

Run keyboard test tool in another termibal

```bash
ssh root@[ip-addr]
source /opt/qcom/qirp-sdk/qirp-setup.sh
ros2 run qrb_ros_robot_base_keyboard robot_base
```
Use keyboard to control
```
---------------------------------------------------------------
Moving around:      Control mode:        Charger command:
   u    i    o       1   2   3   4       8   9   0
   j    k    l      Motion mode:         Emergency command:
   m    ,    .       5   6   7           [   ]
---------------------------------------------------------------

k : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

1 : application control
2 : charger control
3 : remote controller
4 : query current mode

5 : speed mode (for test only)
6 : motion emergency enable (for test only)
7 : motion emergency disable (for test only)

8 : start charging
9 : stop charging
0 : get battery state

[ : emergency enable
] : emergency disable

CTRL-C to quit
```

## Parameters

### Parameters Definition

| Name                          | Value Description | Default Value       |
| ----------------------------- | ----------------- | ------------------- |
| car_wheel_perimeter           | float (m)         | 0.4115              |
| car_wheel_space               | float (m)         | 0.3302              |
| imu_enable                    | bool              | false               |
| motion_max_speed              | float (m/s)       | 1.0                 |
| motion_max_angle_speed        | float (rad/s)     | 1.5                 |
| motion_pid_speed              | float[3]          | [400.0, 200.0, 0.0] |
| motion_odom_frequency         | 1-50              | 50                  |
| motion_tf_enable              | bool              | false               |
| rc_enable                     | bool              | true                |
| rc_max_speed                  | float (m/s)       | 0.8                 |
| rc_max_angle_speed            | float (rad/s)     | 2.0                 |
| scale_speed                   | float             | 1.0                 |
| scale_speed_odom              | float             | 1.0                 |
| ultra_enable                  | bool              | true                |
| ultra_quantity                | uint              | 7                   |
| oba_bottom_distance           | float (m)         | 0.05                |
| oba_front_distance            | float (m)         | 0.15                |
| oba_side_distance             | float (m)         | 0.15                |
| test_transport_latency_enable | bool              | false               |
| time_sync_interval_sec        | unsigned int (s)  | 300                 |
| time_sync_threshold_ms        | unsigned int (ms) | 10                  |

**❗ Attention**

- rc_max_speed need <= motion_max_speed
- rc_max_angle_speed need <= motion_max_angle_speed

### Robot Models

Change robot model with environment variable:

```bash
export ROBOT_BASE_MODEL=robot_base_mini
```

Current support robot models:

- Standard Robot base: `robot_base` (default)
- Circle Robot base: `robot_base_mini`

### Parameters Configuration

Change /opt/qcom/qirp-sdk/usr/share/qrb_ros_robot_base/config/robot_base.yaml

- Standard Robot base: `robot_base.yaml` (default)
- Circle Robot base: `robot_base_mini.yaml`

```bash
/qrb_robot_base_manager:
  ros__parameters:
    ultra_enable: true
    rc_enable: true
    motion_tf_enable: false
    # other parameters
```
Relaunch ROS node

```bash
ros2 launch qrb_ros_robot_base robot_base.launch.py
```

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcom RB3 gen2.

| Hardware          | Software          |
| ----------------- | ----------------- |
| Qualcomm RB3 gen2 | LE.QCROBOTICS.1.0 |

## Contributions

Thanks for your interest in contributing to qrb_ros_robot_base ! Please read our [Contributions Page](https://github.com/qualcomm-qrb-ros/qrb_ros_robot_base/blob/main/CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_robot_base is licensed under the BSD-3-clause "New" or "Revised" License.

Check out the [LICENSE](https://github.com/qualcomm-qrb-ros/qrb_ros_robot_base/blob/main/LICENSE) for more details.

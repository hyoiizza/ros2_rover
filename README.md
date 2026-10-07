# ros2_rover

This is a ROS 2 (Humble) version of the [Sawppy the Rover](https://github.com/Roger-random/Sawppy_Rover). A [C++](./rover_motor_controller_cpp) version and a [Python](./rover_motor_controller) version of the lx16a controller are included. Manual driving uses a [keyboard teleop node](./rover_teleop/rover_teleop/teleop_keyboard_node.py). The following sensors are used: a [Slamtec RPLIDAR C1](./rover_bringup/launch/rplidar.launch.py) 2D lidar, an [Orbbec Gemini 335](./rover_bringup/launch/gemini_335.launch.py) RGB-D camera and an [Adafruit BNO085](./rover_bringup/launch/bno085.launch.py) 9-DOF IMU.

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/ros2_rover.svg)](https://github.com/mgonzs13/ros2_rover/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/ros2_rover.svg?branch=humble)](https://github.com/mgonzs13/ros2_rover?branch=humble) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/ros2_rover.svg)](https://github.com/mgonzs13/ros2_rover/commits/maihumblen) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/ros2_rover)](https://github.com/mgonzs13/ros2_rover/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/ros2_rover)](https://github.com/mgonzs13/ros2_rover/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/ros2_rover.svg)](https://github.com/mgonzs13/ros2_rover/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/ros2_rover/actions/workflows/python-formatter.yml/badge.svg?branch=humble)](https://github.com/mgonzs13/ros2_rover/actions/workflows/python-formatter.yml?branch=humble) [![C++ Formatter Check](https://github.com/mgonzs13/ros2_rover/actions/workflows/cpp-formatter.yml/badge.svg?branch=humble)](https://github.com/mgonzs13/ros2_rover/actions/workflows/cpp-formatter.yml?branch=humble)

| ROS 2 Distro |                             Branch                             |                                                                                                        Build status                                                                                                         |                                                             Docker Image                                                             | Documentation                                                                                                                                                    |
| :----------: | :------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
|  **Humble**  | [`humble`](https://github.com/mgonzs13/ros2_rover/tree/humble) | [![Humble Build](https://github.com/mgonzs13/ros2_rover/actions/workflows/humble-docker-build.yml/badge.svg?branch=humble)](https://github.com/mgonzs13/ros2_rover/actions/workflows/humble-docker-build.yml?branch=humble) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/rover/tags?name=humble) | [![Doxygen Deployment](https://github.com/mgonzs13/ros2_rover/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/ros2_rover/latest) |
|   **Iron**   | [`humble`](https://github.com/mgonzs13/ros2_rover/tree/humble) |    [![Iron Build](https://github.com/mgonzs13/ros2_rover/actions/workflows/iron-docker-build.yml/badge.svg?branch=humble)](https://github.com/mgonzs13/ros2_rover/actions/workflows/iron-docker-build.yml?branch=humble)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/rover/tags?name=iron)   | [![Doxygen Deployment](https://github.com/mgonzs13/ros2_rover/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/ros2_rover/latest) |

</div>

## Table of Contents

1. [Installation](#installation)
2. [Usage](#usage)
   - [Linux Service](#linux-service)
3. [Docker](#docker)
4. [Gazebo Simulation](#gazebo-simulation)
5. [Citations](#citations)

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/ros2_rover
cd ~/ros2_ws
rosdep install --from-paths src -r -y
colcon build
```

## Docker

You can create a docker image to test this repo. Use the following common inside the directory of ros2_rover.

```shell
docker build -t rover .
```

After the image is created, run a docker container with the following command.

```shell
docker run -it --rm rover
```

## Usage

<div align="center">
    <img src="docs/rover.png" width="75%"/>
</div>

```shell
ros2 launch rover_bringup rover.launch.py
```

### Mapping and Navigation

Driving runs entirely on the RPLIDAR C1. The Gemini 335 is reserved for the
RGB-D vision map and is not in the navigation loop.

Odometry comes from `rf2o_laser_odometry` (scan matching) fused with the BNO085
in the EKF, because the LX-16A servos provide no usable wheel feedback.

Build a map:

```shell
ros2 launch rover_bringup rover.launch.py
ros2 launch rover_localization localization.launch.py          # slam:=mapping is the default
ros2 run rover_teleop teleop_keyboard_node                     # drive it around
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

Navigate that map afterwards:

```shell
ros2 launch rover_bringup rover.launch.py
ros2 launch rover_localization localization.launch.py slam:=off
ros2 launch rover_navigation bringup.launch.py map:=$HOME/maps/my_map.yaml
```

Add `use_vision_map:=True` to the localization launch to also run rtabmap on the
RGB-D camera. It publishes to `/rtabmap/map` and does not touch TF.

In simulation both flows are driven from one launch file:

```shell
ros2 launch rover_gazebo moon.launch.py                        # maps live
ros2 launch rover_gazebo moon.launch.py map:=$HOME/maps/my_map.yaml
```

### Linux Service

A Linux service can be created to control the execution and launch everything at boot time. To create the rover service, the following commands are used:

```shell
cd ~/ros2_ws/src/ros2_rover/rover_service
sudo ./install.sh
```

Check rover service:

```shell
sudo service rover status
```

## Gazebo Simulation

### Moon

```shell
ros2 launch rover_gazebo moon.launch.py
```

<div>
    <img src="docs/moon.png" width="100%"/>
</div>

### Mars

```shell
ros2 launch rover_gazebo mars.launch.py
```

<div>
    <img src="docs/mars.png" width="100%"/>
</div>

### Forest

```shell
ros2 launch rover_gazebo forest.launch.py
```

<div>
    <img src="docs/forest.png" width="100%"/>
</div>

## Citations

The `v0.7` version has been used in the work `Comparison of Concentric Surface Planetary Explorations Using an Ackerman Rover in a Lunar Simulation` from the `2024 International Conference on Space Robotics (iSpaRo)`.

```bibtex
@INPROCEEDINGS{10685849,
  author={González-Santamarta, Miguel Á. and Rodríguez-Lera, Francisco J.},
  booktitle={2024 International Conference on Space Robotics (iSpaRo)},
  title={Comparison of Concentric Surface Planetary Explorations Using an Ackerman Rover in a Lunar Simulation},
  year={2024},
  volume={},
  number={},
  pages={239-244},
  keywords={Spirals;Navigation;Moon;Cameras;Sensors;Visual odometry;Testing},
  doi={10.1109/iSpaRo60631.2024.10685849}
}
```

# TurtleBot4 Search and Follow - IRN Project

This repository contains a ROS 2 real-robot pipeline for TurtleBot4 search,
ArUco marker detection, target pose estimation, and predictive following with
Nav2.

Developed for the Intelligent Robotics and Navigation course, the project
combines computer vision, waypoint-based exploration, Behavior Trees, and
navigation recovery logic to search for a target marker and switch to a
dedicated following behavior once the target is detected.

## Project Overview

The system is designed for a TurtleBot4 equipped with an OAK-D camera. During
the search phase, the robot visits an offline-optimized waypoint route. The
ArUco detector runs in parallel and publishes marker poses when the target is
visible. Once the marker is detected, the follow pipeline transforms the marker
pose into the map frame, computes a suitable robot goal, and uses Nav2 to
approach the target. If the marker is temporarily lost, a Kalman-based tracker
publishes a predicted goal and the finite state machine manages recovery.

## Main Features

- ArUco marker detection from compressed or raw OAK-D RGB images.
- Camera-based pose estimation with OpenCV and ROS 2 `cv_bridge`.
- TF transformation from camera frame to map frame.
- Predictive target tracking with a Kalman filter.
- Nav2 goal generation for visible and predicted target poses.
- Finite state machine for target following, lost-target behavior, and
  reacquisition.
- Waypoint search mission with offline MILP route optimization.
- Behavior Tree XML files for search and follow navigation.
- Real-robot launch files for localization, Nav2, RViz, search, and follow.

## Repository Structure

```text
irn_g01/        ROS 2 package for ArUco detection, pose estimation, and follow FSM
tb4_project/    ROS 2 package with launch files, Nav2 configs, BTs, and missions
doc_irn.pdf     Project report
```

## Requirements

- ROS 2 with `ament_python`
- TurtleBot4 stack
- Nav2
- OpenCV with ArUco support
- Python dependencies listed in `irn_g01/requirements.txt`
- ROS dependencies installable through `rosdep`

## Installation

From a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repo-url> progetto_irn
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r src/progetto_irn/irn_g01/requirements.txt
colcon build --packages-select irn_g01 tb4_project
source install/setup.bash
```

## Usage

Launch the complete real-robot pipeline:

```bash
ros2 launch tb4_project full_real.launch.py
```

Launch only the waypoint search mission:

```bash
ros2 launch tb4_project real_search_mission.launch.py
```

Launch only the ArUco follow pipeline:

```bash
ros2 launch tb4_project follow.launch.py
```

Launch only the ArUco detector:

```bash
ros2 launch tb4_project aruco_reader.launch.py
```


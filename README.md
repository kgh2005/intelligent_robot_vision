# Intelligent Robot Vision 2025 Package

This package is part of the Intelligent Robot High-Tech Vision System, running on **Ubuntu 22.04** and **ROS 2 Humble**.  
It is designed to work with the **Intel RealSense D435** camera.

## Nodes

The package includes the following nodes:
- **realsense_node**: Captures RGB and depth data from the RealSense D435.
- **detection_node**: Runs object detection using a YOLO model.
- **refiner_node**: Refines detection results and performs additional processing.
- **pantilt_node**: Controls the pan-tilt mechanism for the camera.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Intel RealSense D435 with `realsense2_camera` ROS 2 package installed
- YOLO model files (placed in the `model/` directory)

## Build

```bash
colcon build --packages-select intelligent_robot_vision
```

## Setup Environment

```bash
source ~/.bashrc
```

Make sure your ROS 2 workspace setup is also sourced (if not already in `~/.bashrc`):

```bash
source ~/colcon_ws/install/setup.bash
```

## Run

```bash
ros2 launch intelligent_robot_vision intelligent_robot_vision.launch.py
```

This will start:
1. The RealSense D435 camera node
2. The object detection node
3. The refiner node
4. The pan-tilt control node

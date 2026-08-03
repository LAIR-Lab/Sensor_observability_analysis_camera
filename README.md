## Work in progress

# Sensor_observability_analysis_camera
An open-source ROS 2 implementation of Sensor Observability Analysis and demo for USB cameras viewpoints through observability metrics and Jacobian-based motion planning.

## Features

- SOA metric computation
- Camera Jacobian calculation
- TF-based camera tracking
- Gradient ascent optimization
- Gazebo simulation

## Requirements

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- Python 3.12

## Build
Build the latest version via:
```bash

cd ~/SOA
colcon build
source install/setup.bash
```
## Usage
The usage of this package on a demo so-101 robot requires 1 launch file and 2 ros2 nodes in required order.

1. Run the `gazebo.launch.py` launch file in `digital_twin` to generate the so-101 robot in rviz2 and gazebo. Initialize ros2-gazebo plugins,controllers.
```bash
ros2 launch digital_twin gazebo.launch.py
```

2. Run the `soa_camera_jacobian_node` launch file next to calculate the SOA-jacobian for each joints.
```bash
ros2 run sensor_observability_analysis_py soa_camera_jacobian_node
```

3. Finally run `` launch file to use the controller which utilizes the SOA-jacobian values to move the robot to obtain higher
```bash
ros2 run sensor_observability_analysis_py soa_cam_move_node
```

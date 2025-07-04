# My SLAM Robot Project

This project provides a ROS 2-based SLAM (Simultaneous Localization and Mapping) system for a mobile robot, integrating simulation, mapping, object detection, and visualization tools.

## Features
- **Gazebo Simulation**: Launches a simulated world with a custom robot model.
- **Robot State Publisher**: Publishes robot state using URDF/Xacro.
- **Cartographer SLAM**: 2D SLAM using Cartographer ROS for real-time mapping and localization.
- **YOLOv8 Object Detection**: Real-time object detection using YOLOv8 and annotated image publishing.
- **Sensor Bridges**: ROS-Gazebo bridge for sensor topics (camera, lidar, IMU, odometry).
- **TF Management**: Custom TF connector and static transforms.
- **Visualization**: RViz2 configuration for SLAM and object detection overlays.

## Directory Structure
```
src/my_slam/
├── CMakeLists.txt
├── package.xml
├── configs/           # Cartographer and robot configs
├── launch/            # Launch files for simulation and SLAM
├── models/            # Gazebo models (robot, ground plane)
├── rviz/              # RViz configuration files
├── scripts/           # Python scripts (YOLO, scan fixer, TF connector, etc.)
├── urdf/              # Robot description (URDF/Xacro)
├── worlds/            # Gazebo world files
```

## Main Launch Files
- `my_slam.launch.py`: Launches simulation, robot, bridges, YOLO, and RViz.
- `cartographer.launch.py`: Launches Cartographer SLAM and occupancy grid mapping.

## Key Scripts
- `yoloV8.py`: YOLOv8 object detection node.
- `fix_scan_frame.py`: Fixes scan frame for Cartographer.
- `tf_connector.py`: Custom TF management.

## Usage
1. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
2. **Launch simulation and SLAM:**
   ```bash
   ros2 launch my_slam my_slam.launch.py
   # or for SLAM only
   ros2 launch my_slam cartographer.launch.py
   ```
3. **View results in RViz2.**

## Requirements
- ROS 2 (Foxy, Galactic, or Humble recommended)
- Gazebo (Ignition or Fortress)
- Python dependencies: `ultralytics`, `opencv-python`, `cv_bridge`, etc.

## Notes
- The YOLOv8 model file (`yolov8n.pt`) should be present in the `scripts/` directory.
- Adjust `use_sim_time` as needed for simulation or real robot.
- For GPU acceleration, set the YOLO node's `device` parameter to `cuda:0`.

## License
MIT License

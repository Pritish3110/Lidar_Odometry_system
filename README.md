# LiDAR Odometry System

> Real-time LiDAR-only odometry pipeline built with ROS2 Humble using ICP and Probabilistic Kernel Optimization

---

## Table of Contents

1. [Introduction & Motivation](#introduction--motivation)
2. [System Architecture Overview](#system-architecture-overview)
3. [Key Features](#key-features)
4. [Technology Stack](#technology-stack)
5. [ROS2 Workspace & Folder Structure](#ros2-workspace--folder-structure)
6. [Dataset Description](#dataset-description)
7. [Installation Prerequisites](#installation-prerequisites)
8. [Step-by-Step Installation](#step-by-step-installation)
9. [Build Instructions](#build-instructions)
10. [Environment Setup](#environment-setup)
11. [Dataset Conversion to ROS2 Bag](#dataset-conversion-to-ros2-bag)
12. [Running the LiDAR Odometry System](#running-the-lidar-odometry-system)
13. [RViz Visualization Guide](#rviz-visualization-guide)
14. [Algorithm Explanation](#algorithm-explanation)
15. [Performance Notes](#performance-notes)
16. [Common Errors & Troubleshooting](#common-errors--troubleshooting)
17. [Future Improvements](#future-improvements)
18. [License](#license)
19. [Acknowledgements](#acknowledgements)
20. [Author Information](#author-information)

---

## Introduction & Motivation

This project implements a complete **LiDAR-only odometry pipeline** in ROS2 Humble that estimates vehicle motion using consecutive 3D LiDAR scans. Unlike traditional approaches that rely on GPS or IMU sensors, this system uses **feature-based scan matching** and **probabilistic optimization** to compute accurate pose estimates in real-time.

The system has been successfully tested using the **KITTI Odometry Dataset (Sequence 07)** with Velodyne HDL-64E LiDAR scans, demonstrating robust performance in urban driving scenarios.

---

## System Architecture Overview

The data flows through the system in the following sequence:

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   KITTI .bin    │────▶│  ROS2 Bag File   │────▶│  PointCloud2    │
│   LiDAR Scans   │     │  (.db3 format)   │     │  Publisher      │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   RViz2         │◀────│  TF Broadcaster  │◀────│  Feature        │
│   Visualization │     │  map→odom→base   │     │  Extraction     │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                          │
                        ┌──────────────────┐              ▼
                        │  nav_msgs/       │     ┌─────────────────┐
                        │  Odometry        │◀────│  ICP + PKO      │
                        └──────────────────┘     │  Registration   │
                                                 └─────────────────┘
```

1. **Data Input**: KITTI `.bin` scans are converted to a ROS2 bag file
2. **Point Cloud Publishing**: Scans are published as `sensor_msgs/PointCloud2` on `/velodyne_points`
3. **Feature Extraction**: Edge and planar features are extracted from the raw point cloud
4. **ICP Registration**: Consecutive scans are aligned using Iterative Closest Point
5. **PKO Optimization**: Probabilistic Kernel Optimization refines the pose estimates using Ceres Solver
6. **Odometry Output**: Pose is published as `nav_msgs/Odometry`
7. **TF Broadcasting**: Transform tree is broadcast: `map → odom → base_link`
8. **Visualization**: Trajectory and point clouds are displayed in RViz2

---

## Key Features

- ✅ **GPS/IMU-free odometry** — Pure LiDAR-based motion estimation
- ✅ **Feature-based matching** — Extracts edge and planar features for robust registration
- ✅ **ICP + PKO pipeline** — Combines scan matching with probabilistic optimization
- ✅ **Real-time performance** — Processes scans at sensor frame rate
- ✅ **ROS2 Humble compatible** — Native ROS2 integration with standard messages
- ✅ **Loop closure ready** — LiDAR Iris support included in codebase
- ✅ **Ceres Solver integration** — Robust nonlinear optimization backend

---

## Technology Stack

| Component | Technology |
|-----------|------------|
| Middleware | ROS2 Humble (built from source) |
| Language | C++ |
| Linear Algebra | Eigen3 |
| Point Cloud Processing | PCL (Point Cloud Library) |
| Optimization | Ceres Solver |
| Transforms | TF2 |
| Visualization | RViz2 |
| Build System | colcon |

---

## ROS2 Workspace & Folder Structure

```
~/lidar_odom_ws/
├── src/
│   └── lidar_odometry_ros_wrapper/
│       ├── lidar_odometry_ros/
│       │   ├── config/
│       │   │   └── kitti.yaml
│       │   ├── launch/
│       │   │   └── lidar_odometry.launch.py
│       │   ├── src/
│       │   │   ├── lidar_odometry_node.cpp
│       │   │   ├── feature_extraction.cpp
│       │   │   ├── icp_registration.cpp
│       │   │   └── pko_optimizer.cpp
│       │   ├── include/
│       │   │   ├── lidar_odometry_node.hpp
│       │   │   ├── feature_extraction.hpp
│       │   │   ├── icp_registration.hpp
│       │   │   └── pko_optimizer.hpp
│       │   ├── package.xml
│       │   └── CMakeLists.txt
│       └── lidar_iris/
│           ├── loop_closure.cpp
│           └── package.xml
├── install/
│   ├── setup.bash
│   └── local_setup.bash
├── build/
└── log/
```

**KITTI Dataset Folder:**

```
~/kitti_data/
├── kitti_seq07.db3          # Converted ROS2 bag file
├── kitti_to_rosbag.py       # Conversion script
└── sequences/
    └── 07/
        └── velodyne/
            ├── 000000.bin
            ├── 000001.bin
            └── ...
```

---

## Dataset Description

This project uses the **KITTI Odometry Dataset**, a widely-used benchmark for visual and LiDAR odometry research.

| Property | Value |
|----------|-------|
| Sequence | 07 |
| Sensor | Velodyne HDL-64E |
| Format | Raw `.bin` point cloud files |
| Environment | Urban driving scenario |
| Points per scan | ~120,000 |

**Download**: [KITTI Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

---

## Installation Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill (built from source recommended)
- **CMake**: 3.16 or higher
- **Compiler**: GCC 11+
- **Dependencies**: Eigen3, PCL, Ceres Solver

### Install System Dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y libeigen3-dev
sudo apt install -y libpcl-dev
sudo apt install -y libceres-dev
```

---

## Step-by-Step Installation

### 1. Create the ROS2 Workspace

```bash
mkdir -p ~/lidar_odom_ws/src
cd ~/lidar_odom_ws/src
```

### 2. Clone the Repository

```bash
git clone <repository-url> lidar_odometry_ros_wrapper
```

### 3. Install ROS2 Dependencies

```bash
cd ~/lidar_odom_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Build Instructions

Build the workspace using colcon:

```bash
cd ~/lidar_odom_ws
colcon build --symlink-install
```

> **Note**: The `--symlink-install` flag creates symbolic links, allowing you to modify Python files without rebuilding.

### Build Specific Package Only

```bash
colcon build --packages-select lidar_odometry_ros --symlink-install
```

---

## Environment Setup

Source the ROS2 and workspace setup files:

```bash
source ~/ros2_humble/install/setup.bash
source ~/lidar_odom_ws/install/setup.bash
```

### Add to ~/.bashrc for Automatic Sourcing

```bash
echo 'source ~/ros2_humble/install/setup.bash' >> ~/.bashrc
echo 'source ~/lidar_odom_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## Dataset Conversion to ROS2 Bag

The KITTI dataset provides raw `.bin` LiDAR scans that must be converted to a ROS2 bag file for playback.

### Conversion Process

The conversion script (`kitti_to_rosbag.py`) performs the following:

1. Reads each binary `.bin` file from the KITTI sequence
2. Parses the XYZI (X, Y, Z, Intensity) point data
3. Converts to `sensor_msgs/PointCloud2` format
4. Writes to a ROS2 bag file (`.db3` format)

### Run Conversion

```bash
python3 kitti_to_rosbag.py \
  --input ~/kitti_data/sequences/07/velodyne \
  --output ~/kitti_data/kitti_seq07.db3 \
  --topic /velodyne_points
```

---

## Running the LiDAR Odometry System

### Terminal 1: Launch the Odometry Node

```bash
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
  config_file:=$(ros2 pkg prefix lidar_odometry_ros)/share/lidar_odometry_ros/config/kitti.yaml \
  use_sim_time:=true \
  pointcloud_topic:=/velodyne_points
```

### Terminal 2: Play the Dataset

```bash
ros2 bag play ~/kitti_data/kitti_seq07.db3 --clock
```

> **Important**: The `--clock` flag publishes simulated time, which is required when using `use_sim_time:=true`.

---

## RViz Visualization Guide

### Launch RViz2

```bash
rviz2
```

### Configure Displays

| Display Type | Configuration |
|--------------|---------------|
| **Fixed Frame** | Set to `map` |
| **PointCloud2** | Topic: `/velodyne_points` |
| **Odometry** | Topic: `/odometry` |
| **TF** | Enable to visualize transform tree |
| **Path** | Topic: `/trajectory` |

### Recommended Settings

- **Point Size**: 0.03 for better visibility
- **Decay Time**: 0 for PointCloud2 (show current scan only)
- **Color Transformer**: Intensity or AxisColor

---

## Algorithm Explanation

### Iterative Closest Point (ICP)

ICP is a scan matching algorithm that aligns two point clouds by iteratively finding correspondences and minimizing the distance between them.

**Algorithm Steps:**

1. **Correspondence Search**: For each point in the source cloud, find the nearest neighbor in the target cloud
2. **Transformation Estimation**: Compute the optimal rigid transformation (rotation + translation) using SVD
3. **Apply Transformation**: Transform the source cloud using the estimated pose
4. **Convergence Check**: Repeat until the change in error is below a threshold

**Mathematical Formulation:**

```
minimize Σ ||R·pᵢ + t - qᵢ||²
```

Where:
- `R` = Rotation matrix
- `t` = Translation vector
- `pᵢ` = Source points
- `qᵢ` = Corresponding target points

### Probabilistic Kernel Optimization (PKO)

PKO improves upon basic ICP by treating pose estimation as a probabilistic inference problem.

**Key Advantages:**

- Uses **kernel-based soft assignments** instead of hard correspondences
- More robust to **outliers** and **partial overlaps**
- Handles **non-Gaussian noise** distributions
- Leverages **Ceres Solver** for efficient nonlinear optimization

**Optimization Objective:**

```
maximize Σ K(||R·pᵢ + t - qⱼ||)
```

Where `K` is a kernel function (e.g., Gaussian kernel) that provides soft weighting.

---

## Performance Notes

| Metric | Value |
|--------|-------|
| Processing Rate | Real-time at sensor frame rate |
| Memory Usage | Scales with point cloud density |
| GPU Requirement | Not required (CPU-only) |
| Accuracy | Comparable to state-of-the-art methods |

### Optimization Tips

- Reduce point cloud density using voxel grid filtering for faster processing
- Adjust the number of ICP iterations in `kitti.yaml`
- Use multi-threaded execution where supported

---

## Common Errors & Troubleshooting

### TF Timeout Warnings

**Error**: `Could not transform...` or TF timeout warnings

**Solution**: Ensure `use_sim_time:=true` is set and the bag is playing with `--clock`

```bash
ros2 bag play ~/kitti_data/kitti_seq07.db3 --clock
```

---

### No Point Cloud in RViz

**Problem**: RViz shows empty visualization

**Solutions**:
1. Check the fixed frame is set to `map`
2. Verify the topic name matches `/velodyne_points`
3. Ensure the bag is actively playing

```bash
ros2 topic list
ros2 topic echo /velodyne_points --no-arr
```

---

### Build Errors with PCL

**Error**: CMake cannot find PCL

**Solution**: Ensure PCL 1.12+ is installed

```bash
sudo apt install libpcl-dev
pkg-config --modversion pcl_common
```

---

### Ceres Solver Not Found

**Error**: `Could not find Ceres`

**Solution**: Install Ceres Solver

```bash
sudo apt install libceres-dev
```

---

## Future Improvements

- [ ] Enable loop closure with LiDAR Iris integration
- [ ] Add ground truth comparison for KITTI sequences
- [ ] Implement multi-threaded feature extraction
- [ ] Support for other LiDAR sensors (Ouster, Livox)
- [ ] Integration with mapping frameworks (SLAM)
- [ ] GPU-accelerated ICP using CUDA
- [ ] Docker container for easy deployment

---

## Acknowledgements

- **KITTI Vision Benchmark Suite** for providing the odometry dataset
- **ROS2 Community** for the robotics middleware
- **PCL Developers** for the Point Cloud Library
- **Ceres Solver Team** for the nonlinear optimization library
- **Open-source LiDAR Odometry Community** for research and inspiration

---

## Author Information

Developed as a ROS2 Humble implementation of feature-based LiDAR odometry.

For questions, bug reports, or contributions:
- Open an **Issue** on GitHub
- Submit a **Pull Request**

---

**⭐ If this project helped you, please consider giving it a star!**

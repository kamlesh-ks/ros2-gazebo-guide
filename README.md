# 🤖 ROS2 Gazebo Simulation Guide

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)
![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)
[![Documentation](https://img.shields.io/badge/docs-online-blue)](https://kamlesh-ks.github.io/ros2-gazebo-guide/)
[![CI](https://github.com/kamlesh-ks/ros2-gazebo-guide/actions/workflows/deploy.yml/badge.svg)](https://github.com/kamlesh-ks/ros2-gazebo-guide/actions)

**Complete guide for setting up ROS2 Humble with Ignition Gazebo Fortress**

[📖 Documentation](https://kamlesh-ks.github.io/ros2-gazebo-guide/) •
[🐛 Report Bug](https://github.com/kamlesh-ks/ros2-gazebo-guide/issues) •
[✨ Request Feature](https://github.com/kamlesh-ks/ros2-gazebo-guide/issues)

</div>

---

## 📋 Overview

This repository contains a comprehensive guide and example code for setting up a complete robot simulation environment with:

- **ROS2 Humble** - Latest LTS release
- **Ignition Gazebo Fortress** - Modern robotics simulator
- **Navigation2** - Autonomous navigation
- **SLAM Toolbox** - Mapping capabilities

### Features

- ✅ Differential drive robot with URDF/Xacro
- ✅ Multiple sensors (LiDAR, Camera, IMU, Odometry)
- ✅ Complete Gazebo world with obstacles
- ✅ ROS-Gazebo bridge configuration
- ✅ SLAM mapping workflow
- ✅ Navigation2 integration
- ✅ Ready-to-use launch files
- ✅ Comprehensive documentation

---

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Ignition Gazebo Fortress

### Installation

```bash
# Clone the repository
git clone https://github.com/kamlesh-ks/ros2-gazebo-guide.git
cd ros2-gazebo-guide

# Install dependencies
sudo apt install ros-humble-ros-gz ros-humble-navigation2 ros-humble-slam-toolbox

# Build
colcon build
source install/setup.bash
```

### Run Simulation

```bash
# Launch with SLAM (for mapping)
ros2 launch my_robot_sim full_simulation.launch.py slam:=true

# Drive the robot (in another terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Launch with Navigation (using saved map)
ros2 launch my_robot_sim full_simulation.launch.py nav:=true
```

---

## 📚 Documentation

Full documentation is available at: **[https://kamlesh-ks.github.io/ros2-gazebo-guide/](https://kamlesh-ks.github.io/ros2-gazebo-guide/)**

### Documentation Sections

| Section                                              | Description                  |
| ---------------------------------------------------- | ---------------------------- |
| [Getting Started](docs/getting-started/overview.md)  | Installation and first steps |
| [Robot Setup](docs/robot/urdf-basics.md)             | URDF, sensors, plugins       |
| [Simulation](docs/simulation/world-creation.md)      | World, bridge, launch files  |
| [SLAM & Mapping](docs/slam/slam-toolbox.md)          | Create maps                  |
| [Navigation](docs/navigation/nav2-setup.md)          | Autonomous navigation        |
| [Troubleshooting](docs/reference/troubleshooting.md) | Common issues                |

---

## 📁 Repository Structure

```
ros2-gazebo-guide/
├── .github/
│   └── workflows/
│       └── deploy.yml          # GitHub Actions for docs deployment
├── docs/                        # Documentation source
│   ├── index.md                # Landing page
│   ├── getting-started/        # Setup guides
│   ├── robot/                  # Robot configuration
│   ├── simulation/             # Simulation setup
│   ├── slam/                   # SLAM guides
│   ├── navigation/             # Navigation guides
│   ├── reference/              # Reference materials
│   └── guides/                 # Full comprehensive guides
├── src/
│   └── my_robot_sim/           # Example ROS2 package
│       ├── urdf/
│       ├── worlds/
│       ├── launch/
│       ├── config/
│       └── maps/
├── mkdocs.yml                  # MkDocs configuration
├── requirements-docs.txt       # Python dependencies for docs
└── README.md
```

---

## 🛠️ Development

### Building Documentation Locally

```bash
# Install dependencies
pip install -r requirements-docs.txt

# Serve documentation locally
mkdocs serve

# Build static site
mkdocs build
```

Documentation will be available at `http://127.0.0.1:8000/`

---

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2](https://navigation.ros.org/)
- [Gazebo Sim](https://gazebosim.org/)
- [MkDocs Material](https://squidfunk.github.io/mkdocs-material/)

---

# Complete Guide: ROS2 Humble + Ignition Gazebo Fortress
## Setting Up Simulation Environment for Navigation

---

## Table of Contents

1. [Overview & Architecture](#1-overview--architecture)
2. [Prerequisites & Installation](#2-prerequisites--installation)
3. [Understanding Ignition Gazebo Fortress](#3-understanding-ignition-gazebo-fortress)
4. [Creating the Robot Model (URDF/SDF)](#4-creating-the-robot-model-urdfsdf)
5. [Adding Sensors](#5-adding-sensors)
6. [Differential Drive Setup](#6-differential-drive-setup)
7. [Creating the Gazebo World](#7-creating-the-gazebo-world)
8. [ROS2-Gazebo Bridge Configuration](#8-ros2-gazebo-bridge-configuration)
9. [Map Creation with SLAM](#9-map-creation-with-slam)
10. [Navigation Setup](#10-navigation-setup)
11. [Complete Launch Files](#11-complete-launch-files)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Overview & Architecture

### 1.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        COMPLETE SIMULATION STACK                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         ROS2 HUMBLE                                  │   │
│  │                                                                      │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐    │   │
│  │  │Navigation│  │   SLAM   │  │   RViz   │  │  Your Nodes      │    │   │
│  │  │  Stack   │  │Toolbox/  │  │          │  │  (footprint etc) │    │   │
│  │  │  (Nav2)  │  │Cartograph│  │          │  │                  │    │   │
│  │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘    │   │
│  │       │             │             │                  │              │   │
│  │       └─────────────┴─────────────┴──────────────────┘              │   │
│  │                              │                                       │   │
│  │                    ROS2 Topics/Services                              │   │
│  │                              │                                       │   │
│  └──────────────────────────────┼───────────────────────────────────────┘   │
│                                 │                                           │
│                    ┌────────────┴────────────┐                              │
│                    │     ros_gz_bridge       │  ← Translates messages       │
│                    │  (ros_ign_bridge)       │                              │
│                    └────────────┬────────────┘                              │
│                                 │                                           │
│                    Ignition Transport Topics                                │
│                                 │                                           │
│  ┌──────────────────────────────┼───────────────────────────────────────┐   │
│  │              IGNITION GAZEBO FORTRESS                                │   │
│  │                              │                                       │   │
│  │  ┌───────────────────────────┴───────────────────────────────────┐  │   │
│  │  │                      Physics Engine                           │  │   │
│  │  │                    (DART / Bullet)                            │  │   │
│  │  └───────────────────────────────────────────────────────────────┘  │   │
│  │                                                                      │   │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐    │   │
│  │  │   Robot    │  │   LiDAR    │  │   Camera   │  │    IMU     │    │   │
│  │  │  (Model)   │  │  (Sensor)  │  │  (Sensor)  │  │  (Sensor)  │    │   │
│  │  └────────────┘  └────────────┘  └────────────┘  └────────────┘    │   │
│  │                                                                      │   │
│  │  ┌───────────────────────────────────────────────────────────────┐  │   │
│  │  │                         WORLD                                 │  │   │
│  │  │              (walls, obstacles, ground)                       │  │   │
│  │  └───────────────────────────────────────────────────────────────┘  │   │
│  │                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DATA FLOW                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  SENSORS (Gazebo → ROS2):                                                   │
│  ─────────────────────────                                                  │
│                                                                             │
│  LiDAR ─────► /scan (sensor_msgs/LaserScan)                                 │
│  Camera ────► /camera/image_raw (sensor_msgs/Image)                         │
│  IMU ───────► /imu (sensor_msgs/Imu)                                        │
│  Odom ──────► /odom (nav_msgs/Odometry)                                     │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  CONTROL (ROS2 → Gazebo):                                                   │
│  ────────────────────────                                                   │
│                                                                             │
│  /cmd_vel (geometry_msgs/Twist) ─────► Diff Drive Controller                │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  TF TREE:                                                                   │
│  ────────                                                                   │
│                                                                             │
│  map ──► odom ──► base_link ──┬──► laser_frame                              │
│                               ├──► camera_link                              │
│                               ├──► imu_link                                 │
│                               ├──► left_wheel                               │
│                               └──► right_wheel                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Prerequisites & Installation

### 2.1 System Requirements

```bash
# Ubuntu 22.04 (required for ROS2 Humble)
# At least 8GB RAM recommended
# GPU with OpenGL 3.3+ support for Gazebo rendering
```

### 2.2 Install ROS2 Humble (if not already installed)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

### 2.3 Install Ignition Gazebo Fortress

```bash
# Install Ignition Fortress (compatible with ROS2 Humble)
sudo apt-get update
sudo apt-get install ignition-fortress

# Verify installation
ign gazebo --version
# Should show: Ignition Gazebo, version 6.x.x
```

### 2.4 Install ROS-Gazebo Integration Packages

```bash
# Install ros_gz (ROS2-Ignition bridge packages)
sudo apt install ros-humble-ros-gz

# This installs:
# - ros_gz_bridge: Message translation between ROS2 and Ignition
# - ros_gz_sim: Launch Ignition from ROS2
# - ros_gz_image: Image transport bridge

# Install additional useful packages
sudo apt install ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-ros-gz-interfaces
```

### 2.5 Install Navigation & SLAM Packages

```bash
# Navigation2 stack
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup

# SLAM packages
sudo apt install ros-humble-slam-toolbox \
                 ros-humble-cartographer \
                 ros-humble-cartographer-ros

# Robot localization
sudo apt install ros-humble-robot-localization

# Teleop for manual control
sudo apt install ros-humble-teleop-twist-keyboard \
                 ros-humble-teleop-twist-joy
```

### 2.6 Install Development Tools

```bash
# Additional tools
sudo apt install ros-humble-xacro \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2

# Python tools
pip3 install transforms3d
```

### 2.7 Environment Setup

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export IGN_GAZEBO_RESOURCE_PATH=\$IGN_GAZEBO_RESOURCE_PATH:~/ros2_ws/src" >> ~/.bashrc
echo "export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=\$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib" >> ~/.bashrc

# Source it
source ~/.bashrc
```

---

## 3. Understanding Ignition Gazebo Fortress

### 3.1 Ignition vs Classic Gazebo

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    GAZEBO VERSIONS COMPARISON                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Gazebo Classic (gazebo11)         Ignition Gazebo Fortress                 │
│  ─────────────────────────         ────────────────────────                 │
│                                                                             │
│  • Command: gazebo                 • Command: ign gazebo                    │
│  • Plugin: .so files               • Plugin: .so files (different API)      │
│  • Topics: gazebo/                 • Topics: /world/<name>/                 │
│  • ROS bridge: gazebo_ros          • ROS bridge: ros_gz_bridge              │
│  • Model format: SDF               • Model format: SDF                      │
│  • URDF: Direct support            • URDF: Needs conversion                 │
│                                                                             │
│  Fortress is part of "Ignition" rebranding                                  │
│  (Now called "Gazebo" again in newer versions)                              │
│                                                                             │
│  Version mapping:                                                           │
│  ┌───────────────┬──────────────────┬─────────────────────┐                 │
│  │ ROS2 Version  │ Gazebo Version   │ Ignition Name       │                 │
│  ├───────────────┼──────────────────┼─────────────────────┤                 │
│  │ Humble        │ Fortress (6.x)   │ Ignition Fortress   │                 │
│  │ Iron          │ Garden (7.x)     │ Gazebo Garden       │                 │
│  │ Jazzy         │ Harmonic (8.x)   │ Gazebo Harmonic     │                 │
│  └───────────────┴──────────────────┴─────────────────────┘                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Ignition Gazebo Commands

```bash
# Launch empty world
ign gazebo empty.sdf

# Launch with specific world
ign gazebo my_world.sdf

# Launch with verbose output
ign gazebo -v 4 my_world.sdf

# List available topics (Ignition transport)
ign topic -l

# Echo a topic
ign topic -e -t /world/default/model/robot/link/base_link/sensor/imu/imu

# List available services
ign service -l

# Get model info
ign model --list
```

### 3.3 Ignition Topic Naming Convention

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    IGNITION TOPIC NAMING                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Pattern: /world/<world_name>/<entity_type>/<entity_name>/...               │
│                                                                             │
│  Examples:                                                                  │
│                                                                             │
│  /world/default/model/my_robot/link/base_link/sensor/lidar/scan             │
│  └──┬──┘└──┬───┘└─┬─┘└───┬───┘└─┬─┘└───┬────┘└──┬──┘└─┬──┘└─┬─┘             │
│     │      │      │      │      │      │        │     │     │               │
│     │      │      │      │      │      │        │     │     └─ Topic name   │
│     │      │      │      │      │      │        │     └─ Sensor name        │
│     │      │      │      │      │      │        └─ Entity type              │
│     │      │      │      │      │      └─ Link name                         │
│     │      │      │      │      └─ Entity type                              │
│     │      │      │      └─ Model name                                      │
│     │      │      └─ Entity type                                            │
│     │      └─ World name                                                    │
│     └─ Root                                                                 │
│                                                                             │
│  These get bridged to ROS2 topics like:                                     │
│    /scan, /imu, /camera/image_raw, /odom                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Creating the Robot Model (URDF/SDF)

### 4.1 Project Structure

```
my_robot_sim/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot.urdf.xacro        # Main robot description
│   ├── materials.xacro         # Colors and materials
│   ├── inertials.xacro         # Inertia macros
│   └── gazebo_plugins.xacro    # Sensor and plugin configs
├── meshes/                      # (Optional) 3D mesh files
│   ├── chassis.stl
│   └── wheel.stl
├── worlds/
│   └── my_world.sdf            # Gazebo world file
├── launch/
│   ├── gazebo.launch.py        # Launch simulation
│   ├── spawn_robot.launch.py   # Spawn robot in Gazebo
│   └── slam.launch.py          # SLAM mapping
├── config/
│   ├── bridge.yaml             # ROS-Gazebo bridge config
│   ├── slam_params.yaml        # SLAM parameters
│   └── nav2_params.yaml        # Navigation parameters
└── maps/                        # Generated maps
    ├── my_map.pgm
    └── my_map.yaml
```

### 4.2 Robot URDF with Xacro

Create `urdf/robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="diff_drive_robot">

  <!-- ==================== PROPERTIES ==================== -->
  
  <!-- Chassis dimensions -->
  <xacro:property name="chassis_length" value="0.4"/>
  <xacro:property name="chassis_width" value="0.3"/>
  <xacro:property name="chassis_height" value="0.15"/>
  <xacro:property name="chassis_mass" value="5.0"/>
  
  <!-- Wheel dimensions -->
  <xacro:property name="wheel_radius" value="0.08"/>
  <xacro:property name="wheel_width" value="0.04"/>
  <xacro:property name="wheel_mass" value="0.5"/>
  <xacro:property name="wheel_separation" value="0.34"/>
  <xacro:property name="wheel_offset_x" value="0.0"/>
  
  <!-- Caster dimensions -->
  <xacro:property name="caster_radius" value="0.04"/>
  <xacro:property name="caster_mass" value="0.1"/>
  
  <!-- Sensor positions -->
  <xacro:property name="lidar_x" value="0.1"/>
  <xacro:property name="lidar_z" value="0.2"/>
  <xacro:property name="camera_x" value="0.18"/>
  <xacro:property name="camera_z" value="0.1"/>
  <xacro:property name="imu_z" value="0.1"/>

  <!-- ==================== MACROS ==================== -->
  
  <!-- Inertia macro for box -->
  <xacro:macro name="box_inertia" params="m x y z">
    <inertia ixx="${(1/12)*m*(y*y+z*z)}" ixy="0" ixz="0"
             iyy="${(1/12)*m*(x*x+z*z)}" iyz="0"
             izz="${(1/12)*m*(x*x+y*y)}"/>
  </xacro:macro>
  
  <!-- Inertia macro for cylinder -->
  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertia ixx="${(1/12)*m*(3*r*r+h*h)}" ixy="0" ixz="0"
             iyy="${(1/12)*m*(3*r*r+h*h)}" iyz="0"
             izz="${(1/2)*m*r*r}"/>
  </xacro:macro>
  
  <!-- Inertia macro for sphere -->
  <xacro:macro name="sphere_inertia" params="m r">
    <inertia ixx="${(2/5)*m*r*r}" ixy="0" ixz="0"
             iyy="${(2/5)*m*r*r}" iyz="0"
             izz="${(2/5)*m*r*r}"/>
  </xacro:macro>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix y_offset">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="${wheel_mass}"/>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${wheel_offset_x} ${y_offset} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.1" friction="0.1"/>
    </joint>
  </xacro:macro>

  <!-- ==================== ROBOT BODY ==================== -->
  
  <!-- Base footprint (ground level) -->
  <link name="base_footprint"/>
  
  <!-- Base link (main chassis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <origin xyz="0 0 ${chassis_height/2 + wheel_radius}" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <origin xyz="0 0 ${chassis_height/2 + wheel_radius}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="${chassis_mass}"/>
      <origin xyz="0 0 ${chassis_height/2 + wheel_radius}" rpy="0 0 0"/>
      <xacro:box_inertia m="${chassis_mass}" x="${chassis_length}" 
                         y="${chassis_width}" z="${chassis_height}"/>
    </inertial>
  </link>
  
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ==================== WHEELS ==================== -->
  
  <!-- Left wheel -->
  <xacro:wheel prefix="left" y_offset="${wheel_separation/2}"/>
  
  <!-- Right wheel -->
  <xacro:wheel prefix="right" y_offset="${-wheel_separation/2}"/>
  
  <!-- Front caster -->
  <link name="front_caster">
    <visual>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${caster_mass}"/>
      <xacro:sphere_inertia m="${caster_mass}" r="${caster_radius}"/>
    </inertial>
  </link>
  
  <joint name="front_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${chassis_length/2 - caster_radius} 0 ${-wheel_radius + caster_radius}" rpy="0 0 0"/>
  </joint>
  
  <!-- Rear caster -->
  <link name="rear_caster">
    <visual>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${caster_mass}"/>
      <xacro:sphere_inertia m="${caster_mass}" r="${caster_radius}"/>
    </inertial>
  </link>
  
  <joint name="rear_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_caster"/>
    <origin xyz="${-chassis_length/2 + caster_radius} 0 ${-wheel_radius + caster_radius}" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## 5. Adding Sensors

### 5.1 Sensor Links and Joints (add to robot.urdf.xacro)

```xml
  <!-- ==================== SENSORS ==================== -->
  
  <!-- ========== LIDAR ========== -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <xacro:cylinder_inertia m="0.2" r="0.05" h="0.04"/>
    </inertial>
  </link>
  
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="${lidar_x} 0 ${lidar_z + wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ========== CAMERA ========== -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.08 0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.08 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <xacro:box_inertia m="0.1" x="0.02" y="0.08" z="0.04"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${camera_x} 0 ${camera_z + wheel_radius}" rpy="0 0 0"/>
  </joint>
  
  <!-- Camera optical frame (Z forward, X right, Y down) -->
  <link name="camera_optical_link"/>
  
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- ========== IMU ========== -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
      <material name="green">
        <color rgba="0.1 0.8 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <xacro:box_inertia m="0.01" x="0.02" y="0.02" z="0.01"/>
    </inertial>
  </link>
  
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${imu_z + wheel_radius}" rpy="0 0 0"/>
  </joint>
```

---

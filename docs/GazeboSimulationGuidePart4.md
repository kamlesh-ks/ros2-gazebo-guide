# Gazebo Simulation Guide - Part 4
## Package Configuration & RViz Setup

---

## 13. Package Configuration

### 13.1 Package Structure (Complete)

```
my_robot_sim/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot.urdf.xacro           # Main robot description
│   └── gazebo_plugins.xacro       # Sensor & plugin configs
├── worlds/
│   └── my_world.sdf               # Gazebo world
├── launch/
│   ├── gazebo.launch.py           # Launch Gazebo + Robot
│   ├── slam.launch.py             # Launch SLAM
│   ├── navigation.launch.py       # Launch Nav2
│   └── full_simulation.launch.py  # Complete demo
├── config/
│   ├── bridge.yaml                # ROS-Gazebo bridge
│   ├── slam_params.yaml           # SLAM Toolbox params
│   ├── nav2_params.yaml           # Navigation params
│   └── simulation.rviz            # RViz config
├── maps/
│   ├── my_map.pgm                 # Saved map image
│   └── my_map.yaml                # Saved map metadata
└── meshes/                         # (Optional) 3D models
```

### 13.2 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_sim</name>
  <version>1.0.0</version>
  <description>
    Differential drive robot simulation package for ROS2 Humble with Ignition Gazebo Fortress.
    Includes URDF model, sensors (LiDAR, Camera, IMU), SLAM mapping, and Nav2 navigation.
  </description>
  <maintainer email="kamleshsingh.singh3@gmail.com">Kamlesh Singh</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tools -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  
  <!-- Gazebo integration -->
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_image</exec_depend>
  
  <!-- Sensors and visualization -->
  <exec_depend>rviz2</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  
  <!-- SLAM -->
  <exec_depend>slam_toolbox</exec_depend>
  
  <!-- Navigation -->
  <exec_depend>navigation2</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  
  <!-- Teleop -->
  <exec_depend>teleop_twist_keyboard</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
    
    <!-- Declare Gazebo model path -->
    <gazebo_ros gazebo_model_path="${prefix}/.."/>
  </export>
</package>
```

### 13.3 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    urdf
    worlds
    launch
    config
    maps
    meshes
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

---

## 14. RViz Configuration

### 14.1 Simulation RViz Config

Create `config/simulation.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2

Visualization Manager:
  Class: ""
  Displays:
    # ==================== GRID ====================
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 50
      Reference Frame: <Fixed Frame>
      Value: true

    # ==================== ROBOT MODEL ====================
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
      Links:
        All Links Enabled: true
      Visual Enabled: true
      Collision Enabled: false
      Update Interval: 0
      Alpha: 1

    # ==================== TF ====================
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.3
      Show Arrows: true
      Show Axes: true
      Show Names: true

    # ==================== LASER SCAN ====================
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Enabled: true
      Topic:
        Value: /scan
      Size (m): 0.05
      Color Transformer: Intensity
      Decay Time: 0
      Min Color: 0; 255; 0
      Max Color: 255; 0; 0
      Style: Points
      
    # ==================== MAP ====================
    - Class: rviz_default_plugins/Map
      Name: Map
      Enabled: true
      Topic:
        Value: /map
      Alpha: 0.7
      Color Scheme: map
      Draw Behind: true
      Update Topic:
        Value: /map_updates

    # ==================== LOCAL COSTMAP ====================
    - Class: rviz_default_plugins/Map
      Name: Local Costmap
      Enabled: true
      Topic:
        Value: /local_costmap/costmap
      Alpha: 0.3
      Color Scheme: costmap
      Draw Behind: false

    # ==================== GLOBAL COSTMAP ====================
    - Class: rviz_default_plugins/Map
      Name: Global Costmap
      Enabled: false
      Topic:
        Value: /global_costmap/costmap
      Alpha: 0.3
      Color Scheme: costmap
      Draw Behind: false

    # ==================== PATH ====================
    - Class: rviz_default_plugins/Path
      Name: Global Path
      Enabled: true
      Topic:
        Value: /plan
      Line Style:
        Line Width: 0.03
        Value: Lines
      Color: 0; 255; 0
      Pose Style:
        Axes Length: 0.3
        Axes Radius: 0.01
        Shape: None

    # ==================== CAMERA IMAGE ====================
    - Class: rviz_default_plugins/Image
      Name: Camera
      Enabled: true
      Topic:
        Value: /camera/image_raw
      Normalize Range: true
      Max Value: 1
      Min Value: 0
      Median window: 5

    # ==================== ODOMETRY ====================
    - Class: rviz_default_plugins/Odometry
      Name: Odometry
      Enabled: false
      Topic:
        Value: /odom
      Position Tolerance: 0.1
      Angle Tolerance: 0.1
      Keep: 100
      Shape:
        Alpha: 1
        Axes Length: 1
        Axes Radius: 0.1
        Color: 255; 25; 0
        Head Length: 0.3
        Head Radius: 0.1
        Shaft Length: 1
        Shaft Radius: 0.05
        Value: Arrow

    # ==================== AMCL PARTICLES ====================
    - Class: rviz_default_plugins/PoseArray
      Name: AMCL Particles
      Enabled: true
      Topic:
        Value: /particlecloud
      Arrow Length: 0.1
      Color: 255; 100; 0

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30

  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose
    - Class: nav2_rviz_plugins/GoalTool

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 15
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785

Window Geometry:
  Displays:
    collapsed: false
  Height: 1080
  Width: 1920
  X: 0
  Y: 0
```

---

## 15. Alternative Sensor Configurations

### 15.1 Alternative LiDAR: 3D LiDAR (Velodyne-style)

```xml
<!-- 3D LiDAR (16 channels) -->
<gazebo reference="lidar_link">
  <sensor name="lidar_3d" type="gpu_lidar">
    <topic>points</topic>
    <update_rate>10</update_rate>
    <always_on>true</always_on>
    <visualize>true</visualize>
    
    <lidar>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
          <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.5</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </lidar>
    
    <frame_id>lidar_link</frame_id>
  </sensor>
</gazebo>
```

Bridge for 3D point cloud:
```yaml
- ros_topic_name: "/points"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/link/lidar_link/sensor/lidar_3d/scan/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  ign_type_name: "ignition.msgs.PointCloudPacked"
  direction: IGN_TO_ROS
```

### 15.2 Stereo Camera Setup

```xml
<!-- Left Camera -->
<link name="left_camera_link">
  <visual>
    <geometry><box size="0.02 0.02 0.02"/></geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry><box size="0.02 0.02 0.02"/></geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="left_camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="left_camera_link"/>
  <origin xyz="0.2 0.06 0.1" rpy="0 0 0"/>
</joint>

<!-- Right Camera -->
<link name="right_camera_link">
  <visual>
    <geometry><box size="0.02 0.02 0.02"/></geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry><box size="0.02 0.02 0.02"/></geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="right_camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="right_camera_link"/>
  <origin xyz="0.2 -0.06 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugins for stereo cameras -->
<gazebo reference="left_camera_link">
  <sensor name="left_camera" type="camera">
    <topic>stereo/left/image_raw</topic>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <frame_id>left_camera_link</frame_id>
  </sensor>
</gazebo>

<gazebo reference="right_camera_link">
  <sensor name="right_camera" type="camera">
    <topic>stereo/right/image_raw</topic>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <frame_id>right_camera_link</frame_id>
  </sensor>
</gazebo>
```

### 15.3 GPS Sensor

```xml
<gazebo reference="base_link">
  <sensor name="gps" type="gps">
    <topic>gps/fix</topic>
    <update_rate>10</update_rate>
    <always_on>true</always_on>
    
    <gps>
      <position_sensing>
        <horizontal>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.5</stddev>
          </noise>
        </horizontal>
        <vertical>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>1.0</stddev>
          </noise>
        </vertical>
      </position_sensing>
    </gps>
    
    <frame_id>base_link</frame_id>
  </sensor>
</gazebo>
```

---

## 16. Summary: Complete Workflow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    COMPLETE SIMULATION WORKFLOW                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     PHASE 1: SETUP                                  │    │
│  ├─────────────────────────────────────────────────────────────────────┤    │
│  │  1. Install ROS2 Humble                                             │    │
│  │  2. Install Ignition Gazebo Fortress                                │    │
│  │  3. Install ros_gz bridge packages                                  │    │
│  │  4. Install Navigation2 and SLAM Toolbox                            │    │
│  │  5. Create package with URDF, world, configs                        │    │
│  │  6. Build with colcon                                               │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                │                                            │
│                                ▼                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     PHASE 2: MAPPING                                │    │
│  ├─────────────────────────────────────────────────────────────────────┤    │
│  │  1. Launch simulation with SLAM:                                    │    │
│  │     ros2 launch my_robot_sim full_simulation.launch.py slam:=true   │    │
│  │                                                                     │    │
│  │  2. Drive robot with teleop:                                        │    │
│  │     ros2 run teleop_twist_keyboard teleop_twist_keyboard            │    │
│  │                                                                     │    │
│  │  3. Explore entire environment                                      │    │
│  │                                                                     │    │
│  │  4. Save map:                                                       │    │
│  │     ros2 run nav2_map_server map_saver_cli -f maps/my_map           │    │
│  │                                                                     │    │
│  │  5. Copy map to package, rebuild                                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                │                                            │
│                                ▼                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     PHASE 3: NAVIGATION                             │    │
│  ├─────────────────────────────────────────────────────────────────────┤    │
│  │  1. Launch simulation with navigation:                              │    │
│  │     ros2 launch my_robot_sim full_simulation.launch.py nav:=true    │    │
│  │                                                                     │    │
│  │  2. Set initial pose in RViz (2D Pose Estimate)                     │    │
│  │                                                                     │    │
│  │  3. Send goal in RViz (Nav2 Goal)                                   │    │
│  │                                                                     │    │
│  │  4. Watch robot navigate autonomously!                              │    │
│  │                                                                     │    │
│  │  5. Tune parameters as needed                                       │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 17. Recommended Learning Resources

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      LEARNING RESOURCES                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Official Documentation:                                                    │
│  ────────────────────────                                                   │
│  • ROS2 Humble: https://docs.ros.org/en/humble/                             │
│  • Navigation2: https://navigation.ros.org/                                 │
│  • Gazebo (Ignition): https://gazebosim.org/docs/fortress                   │
│  • SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox              │
│                                                                             │
│  Tutorials:                                                                 │
│  ──────────                                                                 │
│  • ROS2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html            │
│  • Nav2 Tutorials: https://navigation.ros.org/tutorials/index.html          │
│  • Gazebo Tutorials: https://gazebosim.org/docs/fortress/tutorials          │
│                                                                             │
│  Example Packages:                                                          │
│  ─────────────────                                                          │
│  • TurtleBot3: https://github.com/ROBOTIS-GIT/turtlebot3                    │
│  • Nav2 Examples: https://github.com/ros-planning/navigation2               │
│  • Dolly (Gazebo demo): https://github.com/chapulina/dolly                  │
│                                                                             │
│  Community:                                                                 │
│  ──────────                                                                 │
│  • ROS Discourse: https://discourse.ros.org/                                │
│  • ROS Answers: https://answers.ros.org/                                    │
│  • Gazebo Community: https://community.gazebosim.org/                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

This completes the comprehensive 4-part guide for setting up and using ROS2 Humble with Ignition Gazebo Fortress!

**Summary of all parts:**
- **Part 1**: Overview, Installation, Understanding Ignition, Robot URDF
- **Part 2**: Gazebo Plugins, Sensors, World Creation, Bridge Configuration
- **Part 3**: Launch Files, SLAM, Navigation, Troubleshooting
- **Part 4**: Package Configuration, RViz Setup, Alternative Sensors, Workflow Summary

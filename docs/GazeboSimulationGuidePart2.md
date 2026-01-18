# Gazebo Simulation Guide - Part 2
## Gazebo Plugins, World Creation & Bridge Configuration

---

## 5.2 Gazebo Plugins for Sensors

Create `urdf/gazebo_plugins.xacro` (include this in your main URDF):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ==================== GAZEBO SETTINGS ==================== -->
  
  <gazebo>
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
  </gazebo>
  
  <!-- Material colors for Gazebo -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  
  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  
  <gazebo reference="front_caster">
    <material>Gazebo/Grey</material>
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>
  
  <gazebo reference="rear_caster">
    <material>Gazebo/Grey</material>
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>

  <!-- ==================== DIFFERENTIAL DRIVE ==================== -->
  
  <gazebo>
    <plugin
      filename="libignition-gazebo-diff-drive-system.so"
      name="ignition::gazebo::systems::DiffDrive">
      
      <!-- Wheel joints -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      
      <!-- Kinematics -->
      <wheel_separation>${wheel_separation}</wheel_separation>
      <wheel_radius>${wheel_radius}</wheel_radius>
      
      <!-- Limits -->
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <max_angular_acceleration>2.0</max_angular_acceleration>
      <max_linear_velocity>1.0</max_linear_velocity>
      <max_angular_velocity>2.0</max_angular_velocity>
      
      <!-- Odometry -->
      <odom_publish_frequency>50</odom_publish_frequency>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      
    </plugin>
  </gazebo>

  <!-- ==================== JOINT STATE PUBLISHER ==================== -->
  
  <gazebo>
    <plugin
      filename="libignition-gazebo-joint-state-publisher-system.so"
      name="ignition::gazebo::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <joint_name>left_wheel_joint</joint_name>
      <joint_name>right_wheel_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- ==================== LIDAR SENSOR ==================== -->
  
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <topic>scan</topic>
      <update_rate>10</update_rate>
      <always_on>true</always_on>
      <visualize>true</visualize>
      
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
          <vertical>
            <samples>1</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>
      
      <frame_id>lidar_link</frame_id>
    </sensor>
  </gazebo>

  <!-- ==================== CAMERA SENSOR ==================== -->
  
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <topic>camera/image_raw</topic>
      <update_rate>30</update_rate>
      <always_on>true</always_on>
      
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>100</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      
      <frame_id>camera_optical_link</frame_id>
    </sensor>
  </gazebo>

  <!-- ==================== DEPTH CAMERA (Optional) ==================== -->
  
  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth_camera">
      <topic>camera/depth</topic>
      <update_rate>15</update_rate>
      <always_on>true</always_on>
      
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R_FLOAT32</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      
      <frame_id>camera_optical_link</frame_id>
    </sensor>
  </gazebo>

  <!-- ==================== IMU SENSOR ==================== -->
  
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <topic>imu</topic>
      <update_rate>100</update_rate>
      <always_on>true</always_on>
      
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0002</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0002</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0002</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      
      <frame_id>imu_link</frame_id>
    </sensor>
  </gazebo>

</robot>
```

---

## 6. Differential Drive Deep Dive

### 6.1 How Differential Drive Works

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    DIFFERENTIAL DRIVE KINEMATICS                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Physical Setup:                                                            │
│  ───────────────                                                            │
│           ┌─────────────────────────────┐                                   │
│           │                             │                                   │
│    ┌──────┤          CHASSIS           ├──────┐                             │
│    │ Left │                             │Right │                            │
│    │Wheel │             ●               │Wheel │                            │
│    │      │          (center)           │      │                            │
│    └──────┤                             ├──────┘                            │
│           │                             │                                   │
│           └─────────────────────────────┘                                   │
│              ◄──── wheel_separation ────►                                   │
│                         (L)                                                 │
│                                                                             │
│  Kinematics Equations:                                                      │
│  ─────────────────────                                                      │
│                                                                             │
│  Forward velocity:     v = (v_right + v_left) / 2                           │
│  Angular velocity:     ω = (v_right - v_left) / L                           │
│                                                                             │
│  Where:                                                                     │
│    v_left  = ω_left × r   (left wheel linear velocity)                      │
│    v_right = ω_right × r  (right wheel linear velocity)                     │
│    r = wheel_radius                                                         │
│    L = wheel_separation                                                     │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  Motion Examples:                                                           │
│  ────────────────                                                           │
│                                                                             │
│  Forward:        v_left = v_right     →  v > 0, ω = 0                       │
│  Backward:       v_left = v_right < 0 →  v < 0, ω = 0                       │
│  Rotate Left:    v_left < v_right     →  ω > 0 (CCW)                        │
│  Rotate Right:   v_left > v_right     →  ω < 0 (CW)                         │
│  Spin in Place:  v_left = -v_right    →  v = 0, ω ≠ 0                       │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  Inverse Kinematics (cmd_vel → wheel velocities):                           │
│  ───────────────────────────────────────────────                            │
│                                                                             │
│  v_left  = v - (ω × L / 2)                                                  │
│  v_right = v + (ω × L / 2)                                                  │
│                                                                             │
│  ω_left  = v_left / r                                                       │
│  ω_right = v_right / r                                                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 6.2 Diff Drive Plugin Parameters

```xml
<plugin filename="libignition-gazebo-diff-drive-system.so"
        name="ignition::gazebo::systems::DiffDrive">
```

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    DIFF DRIVE PLUGIN PARAMETERS                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Parameter               │ Description                │ Our Value           │
│  ────────────────────────┼────────────────────────────┼─────────────────    │
│  <left_joint>            │ Name of left wheel joint   │ left_wheel_joint    │
│  <right_joint>           │ Name of right wheel joint  │ right_wheel_joint   │
│  <wheel_separation>      │ Distance between wheels    │ 0.34 m              │
│  <wheel_radius>          │ Radius of wheels           │ 0.08 m              │
│  <max_linear_velocity>   │ Max forward/back speed     │ 1.0 m/s             │
│  <max_angular_velocity>  │ Max rotation speed         │ 2.0 rad/s           │
│  <max_linear_accel>      │ Max linear acceleration    │ 1.0 m/s²            │
│  <max_angular_accel>     │ Max angular acceleration   │ 2.0 rad/s²          │
│  <topic>                 │ Input command topic        │ cmd_vel             │
│  <odom_topic>            │ Output odometry topic      │ odom                │
│  <odom_publish_frequency>│ How often to publish odom  │ 50 Hz               │
│  <frame_id>              │ Odom frame name            │ odom                │
│  <child_frame_id>        │ Robot frame name           │ base_footprint      │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  Input Topic: /cmd_vel (geometry_msgs/Twist)                                │
│  ───────────────────────────────────────────                                │
│  {                                                                          │
│    linear:                                                                  │
│      x: 0.5    # Forward velocity (m/s)                                     │
│      y: 0.0    # Not used in diff drive                                     │
│      z: 0.0    # Not used                                                   │
│    angular:                                                                 │
│      x: 0.0    # Not used                                                   │
│      y: 0.0    # Not used                                                   │
│      z: 0.3    # Rotation velocity (rad/s)                                  │
│  }                                                                          │
│                                                                             │
│  Output Topic: /odom (nav_msgs/Odometry)                                    │
│  ────────────────────────────────────────                                   │
│  Contains:                                                                  │
│    - Position (x, y, z)                                                     │
│    - Orientation (quaternion)                                               │
│    - Linear velocity                                                        │
│    - Angular velocity                                                       │
│    - Covariance matrices                                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 7. Creating the Gazebo World

### 7.1 Basic World File

Create `worlds/my_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="my_world">
    
    <!-- ==================== PHYSICS ==================== -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- ==================== PLUGINS ==================== -->
    
    <!-- Required: Physics plugin -->
    <plugin
      filename="libignition-gazebo-physics-system.so"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    
    <!-- Required: User commands (spawn, delete models) -->
    <plugin
      filename="libignition-gazebo-user-commands-system.so"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    
    <!-- Required: Scene broadcaster (for visualization) -->
    <plugin
      filename="libignition-gazebo-scene-broadcaster-system.so"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Sensors plugin -->
    <plugin
      filename="libignition-gazebo-sensors-system.so"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    
    <!-- IMU plugin -->
    <plugin
      filename="libignition-gazebo-imu-system.so"
      name="ignition::gazebo::systems::Imu">
    </plugin>
    
    <!-- Contact sensor plugin -->
    <plugin
      filename="libignition-gazebo-contact-system.so"
      name="ignition::gazebo::systems::Contact">
    </plugin>

    <!-- ==================== LIGHTING ==================== -->
    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- ==================== GROUND PLANE ==================== -->
    
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ==================== WALLS ==================== -->
    
    <!-- Wall 1 (North) -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 2 (South) -->
    <model name="wall_south">
      <static>true</static>
      <pose>0 -5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 3 (East) -->
    <model name="wall_east">
      <static>true</static>
      <pose>5 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Wall 4 (West) -->
    <model name="wall_west">
      <static>true</static>
      <pose>-5 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ==================== OBSTACLES ==================== -->
    
    <!-- Box obstacle 1 -->
    <model name="obstacle_box_1">
      <static>true</static>
      <pose>2 2 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Cylinder obstacle -->
    <model name="obstacle_cylinder">
      <static>true</static>
      <pose>-2 1 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- L-shaped wall -->
    <model name="l_wall_1">
      <static>true</static>
      <pose>0 -2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="l_wall_2">
      <static>true</static>
      <pose>1 -3 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## 8. ROS2-Gazebo Bridge Configuration

### 8.1 Understanding the Bridge

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS-GAZEBO BRIDGE                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  The bridge translates between:                                             │
│    • Ignition Transport (Gazebo's messaging system)                         │
│    • ROS2 DDS (ROS2's messaging system)                                     │
│                                                                             │
│  ┌─────────────────────┐                    ┌─────────────────────┐         │
│  │   IGNITION GAZEBO   │                    │       ROS2          │         │
│  │                     │                    │                     │         │
│  │  /world/.../scan    │◄────── Bridge ────►│ /scan               │         │
│  │  ignition.msgs.     │                    │ sensor_msgs/        │         │
│  │  LaserScan          │                    │ LaserScan           │         │
│  │                     │                    │                     │         │
│  │  /model/.../cmd_vel │◄────── Bridge ────►│ /cmd_vel            │         │
│  │  ignition.msgs.     │                    │ geometry_msgs/      │         │
│  │  Twist              │                    │ Twist               │         │
│  └─────────────────────┘                    └─────────────────────┘         │
│                                                                             │
│  Bridge Direction:                                                          │
│    [Ignition → ROS2]: Sensors (scan, odom, imu, camera)                     │
│    [ROS2 → Ignition]: Commands (cmd_vel)                                    │
│    [Bidirectional]:   Some topics can go both ways                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 8.2 Bridge Configuration File

Create `config/bridge.yaml`:

```yaml
# ROS-Ignition Bridge Configuration
# Format: - ros_topic_name: "topic" 
#           ros_type_name: "package/MsgType"
#           ign_topic_name: "/ignition/topic"
#           ign_type_name: "ignition.msgs.MsgType"
#           direction: BIDIRECTIONAL | ROS_TO_IGN | IGN_TO_ROS

---

# ==================== CLOCK ====================
# Simulation time
- ros_topic_name: "/clock"
  ign_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  ign_type_name: "ignition.msgs.Clock"
  direction: IGN_TO_ROS

# ==================== CONTROL ====================
# Velocity commands (ROS2 → Gazebo)
- ros_topic_name: "/cmd_vel"
  ign_topic_name: "/model/diff_drive_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  ign_type_name: "ignition.msgs.Twist"
  direction: ROS_TO_IGN

# ==================== ODOMETRY ====================
# Odometry (Gazebo → ROS2)
- ros_topic_name: "/odom"
  ign_topic_name: "/model/diff_drive_robot/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  ign_type_name: "ignition.msgs.Odometry"
  direction: IGN_TO_ROS

# ==================== TF ====================
# Transform tree
- ros_topic_name: "/tf"
  ign_topic_name: "/model/diff_drive_robot/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  ign_type_name: "ignition.msgs.Pose_V"
  direction: IGN_TO_ROS

# ==================== JOINT STATES ====================
- ros_topic_name: "/joint_states"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  ign_type_name: "ignition.msgs.Model"
  direction: IGN_TO_ROS

# ==================== LIDAR ====================
- ros_topic_name: "/scan"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/link/lidar_link/sensor/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  ign_type_name: "ignition.msgs.LaserScan"
  direction: IGN_TO_ROS

# ==================== CAMERA ====================
- ros_topic_name: "/camera/image_raw"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/link/camera_link/sensor/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  ign_type_name: "ignition.msgs.Image"
  direction: IGN_TO_ROS

# Camera info
- ros_topic_name: "/camera/camera_info"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/link/camera_link/sensor/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  ign_type_name: "ignition.msgs.CameraInfo"
  direction: IGN_TO_ROS

# ==================== DEPTH CAMERA ====================
- ros_topic_name: "/camera/depth/image_raw"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/link/camera_link/sensor/depth_camera/depth_image"
  ros_type_name: "sensor_msgs/msg/Image"
  ign_type_name: "ignition.msgs.Image"
  direction: IGN_TO_ROS

# ==================== IMU ====================
- ros_topic_name: "/imu"
  ign_topic_name: "/world/my_world/model/diff_drive_robot/link/imu_link/sensor/imu/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  ign_type_name: "ignition.msgs.IMU"
  direction: IGN_TO_ROS
```

### 8.3 Bridge Command Line (Alternative)

Instead of YAML, you can bridge topics directly:

```bash
# Single topic bridge
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan

# Multiple topics
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry

# Syntax:
#   [  means Ignition → ROS2
#   ]  means ROS2 → Ignition
#   @  separates topic from type
```

---


# Gazebo Simulation Guide - Part 3
## Launch Files, SLAM & Navigation

---

## 9. Launch Files

### 9.1 Main Gazebo Launch File

Create `launch/gazebo.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch Ignition Gazebo with the world and robot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_sim')
    
    # Paths
    world_file = os.path.join(pkg_dir, 'worlds', 'my_world.sdf')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Process URDF with xacro
    robot_description_content = os.popen(f'xacro {urdf_file}').read()
    
    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # ==================== GAZEBO ====================
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo', '-v', '4', '-r',  # -r starts simulation immediately
                world_file
            ],
            output='screen'
        ),
        
        # ==================== SPAWN ROBOT ====================
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'diff_drive_robot',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),
        
        # ==================== ROBOT STATE PUBLISHER ====================
        # Publishes robot URDF to /robot_description and TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
            }]
        ),
        
        # ==================== ROS-GAZEBO BRIDGE ====================
        # Bridge Ignition topics to ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': bridge_config,
                'use_sim_time': use_sim_time,
            }],
            output='screen'
        ),
        
        # ==================== STATIC TRANSFORMS ====================
        # If needed, publish static transforms
        # (Usually handled by robot_state_publisher from URDF)
    ])
```

### 9.2 SLAM Launch File

Create `launch/slam.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch SLAM Toolbox for mapping
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_sim')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # SLAM Toolbox - Online Async mode (for real-time mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/scan', '/scan'),
            ]
        ),
    ])
```

### 9.3 SLAM Parameters

Create `config/slam_params.yaml`:

```yaml
# SLAM Toolbox Parameters
slam_toolbox:
  ros__parameters:
    # ==================== GENERAL ====================
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ==================== FRAMES ====================
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping  # or 'localization' for using existing map

    # ==================== MAP UPDATE ====================
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 10.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    
    # ==================== POSE TRACKING ====================
    # Distance traveled before processing new scan
    minimum_travel_distance: 0.3
    # Rotation before processing new scan  
    minimum_travel_heading: 0.3
    
    # ==================== SCAN MATCHING ====================
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # ==================== CORRELATION ====================
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # ==================== LOOP CLOSURE ====================
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    loop_search_maximum_distance: 3.0

    # ==================== DEBUG ====================
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_file_name: ""
    map_start_pose: [0.0, 0.0, 0.0]
    map_start_at_dock: true
    
    # ==================== INTERACTIVE MODE ====================
    enable_interactive_mode: true
```

### 9.4 Navigation Launch File

Create `launch/navigation.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch Navigation2 stack
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml = LaunchConfiguration('map', default=map_file)
    
    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start nav2 stack'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Path to map yaml file'
        ),
        
        # ==================== NAV2 BRINGUP ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map': map_yaml,
                'params_file': nav2_params_file,
            }.items()
        ),
    ])
```

### 9.5 Navigation Parameters

Create `config/nav2_params.yaml`:

```yaml
# Nav2 Parameters for Simulation
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    use_sim_time: True
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: ""

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True

recoveries_server:
  ros__parameters:
    use_sim_time: True
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_footprint
    transform_timeout: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
```

### 9.6 Complete Demo Launch File

Create `launch/full_simulation.launch.py`:

```python
#!/usr/bin/env python3
"""
Complete simulation launch file
Launches: Gazebo + Robot + Bridge + RViz + (optionally SLAM or Navigation)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_sim')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='false')
    nav = LaunchConfiguration('nav', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    
    # RViz config
    rviz_config = os.path.join(pkg_dir, 'config', 'simulation.rviz')
    
    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false', 
                              description='Start SLAM for mapping'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Start Navigation with existing map'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        
        # ==================== GAZEBO ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # ==================== SLAM (conditional) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'slam.launch.py')
            ),
            condition=IfCondition(slam),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # ==================== NAVIGATION (conditional) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
            ),
            condition=IfCondition(nav),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # ==================== RVIZ ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen'
        ),
        
        # ==================== TELEOP ====================
        # Uncomment to include teleop keyboard
        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     name='teleop',
        #     prefix='xterm -e',
        #     output='screen'
        # ),
    ])
```

---

## 10. Map Creation Workflow

### 10.1 Step-by-Step Mapping Process

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    MAP CREATION WORKFLOW                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  STEP 1: Launch Simulation with SLAM                                        │
│  ───────────────────────────────────                                        │
│  $ ros2 launch my_robot_sim full_simulation.launch.py slam:=true            │
│                                                                             │
│  This starts:                                                               │
│    ✓ Ignition Gazebo with world                                             │
│    ✓ Robot spawned in world                                                 │
│    ✓ ROS-Gazebo bridge                                                      │
│    ✓ SLAM Toolbox                                                           │
│    ✓ RViz for visualization                                                 │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 2: Drive Robot Around (in separate terminal)                          │
│  ─────────────────────────────────────────────────                          │
│  $ ros2 run teleop_twist_keyboard teleop_twist_keyboard                     │
│                                                                             │
│  Controls:                                                                  │
│    u   i   o       (forward + turning)                                      │
│    j   k   l       (left, stop, right)                                      │
│    m   ,   .       (backward + turning)                                     │
│                                                                             │
│    q/z : increase/decrease max speeds by 10%                                │
│    w/x : increase/decrease only linear speed by 10%                         │
│    e/c : increase/decrease only angular speed by 10%                        │
│                                                                             │
│  Drive through all areas of the environment!                                │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 3: Monitor in RViz                                                    │
│  ───────────────────────                                                    │
│  Add displays:                                                              │
│    • Map (topic: /map)                                                      │
│    • LaserScan (topic: /scan)                                               │
│    • TF                                                                     │
│    • RobotModel                                                             │
│                                                                             │
│  Watch the map build as you drive!                                          │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 4: Save the Map                                                       │
│  ────────────────────                                                       │
│  When mapping is complete:                                                  │
│                                                                             │
│  $ ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map                  │
│                                                                             │
│  This creates:                                                              │
│    ~/maps/my_map.pgm  (image file - black=occupied, white=free)             │
│    ~/maps/my_map.yaml (metadata - resolution, origin, thresholds)           │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 5: Copy Map to Package                                                │
│  ───────────────────────────                                                │
│  $ cp ~/maps/my_map.* ~/ros2_ws/src/my_robot_sim/maps/                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 10.2 Map File Format

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         MAP FILE FORMAT                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  my_map.yaml (Metadata):                                                    │
│  ───────────────────────                                                    │
│                                                                             │
│  image: my_map.pgm                                                          │
│  resolution: 0.05          # meters per pixel                               │
│  origin: [-5.0, -5.0, 0.0] # [x, y, yaw] of lower-left corner               │
│  occupied_thresh: 0.65     # probability > this = occupied                  │
│  free_thresh: 0.25         # probability < this = free                      │
│  negate: 0                 # 0 = white is free, 1 = white is occupied       │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  my_map.pgm (Image):                                                        │
│  ──────────────────                                                         │
│                                                                             │
│  PGM = Portable Gray Map (grayscale image)                                  │
│                                                                             │
│  Pixel values:                                                              │
│    254 (white)  = Free space                                                │
│    0 (black)    = Occupied (wall/obstacle)                                  │
│    205 (gray)   = Unknown                                                   │
│                                                                             │
│  Visual example:                                                            │
│  ┌──────────────────────────────────┐                                       │
│  │████████████████████████████████│                                         │
│  │█                              █│                                         │
│  │█   ░░░░░░░░░░░░░░░░░░░░░░    █│  ░ = free space                          │
│  │█   ░░░░░░░░░░░░░░░░░░░░░░    █│  █ = walls                               │
│  │█   ░░░░░█████░░░░░░░░░░░░    █│  ▓ = obstacle                            │
│  │█   ░░░░░█▓▓▓█░░░░░░░░░░░░    █│                                          │
│  │█   ░░░░░░░░░░░░░░░░░░░░░░    █│                                          │
│  │█                              █│                                         │
│  │████████████████████████████████│                                         │
│  └──────────────────────────────────┘                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 11. Navigation Workflow

### 11.1 Using Navigation with Saved Map

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    NAVIGATION WORKFLOW                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  STEP 1: Launch Simulation with Navigation                                  │
│  ─────────────────────────────────────────                                  │
│  $ ros2 launch my_robot_sim full_simulation.launch.py nav:=true             │
│                                                                             │
│  This starts:                                                               │
│    ✓ Ignition Gazebo with world                                             │
│    ✓ Robot spawned                                                          │
│    ✓ ROS-Gazebo bridge                                                      │
│    ✓ Navigation2 stack (with saved map)                                     │
│    ✓ AMCL (localization)                                                    │
│    ✓ RViz                                                                   │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 2: Set Initial Pose in RViz                                           │
│  ────────────────────────────────                                           │
│  1. Click "2D Pose Estimate" button in RViz toolbar                         │
│  2. Click on map where robot is located                                     │
│  3. Drag arrow to indicate robot's orientation                              │
│  4. AMCL will converge to correct position                                  │
│                                                                             │
│  Why needed?                                                                │
│    • AMCL needs initial guess of robot position                             │
│    • Simulation starts at known position, but AMCL doesn't know it          │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 3: Send Navigation Goal                                               │
│  ────────────────────────────                                               │
│  1. Click "Nav2 Goal" button in RViz toolbar                                │
│  2. Click on map where you want robot to go                                 │
│  3. Drag arrow to indicate desired final orientation                        │
│  4. Watch robot navigate!                                                   │
│                                                                             │
│  OR via command line:                                                       │
│  $ ros2 action send_goal /navigate_to_pose \                                │
│      nav2_msgs/action/NavigateToPose \                                      │
│      "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, \        │
│       y: 1.0}, orientation: {w: 1.0}}}}"                                    │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  STEP 4: Monitor Navigation                                                 │
│  ─────────────────────────                                                  │
│  In RViz, add:                                                              │
│    • Path (topic: /plan) - global path                                      │
│    • Local Costmap (topic: /local_costmap/costmap)                          │
│    • Global Costmap (topic: /global_costmap/costmap)                        │
│    • Markers for trajectories                                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 12. Troubleshooting

### 12.1 Common Issues & Solutions

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      TROUBLESHOOTING                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ISSUE: Robot doesn't move when sending cmd_vel                             │
│  ──────────────────────────────────────────────                             │
│  Solutions:                                                                 │
│    1. Check bridge is running: ros2 topic list (should see /cmd_vel)        │
│    2. Check Ignition topic: ign topic -l | grep cmd_vel                     │
│    3. Verify model name matches in bridge config                            │
│    4. Check wheel joint names match diff_drive plugin config                │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  ISSUE: No LiDAR data on /scan                                              │
│  ─────────────────────────────                                              │
│  Solutions:                                                                 │
│    1. Verify sensor plugin loaded: check Gazebo console for errors          │
│    2. Check topic path in bridge config matches Ignition topic              │
│    3. Use: ign topic -e -t /world/my_world/.../scan                         │
│    4. Make sure sensor update_rate > 0                                      │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  ISSUE: TF tree broken (frames not connected)                               │
│  ────────────────────────────────────────────                               │
│  Solutions:                                                                 │
│    1. Check: ros2 run tf2_tools view_frames                                 │
│    2. Ensure robot_state_publisher is running                               │
│    3. Verify odom→base_footprint TF from diff_drive plugin                  │
│    4. Check frame_id names match across all configs                         │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  ISSUE: Robot falls through ground                                          │
│  ─────────────────────────────────                                          │
│  Solutions:                                                                 │
│    1. Check collision elements in URDF                                      │
│    2. Verify inertia values are reasonable                                  │
│    3. Spawn robot slightly above ground (z: 0.1)                            │
│    4. Check physics step size (try smaller value)                           │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  ISSUE: SLAM doesn't create map                                             │
│  ──────────────────────────────                                             │
│  Solutions:                                                                 │
│    1. Check /scan topic is publishing: ros2 topic hz /scan                  │
│    2. Verify TF from base_footprint to lidar_link exists                    │
│    3. Check slam_params.yaml frame names                                    │
│    4. Drive robot - SLAM needs movement to trigger                          │
│                                                                             │
│  ───────────────────────────────────────────────────────────────────────    │
│                                                                             │
│  ISSUE: Navigation fails to find path                                       │
│  ────────────────────────────────                                           │
│  Solutions:                                                                 │
│    1. Check robot_radius in costmap params (too big?)                       │
│    2. Verify inflation_radius settings                                      │
│    3. Check map is loaded: ros2 topic echo /map --once                      │
│    4. Set initial pose in RViz                                              │
│    5. Check goal is in free space, not inside wall                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 12.2 Useful Debug Commands

```bash
# ==================== TOPIC DEBUGGING ====================
# List all ROS2 topics
ros2 topic list

# List all Ignition topics
ign topic -l

# Check topic publishing rate
ros2 topic hz /scan
ros2 topic hz /odom

# Echo topic data
ros2 topic echo /scan --once
ros2 topic echo /odom --once

# ==================== TF DEBUGGING ====================
# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# ==================== NODE DEBUGGING ====================
# List all nodes
ros2 node list

# Get node info
ros2 node info /slam_toolbox

# ==================== SERVICE DEBUGGING ====================
# List services
ros2 service list

# Call map saver
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: 'map', map_url: 'my_map', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}"

# ==================== GAZEBO DEBUGGING ====================
# Check Gazebo model state
ign model --list

# Check Gazebo world info
ign service -s /world/my_world/state --reqtype ignition.msgs.Empty --reptype ignition.msgs.SerializedState --timeout 2000 --req ""
```

---

## Quick Reference Commands

```bash
# ==================== BUILD ====================
cd ~/ros2_ws
colcon build --packages-select my_robot_sim
source install/setup.bash

# ==================== LAUNCH ====================
# Gazebo only
ros2 launch my_robot_sim gazebo.launch.py

# With SLAM (for mapping)
ros2 launch my_robot_sim full_simulation.launch.py slam:=true

# With Navigation (using saved map)
ros2 launch my_robot_sim full_simulation.launch.py nav:=true

# ==================== TELEOP ====================
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ==================== SAVE MAP ====================
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# ==================== USEFUL CHECKS ====================
ros2 topic list
ros2 topic echo /scan --once
ros2 run tf2_tools view_frames
```

---

This completes the comprehensive guide for setting up ROS2 Humble with Ignition Gazebo Fortress for robot simulation, mapping, and navigation!

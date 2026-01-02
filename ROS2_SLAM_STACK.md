# ROS2 SLAM Stack Configuration

Complete SLAM and navigation stack for Big RoverBot v1.

## Hardware Summary

| Component | Model | ROS2 Interface |
|-----------|-------|----------------|
| Lidar | RPLidar A1 | `/scan` (LaserScan) |
| Depth Camera | Intel RealSense D435 | `/camera/depth`, `/camera/color` |
| IMU | Adafruit BNO055 | `/imu/data` (via Arduino) |
| GPS | TN GPS + Compass | `/gps/fix`, `/gps/heading` |
| Motors | Pololu 37D w/encoders | `/wheel_odom` (via Arduino) |
| Controller | Arduino Mega | USB serial to Pi |
| Brain | Raspberry Pi 5 16GB (Jazzy) | All ROS2 nodes |

---

## 1. Package Installation

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Core navigation and SLAM
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-rtabmap-ros

# Sensor drivers
sudo apt install -y \
  ros-jazzy-rplidar-ros \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description

# GPS and serial
sudo apt install -y \
  ros-jazzy-nmea-navsat-driver \
  ros-jazzy-serial-driver

# micro-ROS for Arduino (optional - alternative to rosserial)
sudo apt install -y \
  ros-jazzy-micro-ros-agent

# Visualization and tools
sudo apt install -y \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt* \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro

# Teleop for testing
sudo apt install -y \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-teleop-twist-joy
```

---

## 2. Workspace Setup

```bash
# Create workspace
mkdir -p ~/big_roverbot_ws/src
cd ~/big_roverbot_ws/src

# Create robot package
ros2 pkg create --build-type ament_python big_roverbot \
  --dependencies rclpy geometry_msgs sensor_msgs nav_msgs

# Create config and launch directories
mkdir -p big_roverbot/config
mkdir -p big_roverbot/launch
mkdir -p big_roverbot/urdf

# Build
cd ~/big_roverbot_ws
colcon build --symlink-install
source install/setup.bash

# Add to .bashrc
echo "source ~/big_roverbot_ws/install/setup.bash" >> ~/.bashrc
```

---

## 3. Robot URDF

Create `big_roverbot/urdf/big_roverbot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="big_roverbot">

  <!-- Properties based on actual rover dimensions -->
  <xacro:property name="chassis_length" value="0.610"/>  <!-- 24 inches -->
  <xacro:property name="chassis_width" value="0.305"/>   <!-- 12 inches -->
  <xacro:property name="chassis_height" value="0.050"/>  <!-- T-slot height -->
  <xacro:property name="wheel_radius" value="0.096"/>    <!-- 192mm / 2 -->
  <xacro:property name="wheel_width" value="0.072"/>     <!-- 72mm -->
  <xacro:property name="wheel_offset_y" value="0.190"/>  <!-- Half width + clearance -->

  <!-- Base footprint (ground projection) -->
  <link name="base_footprint"/>

  <!-- Base link (robot center) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <material name="aluminum">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.3"/>  <!-- Estimated weight in kg -->
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>
    <joint name="${name}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${name}_wheel"/>
      <origin xyz="${x} ${y} 0" rpy="${-pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Four wheels -->
  <xacro:wheel name="front_left" x="0.20" y="${wheel_offset_y}"/>
  <xacro:wheel name="front_right" x="0.20" y="${-wheel_offset_y}"/>
  <xacro:wheel name="rear_left" x="-0.20" y="${wheel_offset_y}"/>
  <xacro:wheel name="rear_right" x="-0.20" y="${-wheel_offset_y}"/>

  <!-- RPLidar A1 -->
  <link name="laser">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.040"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.25 0 0.10" rpy="0 0 0"/>  <!-- Front center, elevated -->
  </joint>

  <!-- RealSense D435 -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.090 0.025 0.025"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.28 0 0.08" rpy="0 0.1 0"/>  <!-- Slight downward tilt -->
  </joint>

  <!-- IMU (BNO055) -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>  <!-- Center of chassis -->
  </joint>

  <!-- GPS -->
  <link name="gps_link"/>
  <joint name="gps_joint" type="fixed">
    <parent link="base_link"/>
    <child link="gps_link"/>
    <origin xyz="-0.20 0 0.20" rpy="0 0 0"/>  <!-- Rear, elevated on mast -->
  </joint>

</robot>
```

---

## 4. Sensor Launch Files

### 4.1 RPLidar Launch

Create `big_roverbot/launch/rplidar.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),
    ])
```

### 4.2 RealSense Launch

Create `big_roverbot/launch/realsense.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_gyro': False,
                'enable_accel': False,
                'pointcloud.enable': True,
                'align_depth.enable': True,
            }],
            output='screen',
        ),
    ])
```

### 4.3 GPS Launch

Create `big_roverbot/launch/gps.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud': 9600,
                'frame_id': 'gps_link',
            }],
            remappings=[
                ('fix', '/gps/fix'),
            ],
            output='screen',
        ),
    ])
```

---

## 5. Robot Localization (EKF Sensor Fusion)

Create `big_roverbot/config/ekf.yaml`:

```yaml
# Extended Kalman Filter configuration
# Fuses wheel odometry + IMU + GPS

ekf_filter_node_odom:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true  # Ground robot

    # Output frame
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Wheel odometry
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    odom0_differential: false
    odom0_relative: false

    # IMU
    imu0: /imu/data
    imu0_config: [false, false, false,  # x, y, z
                  true,  true,  true,   # roll, pitch, yaw
                  false, false, false,  # vx, vy, vz
                  true,  true,  true,   # vroll, vpitch, vyaw
                  true,  true,  true]   # ax, ay, az
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    # Process noise (tune based on testing)
    process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]

# GPS integration (separate node for outdoor use)
ekf_filter_node_map:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: map

    # Use filtered odom from first EKF
    odom0: /odometry/filtered
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false

    # GPS fix
    odom1: /odometry/gps
    odom1_config: [true,  true,  false,
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_differential: false

# Convert GPS to odometry
navsat_transform_node:
  ros__parameters:
    frequency: 30.0
    delay: 3.0
    magnetic_declination_radians: 0.0  # Set for your location
    yaw_offset: 0.0
    zero_altitude: true
    broadcast_utm_transform: true
    publish_filtered_gps: true
    use_odometry_yaw: true
    wait_for_datum: false
```

---

## 6. SLAM Configuration

### 6.1 slam_toolbox (2D SLAM - Primary)

Create `big_roverbot/config/slam_toolbox.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS params
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping  # or 'localization' when using saved map

    # Processing
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05  # 5cm resolution
    max_laser_range: 12.0  # RPLidar A1 max range
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000

    # General params
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation params
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop closure params
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan matcher params
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

### 6.2 RTABMAP (3D SLAM - When using D435)

Create `big_roverbot/config/rtabmap.yaml`:

```yaml
rtabmap_ros:
  ros__parameters:
    frame_id: base_link
    odom_frame_id: odom
    map_frame_id: map
    subscribe_depth: true
    subscribe_rgb: true
    subscribe_scan: true
    approx_sync: true
    queue_size: 10

    # Topic remappings handled in launch file

    # RTABMAP parameters
    Mem/IncrementalMemory: "true"
    Mem/InitWMWithAllNodes: "false"

    # Visual features
    Vis/MinInliers: "15"
    Vis/InlierDistance: "0.1"

    # Loop closure
    Rtabmap/DetectionRate: "1"
    RGBD/NeighborLinkRefining: "true"
    RGBD/ProximityBySpace: "true"
    RGBD/OptimizeFromGraphEnd: "false"

    # Grid map from lidar
    Grid/FromDepth: "false"
    Grid/RangeMax: "12.0"
    Grid/CellSize: "0.05"

    # 3D map
    Grid/3D: "true"
    GridGlobal/MinSize: "20.0"
```

---

## 7. Nav2 Configuration

Create `big_roverbot/config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.15
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
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.25

    # DWB controller (good for skid-steer)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.4       # ~1 mph for safety
      max_vel_y: 0.0       # Skid steer, no lateral
      max_vel_theta: 0.8
      min_speed_xy: 0.0
      max_speed_xy: 0.4
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_y: 0.0
      decel_lim_theta: -2.0
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.15
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
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      # Robot footprint (rectangular, 24" x 12")
      robot_radius: 0.35  # Approximation for circular
      # Or use footprint for rectangle:
      # footprint: "[[0.305, 0.152], [0.305, -0.152], [-0.305, -0.152], [-0.305, 0.152]]"
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
        observation_sources: scan depth
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 12.0
          raytrace_min_range: 0.0
          obstacle_max_range: 11.0
          obstacle_min_range: 0.0
        depth:
          topic: /camera/depth/color/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.05
          clearing: True
          marking: True
          data_type: "PointCloud2"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.35
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
          raytrace_max_range: 12.0
          raytrace_min_range: 0.0
          obstacle_max_range: 11.0
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
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: false
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.8
    min_rotational_vel: 0.2
    rotational_acc_lim: 2.0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.4, 0.0, 0.8]
    min_velocity: [-0.4, 0.0, -0.8]
    max_accel: [1.0, 0.0, 2.0]
    max_decel: [-1.0, 0.0, -2.0]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

---

## 8. Main Launch File

Create `big_roverbot/launch/big_roverbot.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('big_roverbot')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_mode = LaunchConfiguration('slam', default='true')
    nav_mode = LaunchConfiguration('nav', default='true')

    # Process URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'big_roverbot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav', default_value='true'),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
        ),

        # RealSense D435
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'pointcloud.enable': True,
                'align_depth.enable': True,
            }],
        ),

        # Robot localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),

        # SLAM Toolbox
        Node(
            condition=IfCondition(slam_mode),
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')],
        ),

        # Nav2 (when navigation enabled)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ]),
            condition=IfCondition(nav_mode),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            }.items(),
        ),
    ])
```

---

## 9. Arduino Interface

The Arduino Mega handles:
- Motor PWM control (Cytron MDD10A)
- Encoder reading (4x quadrature)
- IMU reading (BNO055 via I2C)
- Publishing odometry and IMU to ROS2

### Option A: rosserial (deprecated but simple)

```cpp
// Arduino sketch - simplified example
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher odom_pub("/wheel_odom", &odom_msg);
ros::Publisher imu_pub("/imu/data", &imu_msg);

void cmdVelCallback(const geometry_msgs::Twist& msg) {
    // Convert twist to motor commands
    float linear = msg.linear.x;
    float angular = msg.angular.z;
    // ... set motor PWM
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("/cmd_vel", cmdVelCallback);

void setup() {
    nh.initNode();
    nh.advertise(odom_pub);
    nh.advertise(imu_pub);
    nh.subscribe(cmd_sub);
}

void loop() {
    // Read encoders, compute odometry
    // Read IMU
    // Publish messages
    nh.spinOnce();
}
```

### Option B: micro-ROS (recommended for ROS2)

Uses micro-ROS agent on Pi to bridge Arduino to ROS2 natively.

```bash
# Run micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

---

## 10. Quick Start Commands

```bash
# Terminal 1: Launch robot
ros2 launch big_roverbot big_roverbot.launch.py

# Terminal 2: Teleop (keyboard control)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Visualize in RViz2
rviz2 -d ~/big_roverbot_ws/src/big_roverbot/config/big_roverbot.rviz

# Save map after SLAM
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Load map for navigation (no SLAM)
ros2 launch big_roverbot big_roverbot.launch.py slam:=false map:=~/maps/my_map.yaml
```

---

## 11. Sensor Topic Summary

| Topic | Type | Source |
|-------|------|--------|
| `/scan` | LaserScan | RPLidar A1 |
| `/camera/color/image_raw` | Image | RealSense D435 |
| `/camera/depth/image_rect_raw` | Image | RealSense D435 |
| `/camera/depth/color/points` | PointCloud2 | RealSense D435 |
| `/imu/data` | Imu | BNO055 via Arduino |
| `/wheel_odom` | Odometry | Encoders via Arduino |
| `/gps/fix` | NavSatFix | TN GPS |
| `/odometry/filtered` | Odometry | robot_localization EKF |
| `/cmd_vel` | Twist | Teleop / Nav2 |
| `/map` | OccupancyGrid | slam_toolbox |

---

## 12. Tuning Checklist

- [ ] Calibrate wheel odometry (measure actual vs reported distance)
- [ ] Calibrate IMU (BNO055 auto-calibrates, but verify)
- [ ] Set magnetic declination for GPS
- [ ] Tune EKF covariances based on sensor quality
- [ ] Adjust Nav2 velocity limits for your terrain
- [ ] Test SLAM in small area first, then expand
- [ ] Verify TF tree with `ros2 run tf2_tools view_frames`

---

*Created: 2024-12-28*

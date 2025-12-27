# ROS2 Guide

Robot Operating System 2 - middleware framework for building robot software.

## What is ROS2?

ROS2 is **not** an operating system. It's a set of:
- Communication protocols (publish/subscribe, services, actions)
- Standard message formats
- Tools for visualization, simulation, debugging
- Reusable packages (navigation, SLAM, control)

## Why ROS2?

### Without ROS2
```
Camera Code --> Your Custom Glue --> Motor Code
    |                                    |
    +---> Your Custom Protocol ----------+
    |                                    |
Sensor Code --> More Custom Glue --> Planner Code

Result: Rewrite everything for each robot
```

### With ROS2
```
Camera Node --+
              |
Sensor Node --+--> ROS2 Message Bus --> Planner Node --> Motor Node
              |
Lidar Node ---+

Result: Swap components, reuse code, standard interfaces
```

## Core Concepts

### 1. Nodes

Independent processes that do one thing well:

```
/camera_node        - Publishes images
/detector_node      - Subscribes to images, publishes detections
/motor_node         - Subscribes to velocity commands
/planner_node       - Subscribes to detections, publishes commands
/lidar_node         - Publishes laser scans
/slam_node          - Subscribes to scans, publishes map
```

List running nodes:
```bash
ros2 node list
```

### 2. Topics

Named message buses for publish/subscribe communication:

```
/camera/image_raw       [sensor_msgs/Image]
/scan                   [sensor_msgs/LaserScan]
/detections             [vision_msgs/Detection2DArray]
/cmd_vel                [geometry_msgs/Twist]
/odom                   [nav_msgs/Odometry]
/map                    [nav_msgs/OccupancyGrid]
```

Topic commands:
```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /cmd_vel

# Echo messages (live view)
ros2 topic echo /cmd_vel

# Publish a message
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```

### 3. Messages

Structured data types with defined fields:

```python
# geometry_msgs/Twist - velocity command
linear:
  x: 0.5   # forward m/s
  y: 0.0   # sideways m/s (holonomic only)
  z: 0.0   # vertical m/s (flying robots)
angular:
  x: 0.0   # roll rate
  y: 0.0   # pitch rate
  z: 0.2   # yaw rate (turning)
```

```python
# sensor_msgs/LaserScan - lidar data
angle_min: -3.14
angle_max: 3.14
ranges: [1.2, 1.3, 1.5, ...]  # distances at each angle
```

Show message definition:
```bash
ros2 interface show geometry_msgs/Twist
```

### 4. Services

Synchronous request/response (like function calls):

```
Client                          Server
   |                               |
   |-- Request: {filename} ------->|
   |                               | (processes)
   |<-- Response: {success} -------|
```

```bash
# List services
ros2 service list

# Call a service
ros2 service call /take_snapshot std_srvs/Trigger
```

### 5. Actions

Long-running tasks with feedback:

```
Client                          Server
   |                               |
   |-- Goal: {x: 5, y: 3} -------->|
   |                               |
   |<-- Feedback: {dist: 4.2} -----|
   |<-- Feedback: {dist: 3.1} -----|
   |<-- Feedback: {dist: 1.5} -----|
   |                               |
   |<-- Result: {success: true} ---|
```

Used for navigation goals, arm movements, etc.

### 6. Parameters

Runtime configuration for nodes:

```bash
# List parameters
ros2 param list /camera_node

# Get parameter value
ros2 param get /camera_node frame_rate

# Set parameter
ros2 param set /camera_node frame_rate 30
```

### 7. Launch Files

Start multiple nodes with configuration:

```python
# my_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='camera_node',
            parameters=[{'frame_rate': 30}]
        ),
        Node(
            package='my_robot',
            executable='motor_node',
        ),
        Node(
            package='my_robot',
            executable='detector_node',
        ),
    ])
```

```bash
ros2 launch my_robot my_robot.launch.py
```

## Architecture Example

```
+------------------+     +------------------+     +------------------+
|   camera_node    |     |  detector_node   |     |  planner_node    |
|                  |     |                  |     |                  |
| Publishes:       |     | Subscribes:      |     | Subscribes:      |
| /image_raw  -----+---->| /image_raw       |     | /detections      |
|                  |     |                  |     | /scan            |
|                  |     | Publishes:       |     | /odom            |
|                  |     | /detections -----+---->|                  |
+------------------+     +------------------+     | Publishes:       |
                                                  | /cmd_vel    -----+--+
+------------------+     +------------------+     +------------------+  |
|   lidar_node     |     |    slam_node     |                          |
|                  |     |                  |                          |
| Publishes:       |     | Subscribes:      |                          |
| /scan       -----+---->| /scan            |                          |
|                  |     | /odom            |                          |
+------------------+     |                  |     +------------------+  |
                         | Publishes:       |     |   motor_node     |<-+
+------------------+     | /map             |     |                  |
|   odom_node      |---->|                  |     | Subscribes:      |
|                  |     +------------------+     | /cmd_vel         |
| Publishes:       |                              |                  |
| /odom            |                              | Controls:        |
| /tf (transforms) |                              | Physical motors  |
+------------------+                              +------------------+
```

## ROS2 Versions

| Version | Release | Support Until | Ubuntu |
|---------|---------|---------------|--------|
| Humble | May 2022 | May 2027 | 22.04 |
| Iron | May 2023 | Nov 2024 | 22.04 |
| **Jazzy** | May 2024 | May 2029 | 24.04 |
| Rolling | Continuous | Dev only | Latest |

**Recommendation:** Jazzy (latest LTS, best for new projects)

## Installation (Ubuntu 24.04)

```bash
# Set up sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Desktop (full)
sudo apt update
sudo apt install ros-jazzy-desktop

# Install development tools
sudo apt install ros-dev-tools

# Source ROS2 (add to .bashrc for permanent)
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## Verify Installation

```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run listener
ros2 run demo_nodes_cpp listener

# Should see messages being exchanged
```

## Essential Tools

### rqt_graph - Visualize node connections
```bash
ros2 run rqt_graph rqt_graph
```

### RViz2 - 3D visualization
```bash
rviz2
```

### ros2 bag - Record and playback
```bash
# Record all topics
ros2 bag record -a -o my_recording

# Playback
ros2 bag play my_recording
```

## Common Packages

### Navigation (Nav2)
Autonomous navigation with obstacle avoidance:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### SLAM
Build maps while navigating:
```bash
sudo apt install ros-jazzy-slam-toolbox
```

### Robot Localization
Sensor fusion for odometry:
```bash
sudo apt install ros-jazzy-robot-localization
```

### MoveIt2
Robot arm motion planning:
```bash
sudo apt install ros-jazzy-moveit
```

### Gazebo
Physics simulation:
```bash
sudo apt install ros-jazzy-ros-gz
```

## Minimal Python Example

```python
#!/usr/bin/env python3
"""
simple_publisher.py - Publishes velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher
        self.publisher = self.create_publisher(
            Twist,           # Message type
            '/cmd_vel',      # Topic name
            10               # Queue size
        )

        # Create timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Publisher started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2   # Forward 0.2 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = SimplePublisher()

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
#!/usr/bin/env python3
"""
simple_subscriber.py - Subscribes to laser scan data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Subscriber started')

    def scan_callback(self, msg):
        # Find minimum distance
        min_dist = min(msg.ranges)
        self.get_logger().info(f'Nearest obstacle: {min_dist:.2f}m')

def main():
    rclpy.init()
    node = SimpleSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot \
  --dependencies rclpy geometry_msgs sensor_msgs

# Package structure
my_robot/
  my_robot/
    __init__.py
    my_node.py
  resource/
    my_robot
  test/
  package.xml
  setup.py
  setup.cfg

# Build
cd ~/ros2_ws
colcon build

# Source
source install/setup.bash

# Run
ros2 run my_robot my_node
```

## TF2 - Transforms

Coordinate frame transformations:

```
       map
        |
      odom
        |
    base_link
     /     \
left_wheel  right_wheel
        |
     camera
```

```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_link
```

## Standard Message Types

| Package | Message | Use |
|---------|---------|-----|
| geometry_msgs | Twist | Velocity commands |
| geometry_msgs | Pose | Position + orientation |
| sensor_msgs | Image | Camera images |
| sensor_msgs | LaserScan | 2D lidar |
| sensor_msgs | PointCloud2 | 3D lidar/depth |
| sensor_msgs | Imu | Accelerometer/gyro |
| nav_msgs | Odometry | Position from encoders |
| nav_msgs | OccupancyGrid | 2D map |
| nav_msgs | Path | Sequence of poses |

## Learning Path

```
Week 1: Basics
  - Install ROS2 Jazzy
  - ros2 topic, ros2 node commands
  - Write publisher/subscriber
  - TurtleSim tutorial

Week 2: Your Robot
  - Create package for your robot
  - Motor control node
  - Sensor nodes (camera, IMU)
  - Launch file

Week 3: Visualization
  - RViz2 configuration
  - URDF robot model
  - TF2 transforms

Week 4: Navigation
  - Install Nav2
  - Map your space (SLAM)
  - Autonomous navigation

Week 5+: Advanced
  - Custom behaviors
  - Multi-robot coordination
  - Simulation with Gazebo
```

## Resources

- Official Tutorials: https://docs.ros.org/en/jazzy/Tutorials.html
- Nav2 Documentation: https://docs.nav2.org/
- ROS2 Design: https://design.ros2.org/
- ROS Answers: https://answers.ros.org/
- The Construct (courses): https://www.theconstructsim.com/

## Comparison to Project 1 (Pi 5 Crawler)

| Aspect | Project 1 (Custom) | Project 2 (ROS2) |
|--------|-------------------|------------------|
| Communication | Direct function calls | Topics/services |
| Visualization | Custom cv2 overlay | RViz2 (3D) |
| Recording | None | rosbag2 |
| Navigation | Custom avoidance | Nav2 (path planning) |
| Mapping | None | slam_toolbox |
| Simulation | --simulate flag | Gazebo (physics) |
| Modularity | Tightly coupled | Swap any component |
| Learning curve | Lower | Higher |
| Flexibility | Limited | Extensive |

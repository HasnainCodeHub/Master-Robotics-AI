---
id: chapter-3-navigation
title: "Chapter 3: Navigation System"
sidebar_label: "3. Navigation"
sidebar_position: 4
---

# Chapter 3: Navigation System Implementation

## Chapter Goal

By the end of this chapter, you will have **implemented a complete navigation system** using VSLAM for localization, Nav2 for path planning, and dynamic obstacle avoidance for safe autonomous navigation.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 3.1 | Configure VSLAM for robot localization |
| 3.2 | Set up Nav2 for path planning and execution |
| 3.3 | Implement dynamic obstacle avoidance |
| 3.4 | Create ROS 2 action interfaces for navigation |
| 3.5 | Handle navigation failures and recovery |

---

## Navigation Architecture {#architecture}

### System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      NAVIGATION SYSTEM                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│  │   VSLAM     │───►│   Nav2      │───►│   Local     │                 │
│  │ Localization│    │  Planner    │    │  Controller │                 │
│  └─────────────┘    └─────────────┘    └─────────────┘                 │
│        ▲                  ▲                  │                          │
│        │                  │                  ▼                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│  │  Stereo +   │    │  Costmap    │    │   cmd_vel   │                 │
│  │    IMU      │    │  (LiDAR)    │    │   Output    │                 │
│  └─────────────┘    └─────────────┘    └─────────────┘                 │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Data Flow

| Stage | Input | Output | Rate |
|-------|-------|--------|------|
| VSLAM | Stereo images, IMU | Odometry, pose | 30 Hz |
| Costmap | LiDAR scan, pose | Occupancy grid | 10 Hz |
| Global Planner | Goal, map | Path | On request |
| Local Planner | Path, costmap | cmd_vel | 20 Hz |

---

## VSLAM Configuration {#vslam}

### Isaac ROS Visual SLAM

```yaml
# config/visual_slam_params.yaml

visual_slam_node:
  ros__parameters:
    # Camera configuration
    enable_rectified_pose: true
    enable_image_denoising: false

    # IMU fusion
    enable_imu_fusion: true
    gyro_noise_density: 0.00016       # rad/s/sqrt(Hz)
    gyro_random_walk: 0.000022        # rad/s^2/sqrt(Hz)
    accel_noise_density: 0.0017       # m/s^2/sqrt(Hz)
    accel_random_walk: 0.00019        # m/s^3/sqrt(Hz)
    calibration_frequency: 200.0      # IMU rate

    # Performance tuning
    enable_verbosity: false
    force_planar_mode: false
    enable_observations_view: false
    enable_landmarks_view: false

    # Output configuration
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    publish_odom_to_base_tf: true
    publish_map_to_odom_tf: true
```

### VSLAM Launch

```python
# launch/vslam.launch.py

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='vslam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    name='visual_slam',
                    parameters=['config/visual_slam_params.yaml'],
                    remappings=[
                        ('stereo_camera/left/image', '/camera/left/image'),
                        ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
                        ('stereo_camera/right/image', '/camera/right/image'),
                        ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
                        ('visual_slam/imu', '/imu'),
                    ]
                ),
            ],
            output='screen',
        ),
    ])
```

### VSLAM Monitoring

```python
#!/usr/bin/env python3
"""Monitor VSLAM health and accuracy."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


class VSLAMMonitor(Node):
    """Monitor VSLAM performance."""

    def __init__(self):
        super().__init__('vslam_monitor')

        # State
        self.pose_history = []
        self.velocity_history = []
        self.last_pose_time = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        # Health check timer
        self.create_timer(1.0, self.health_check)

        # Thresholds
        self.max_velocity = 2.0  # m/s - suspicious if exceeded
        self.stale_threshold = 1.0  # seconds

    def odom_callback(self, msg: Odometry):
        """Process odometry update."""
        current_time = self.get_clock().now()

        # Extract pose
        pose = msg.pose.pose
        position = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])

        # Extract velocity
        velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        # Store
        self.pose_history.append({
            'time': current_time,
            'position': position
        })
        self.velocity_history.append(np.linalg.norm(velocity))

        # Keep limited history
        if len(self.pose_history) > 100:
            self.pose_history.pop(0)
            self.velocity_history.pop(0)

        self.last_pose_time = current_time

    def health_check(self):
        """Check VSLAM health."""
        current_time = self.get_clock().now()

        # Check for stale data
        if self.last_pose_time is not None:
            age = (current_time - self.last_pose_time).nanoseconds / 1e9
            if age > self.stale_threshold:
                self.get_logger().warn(f'VSLAM stale: {age:.2f}s since last update')
                return

        # Check velocity sanity
        if self.velocity_history:
            max_vel = max(self.velocity_history[-10:])
            if max_vel > self.max_velocity:
                self.get_logger().warn(f'Suspicious velocity: {max_vel:.2f} m/s')

        # Check tracking stability
        if len(self.pose_history) >= 10:
            positions = [p['position'] for p in self.pose_history[-10:]]
            variance = np.var(positions, axis=0)
            if np.any(variance > 1.0):  # High variance indicates instability
                self.get_logger().warn(f'High pose variance: {variance}')

        self.get_logger().debug('VSLAM health: OK')
```

---

## Nav2 Configuration {#nav2}

### Navigation Parameters

```yaml
# config/nav2_params.yaml

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 0.1
      stateful: true

    # DWB controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.5
      decel_lim_y: 0.0
      decel_lim_theta: -2.5
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.5
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.1
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.35
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.35
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### Nav2 Launch

```python
# launch/navigation.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(
        get_package_share_directory('humanoid_navigation'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        # Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'true',
            }.items()
        ),

        # Navigation action server wrapper
        Node(
            package='humanoid_navigation',
            executable='navigation_action_server',
            name='navigation_action_server',
            parameters=[{'use_sim_time': True}],
        ),
    ])
```

---

## Navigation Action Server {#action-server}

### Custom Navigation Interface

```python
#!/usr/bin/env python3
"""Navigation action server for humanoid system."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json


class NavigationActionServer(Node):
    """Wrap Nav2 with humanoid-specific interface."""

    def __init__(self):
        super().__init__('navigation_action_server')

        self.callback_group = ReentrantCallbackGroup()

        # Nav2 action client
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # Custom action server for VLA integration
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'humanoid/navigate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            String, '/navigation/status', 10
        )

        # State
        self.current_goal = None
        self.nav2_goal_handle = None

        self.get_logger().info('Navigation action server ready')

    def goal_callback(self, goal_request):
        """Accept or reject incoming goal."""
        self.get_logger().info('Received navigation goal')

        # Validate goal
        if not self.is_valid_goal(goal_request.pose):
            self.get_logger().warn('Invalid goal rejected')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancel request."""
        self.get_logger().info('Cancel requested')

        # Cancel Nav2 goal if active
        if self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()

        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute navigation goal."""
        self.get_logger().info('Executing navigation goal')
        self.current_goal = goal_handle.request.pose

        # Wait for Nav2
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server not available')
            goal_handle.abort()
            return NavigateToPose.Result()

        # Send to Nav2
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.pose

        self.publish_status('navigating', 'Starting navigation')

        # Send goal and wait
        send_goal_future = self.nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=self.nav2_feedback_callback
        )

        # Wait for goal acceptance
        self.nav2_goal_handle = await send_goal_future

        if not self.nav2_goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal')
            self.publish_status('failed', 'Goal rejected by planner')
            goal_handle.abort()
            return NavigateToPose.Result()

        # Wait for result
        result_future = self.nav2_goal_handle.get_result_async()
        result = await result_future

        # Process result
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
            self.publish_status('succeeded', 'Reached goal')
            goal_handle.succeed()
        elif result.status == 5:  # CANCELED
            self.get_logger().info('Navigation canceled')
            self.publish_status('canceled', 'Navigation canceled')
            goal_handle.canceled()
        else:
            self.get_logger().warn(f'Navigation failed with status {result.status}')
            self.publish_status('failed', f'Navigation failed: {result.status}')
            goal_handle.abort()

        return NavigateToPose.Result()

    def nav2_feedback_callback(self, feedback_msg):
        """Forward Nav2 feedback."""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining

        self.publish_status(
            'navigating',
            f'Distance remaining: {distance:.2f}m'
        )

    def is_valid_goal(self, pose: PoseStamped) -> bool:
        """Validate navigation goal."""
        # Check bounds
        x, y = pose.pose.position.x, pose.pose.position.y

        # Workspace limits (should come from params)
        if not (-10 < x < 10 and -10 < y < 10):
            return False

        return True

    def publish_status(self, state: str, message: str):
        """Publish navigation status."""
        status = {
            'state': state,
            'message': message,
            'goal': {
                'x': self.current_goal.pose.position.x if self.current_goal else None,
                'y': self.current_goal.pose.position.y if self.current_goal else None,
            }
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavigationActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Dynamic Obstacle Avoidance {#obstacle-avoidance}

### Obstacle Detection and Tracking

```python
#!/usr/bin/env python3
"""Dynamic obstacle tracking for navigation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN


class ObstacleTracker(Node):
    """Track and predict dynamic obstacles."""

    def __init__(self):
        super().__init__('obstacle_tracker')

        # Tracking state
        self.tracked_obstacles = {}
        self.next_id = 0

        # Parameters
        self.cluster_eps = 0.3  # DBSCAN epsilon
        self.cluster_min_samples = 5
        self.max_tracking_distance = 0.5
        self.prediction_horizon = 1.0  # seconds

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/obstacles/markers', 10
        )

        # Timer for obstacle prediction
        self.create_timer(0.1, self.predict_callback)

    def scan_callback(self, msg: LaserScan):
        """Process laser scan for obstacles."""
        # Convert to Cartesian points
        points = self.scan_to_points(msg)

        if len(points) < self.cluster_min_samples:
            return

        # Cluster points
        clusters = self.cluster_points(points)

        # Update tracking
        self.update_tracking(clusters)

        # Publish visualization
        self.publish_markers()

    def scan_to_points(self, scan: LaserScan) -> np.ndarray:
        """Convert laser scan to 2D points."""
        angles = np.arange(
            scan.angle_min,
            scan.angle_max,
            scan.angle_increment
        )

        ranges = np.array(scan.ranges)

        # Filter invalid ranges
        valid = (ranges > scan.range_min) & (ranges < scan.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        return np.column_stack([x, y])

    def cluster_points(self, points: np.ndarray) -> list:
        """Cluster points into obstacles."""
        clustering = DBSCAN(
            eps=self.cluster_eps,
            min_samples=self.cluster_min_samples
        ).fit(points)

        clusters = []
        for label in set(clustering.labels_):
            if label == -1:  # Noise
                continue

            cluster_points = points[clustering.labels_ == label]
            centroid = np.mean(cluster_points, axis=0)
            radius = np.max(np.linalg.norm(cluster_points - centroid, axis=1))

            clusters.append({
                'centroid': centroid,
                'radius': radius,
                'points': cluster_points
            })

        return clusters

    def update_tracking(self, clusters: list):
        """Update obstacle tracking with new detections."""
        current_time = self.get_clock().now()

        # Match clusters to tracked obstacles
        matched = set()

        for cluster in clusters:
            centroid = cluster['centroid']
            best_match = None
            best_distance = self.max_tracking_distance

            for obs_id, obs in self.tracked_obstacles.items():
                distance = np.linalg.norm(centroid - obs['position'])
                if distance < best_distance:
                    best_match = obs_id
                    best_distance = distance

            if best_match is not None:
                # Update existing obstacle
                obs = self.tracked_obstacles[best_match]
                dt = (current_time - obs['last_seen']).nanoseconds / 1e9

                # Calculate velocity
                if dt > 0:
                    velocity = (centroid - obs['position']) / dt
                    # Exponential smoothing
                    obs['velocity'] = 0.7 * obs['velocity'] + 0.3 * velocity

                obs['position'] = centroid
                obs['radius'] = cluster['radius']
                obs['last_seen'] = current_time
                matched.add(best_match)

            else:
                # New obstacle
                self.tracked_obstacles[self.next_id] = {
                    'position': centroid,
                    'velocity': np.zeros(2),
                    'radius': cluster['radius'],
                    'last_seen': current_time
                }
                self.next_id += 1

        # Remove stale obstacles
        stale_threshold = 1.0  # seconds
        stale_ids = []
        for obs_id, obs in self.tracked_obstacles.items():
            age = (current_time - obs['last_seen']).nanoseconds / 1e9
            if age > stale_threshold:
                stale_ids.append(obs_id)

        for obs_id in stale_ids:
            del self.tracked_obstacles[obs_id]

    def predict_callback(self):
        """Predict future obstacle positions."""
        for obs_id, obs in self.tracked_obstacles.items():
            # Predict position at horizon
            predicted = obs['position'] + obs['velocity'] * self.prediction_horizon
            obs['predicted_position'] = predicted

    def publish_markers(self):
        """Publish obstacle visualization."""
        marker_array = MarkerArray()

        for obs_id, obs in self.tracked_obstacles.items():
            # Current position marker
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = obs_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs['position'][0]
            marker.pose.position.y = obs['position'][1]
            marker.pose.position.z = 0.5
            marker.scale.x = obs['radius'] * 2
            marker.scale.y = obs['radius'] * 2
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.a = 0.5
            marker_array.markers.append(marker)

            # Velocity vector marker
            if np.linalg.norm(obs['velocity']) > 0.1:
                vel_marker = Marker()
                vel_marker.header = marker.header
                vel_marker.ns = 'velocities'
                vel_marker.id = obs_id
                vel_marker.type = Marker.ARROW
                vel_marker.action = Marker.ADD

                start = Point()
                start.x = obs['position'][0]
                start.y = obs['position'][1]
                start.z = 0.5

                end = Point()
                end.x = obs['position'][0] + obs['velocity'][0]
                end.y = obs['position'][1] + obs['velocity'][1]
                end.z = 0.5

                vel_marker.points = [start, end]
                vel_marker.scale.x = 0.05
                vel_marker.scale.y = 0.1
                vel_marker.color.g = 1.0
                vel_marker.color.a = 0.8
                marker_array.markers.append(vel_marker)

        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = ObstacleTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Navigation Recovery Behaviors {#recovery}

### Recovery Behavior Implementation

```python
#!/usr/bin/env python3
"""Navigation recovery behaviors."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class NavigationRecovery(Node):
    """Implement navigation recovery behaviors."""

    def __init__(self):
        super().__init__('navigation_recovery')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        """Track current pose."""
        self.current_pose = msg.pose.pose

    def spin_recovery(self, angle: float = 2 * np.pi, angular_speed: float = 0.5):
        """Rotate in place to clear costmap."""
        self.get_logger().info(f'Executing spin recovery: {np.degrees(angle):.0f} degrees')

        duration = abs(angle / angular_speed)
        direction = 1 if angle > 0 else -1

        twist = Twist()
        twist.angular.z = direction * angular_speed

        start_time = time.time()
        rate = self.create_rate(20)

        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Spin recovery complete')

    def back_up_recovery(self, distance: float = 0.3, speed: float = 0.1):
        """Back up to clear obstacle."""
        self.get_logger().info(f'Executing backup recovery: {distance:.2f}m')

        duration = distance / speed

        twist = Twist()
        twist.linear.x = -speed

        start_time = time.time()
        rate = self.create_rate(20)

        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Backup recovery complete')

    def wait_recovery(self, duration: float = 5.0):
        """Wait for dynamic obstacle to clear."""
        self.get_logger().info(f'Waiting for obstacle to clear: {duration:.1f}s')

        # Stop and wait
        self.cmd_vel_pub.publish(Twist())
        time.sleep(duration)

        self.get_logger().info('Wait recovery complete')

    def execute_recovery_sequence(self):
        """Execute standard recovery sequence."""
        self.get_logger().info('Starting recovery sequence')

        # 1. Wait briefly
        self.wait_recovery(2.0)

        # 2. Spin to update costmap
        self.spin_recovery(np.pi)

        # 3. If still stuck, back up
        # (Would check if still blocked here)
        self.back_up_recovery(0.3)

        # 4. Spin again
        self.spin_recovery(-np.pi / 2)

        self.get_logger().info('Recovery sequence complete')
```

---

## Summary

This chapter covered navigation system implementation:

1. **VSLAM configuration** with Isaac ROS for robust visual-inertial localization.

2. **Nav2 setup** with tuned parameters for costmap, planner, and controller.

3. **Custom action server** wrapping Nav2 for VLA integration.

4. **Dynamic obstacle tracking** using clustering and velocity estimation.

5. **Recovery behaviors** for handling navigation failures.

---

## Navigation Checklist

Before proceeding, verify:

- [ ] VSLAM produces stable odometry
- [ ] Costmaps update with LiDAR data
- [ ] Global planner generates valid paths
- [ ] Local planner follows paths smoothly
- [ ] Obstacle avoidance prevents collisions
- [ ] Recovery behaviors execute when stuck

---

## What's Next

In [Chapter 4: Manipulation System](/capstone/chapter-4-manipulation), you'll implement dual-arm manipulation with MoveIt 2 for pick and place operations.

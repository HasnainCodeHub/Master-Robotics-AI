---
id: chapter-4-visual-slam
title: "Chapter 4: Visual SLAM"
sidebar_label: "4. Visual SLAM"
sidebar_position: 5
---

# Chapter 4: Visual SLAM with Isaac ROS

## Chapter Goal

By the end of this chapter, you will be able to **implement Visual SLAM using Isaac ROS packages** for robust robot localization, understanding the algorithms, configuration, and evaluation methodology.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 4.1 | Explain Visual SLAM concepts: feature tracking, bundle adjustment, loop closure |
| 4.2 | Configure Isaac ROS Visual SLAM for stereo cameras |
| 4.3 | Tune VSLAM parameters for different environments |
| 4.4 | Evaluate VSLAM accuracy using ground truth comparison |
| 4.5 | Integrate VSLAM with ROS 2 navigation stack |

---

## Why Visual SLAM?

### The Localization Problem

Robots need to know where they are. Traditional approaches:

| Method | Sensors | Limitations |
|--------|---------|-------------|
| Wheel odometry | Encoders | Drift, slip |
| GPS | Receiver | Indoor failure |
| Beacon-based | Infrastructure | Requires setup |
| LiDAR SLAM | LiDAR | Expensive sensor |
| **Visual SLAM** | Camera | Works anywhere with texture |

Visual SLAM uses cameras—already present on most robots—for localization and mapping.

### Visual SLAM Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    Visual SLAM Pipeline                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Stereo Cameras ──► Feature Detection ──► Feature Matching      │
│       │                    │                    │                │
│       └── Left/Right      └── ORB, FAST       └── Between frames│
│           images              features                           │
│                                                                  │
│  ──► Motion Estimation ──► Local Optimization ──► Loop Closure  │
│            │                      │                    │         │
│            └── Frame-to-frame    └── Bundle           └── Detect │
│                odometry              adjustment            revisit│
│                                                                  │
│  ──► Map ──► Pose Output                                        │
│        │          │                                              │
│        └── 3D    └── Transform to /odom or /map frame           │
│            points                                                │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Visual Odometry Fundamentals {#visual-odometry}

### Feature Detection

The first step: find distinctive points in images.

```
Image ──► Feature Detector ──► Keypoints + Descriptors
               │
               └── ORB, FAST, SIFT, SURF
```

**ORB Features** (used by Isaac ROS VSLAM):
- Fast to compute
- Rotation invariant
- Scale invariant
- Binary descriptors (efficient matching)

### Feature Matching

Match features between consecutive frames:

```
Frame t ──┐                    ┌── Matched Features
          ├── Feature Matcher ──┤
Frame t+1 ┘                    └── Outlier Rejection
```

**Physical Grounding**: Feature matching fails when:
- Motion blur (fast movement)
- Low texture (blank walls)
- Lighting changes
- Dynamic objects

### Motion Estimation

From matched features, estimate camera motion:

```
Matched Features ──► Essential Matrix ──► Rotation + Translation
                          │
                          └── 5-point algorithm + RANSAC
```

**Result**: Relative pose change between frames (visual odometry).

---

## SLAM vs Odometry {#slam-vs-odom}

### The Drift Problem

Visual odometry accumulates error:

```
True path:    ────────────────────────→
VO estimate:  ────────────────────────→
                      └── Small errors accumulate
                          to large drift
```

After 100m of travel, drift might be 1-5m.

### Loop Closure Solution

SLAM detects when the robot revisits a location:

```
┌──────────────────────────────────────────┐
│                                          │
│    Start ─────────────────────┐          │
│      │                        │          │
│      │     Loop Closure!      │          │
│      │     ◄──────────────────┘          │
│      │                                   │
│      └───────────────────────────────────┘
│                                          │
│  Loop closure corrects accumulated drift │
└──────────────────────────────────────────┘
```

When loop closure is detected, the entire trajectory is optimized to be consistent.

---

## Isaac ROS VSLAM Configuration {#isaac-vslam}

### Package Overview

Isaac ROS VSLAM provides GPU-accelerated visual SLAM:

- CUDA-accelerated feature extraction
- GPU-based feature matching
- Efficient pose graph optimization
- Stereo camera support

### Launch Configuration

```python
# launch/isaac_vslam.launch.py
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
                    parameters=[{
                        # Camera configuration
                        'num_cameras': 2,
                        'enable_rectified_pose': True,
                        'enable_localization_n_mapping': True,

                        # Feature detection
                        'enable_imu_fusion': False,  # True if IMU available
                        'gyro_noise_density': 0.00087,  # From M1 specs
                        'accel_noise_density': 0.039,

                        # Performance tuning
                        'image_jitter_threshold_ms': 34.0,
                        'enable_slam_visualization': True,

                        # Output frames
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                        'base_frame': 'base_link',
                    }],
                    remappings=[
                        ('stereo_camera/left/image', '/camera/left/image_raw'),
                        ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
                        ('stereo_camera/right/image', '/camera/right/image_raw'),
                        ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
                        ('visual_slam/tracking/odometry', '/odom'),
                        ('visual_slam/vis/landmarks_cloud', '/vslam/landmarks'),
                    ]
                ),
            ],
            output='screen',
        ),
    ])
```

### Key Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `num_cameras` | Stereo (2) or mono (1) | 2 |
| `enable_imu_fusion` | Fuse IMU data | True if available |
| `image_jitter_threshold_ms` | Max frame timing jitter | 34 (for 30fps) |
| `map_frame` | SLAM map frame name | "map" |
| `enable_localization_n_mapping` | Full SLAM vs VO only | True |

---

## Environment Tuning {#tuning}

### Indoor Warehouse

```python
# Well-textured indoor environment
indoor_params = {
    'enable_localization_n_mapping': True,
    'enable_slam_visualization': True,
    'path_max_size': 1000,
    'image_jitter_threshold_ms': 34.0,
}
```

### Outdoor Environment

```python
# Variable lighting, larger scale
outdoor_params = {
    'enable_localization_n_mapping': True,
    'enable_slam_visualization': False,  # Save compute
    'path_max_size': 5000,  # Longer paths
    'image_jitter_threshold_ms': 50.0,  # More tolerance
}
```

### Dynamic Environment

```python
# Moving objects present
dynamic_params = {
    'enable_localization_n_mapping': True,
    # More aggressive outlier rejection
    # (Configure in feature matching)
}
```

---

## Accuracy Evaluation {#evaluation}

### Metrics

**ATE (Absolute Trajectory Error)**: Global accuracy

```
ATE = sqrt(mean(||p_estimated - p_ground_truth||²))

p = position at each timestamp
```

**RPE (Relative Pose Error)**: Local accuracy

```
RPE = error between consecutive pose transformations
      Better for odometry evaluation
```

### Evaluation with Ground Truth

```python
#!/usr/bin/env python3
"""Evaluate VSLAM accuracy against ground truth."""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt


class VSLAMEvaluator(Node):
    def __init__(self):
        super().__init__('vslam_evaluator')

        self.vslam_poses = []
        self.gt_poses = []

        # Subscribe to VSLAM odometry
        self.vslam_sub = self.create_subscription(
            Odometry,
            '/odom',  # VSLAM output
            self.vslam_callback,
            10
        )

        # Subscribe to ground truth (from Isaac Sim)
        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth/pose',
            self.gt_callback,
            10
        )

        self.timer = self.create_timer(5.0, self.compute_metrics)

    def vslam_callback(self, msg):
        pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        self.vslam_poses.append(pose)

    def gt_callback(self, msg):
        pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.gt_poses.append(pose)

    def compute_metrics(self):
        if len(self.vslam_poses) < 10 or len(self.gt_poses) < 10:
            return

        vslam = np.array(self.vslam_poses)
        gt = np.array(self.gt_poses)

        # Align lengths (simple approach)
        min_len = min(len(vslam), len(gt))
        vslam = vslam[:min_len]
        gt = gt[:min_len]

        # Compute ATE
        errors = np.linalg.norm(vslam - gt, axis=1)
        ate = np.sqrt(np.mean(errors**2))

        # Compute drift (error at end vs start)
        drift = errors[-1] - errors[0]

        # Compute RPE (simplified)
        vslam_deltas = np.diff(vslam, axis=0)
        gt_deltas = np.diff(gt, axis=0)
        rpe = np.sqrt(np.mean(np.linalg.norm(vslam_deltas - gt_deltas, axis=1)**2))

        self.get_logger().info(f'ATE: {ate:.4f}m')
        self.get_logger().info(f'Drift: {drift:.4f}m')
        self.get_logger().info(f'RPE: {rpe:.4f}m')


def main(args=None):
    rclpy.init(args=args)
    node = VSLAMEvaluator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Expected Performance

| Environment | ATE (m) | Drift (%/m) |
|-------------|---------|-------------|
| Indoor, textured | 0.05-0.1 | 0.5-1% |
| Indoor, low texture | 0.1-0.3 | 1-3% |
| Outdoor, structured | 0.1-0.2 | 1-2% |
| Outdoor, unstructured | 0.2-0.5 | 2-5% |

---

## Navigation Integration {#nav-integration}

### Using VSLAM Odometry with Nav2

```python
# launch/navigation_with_vslam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS VSLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('isaac_vslam.launch.py')
        ),

        # Nav2 with VSLAM odometry
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            parameters=[{
                'odom_topic': '/odom',  # From VSLAM
                'use_sim_time': True,
            }],
        ),
    ])
```

### TF Tree Configuration

```
map ──► odom ──► base_link ──► sensors
 │        │
 │        └── VSLAM provides map→odom transform
 │
 └── VSLAM provides initial map frame
```

---

## Failure Modes and Recovery {#failures}

### Common Failures

| Failure Mode | Cause | Detection | Recovery |
|--------------|-------|-----------|----------|
| Tracking loss | Fast motion, blur | High RPE | Slow down, re-initialize |
| Drift | Low texture | ATE divergence | Loop closure, IMU fusion |
| Scale drift | Monocular only | GT comparison | Use stereo, wheel odometry |
| Map corruption | Dynamic objects | Inconsistent landmarks | Map reset |

### Safe Behavior on Failure

```python
class SafeVSLAMNode(Node):
    def __init__(self):
        super().__init__('safe_vslam')

        self.tracking_lost = False
        self.last_pose_time = None

    def vslam_callback(self, msg):
        current_time = self.get_clock().now()

        # Check for tracking timeout
        if self.last_pose_time is not None:
            dt = (current_time - self.last_pose_time).nanoseconds / 1e9
            if dt > 0.5:  # No update for 0.5s
                self.tracking_lost = True
                self.get_logger().warn('VSLAM tracking lost!')
                self.safe_stop()

        self.last_pose_time = current_time

    def safe_stop(self):
        # Publish zero velocity
        # Alert navigation system
        pass
```

---

## Summary

This chapter covered Visual SLAM with Isaac ROS:

1. **Visual odometry** extracts motion from feature matches between frames.

2. **SLAM** adds loop closure to correct accumulated drift, providing globally consistent localization.

3. **Isaac ROS VSLAM** provides GPU-accelerated stereo SLAM with configurable parameters.

4. **Environment tuning** adapts parameters for indoor, outdoor, and dynamic scenarios.

5. **Accuracy evaluation** uses ATE and RPE metrics compared to ground truth.

6. **Navigation integration** connects VSLAM odometry to Nav2 for autonomous navigation.

---

## Self-Assessment Questions

1. **Drift vs Loop Closure**: After 100m of travel without revisiting locations, your VSLAM shows 2m ATE. Is this a problem? What would help?

2. **Feature Failure**: Your robot enters a hallway with blank walls. VSLAM tracking fails. What sensor could help? How?

3. **Parameter Tuning**: Your indoor warehouse has fluorescent lighting that flickers at 60Hz. Camera is at 30fps. What timing-related problems might occur?

4. **Accuracy Evaluation**: Isaac Sim provides perfect ground truth. Real hardware doesn't. How would you evaluate VSLAM accuracy on real hardware?

5. **Navigation Integration**: VSLAM provides map→odom transform. Nav2 needs odom→base_link too. Where does this come from?

---

## What's Next

In [Chapter 5: Domain Randomization](/module-4/chapter-5-domain-randomization), you'll use Isaac Replicator to train robust perception systems that transfer from simulation to reality.

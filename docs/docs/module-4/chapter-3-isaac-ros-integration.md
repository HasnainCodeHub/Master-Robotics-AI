---
id: chapter-3-isaac-ros-integration
title: "Chapter 3: Isaac ROS Integration"
sidebar_label: "3. Isaac ROS"
sidebar_position: 4
---

# Chapter 3: Isaac ROS Integration

## Chapter Goal

By the end of this chapter, you will be able to **integrate Isaac Sim with ROS 2 using Isaac ROS packages**, enabling GPU-accelerated perception pipelines that process simulated or real sensor data.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 3.1 | Configure Isaac ROS bridge to connect Isaac Sim to ROS 2 topics |
| 3.2 | Explain NITROS and zero-copy GPU messaging |
| 3.3 | Configure Isaac ROS Image Pipeline for GPU-accelerated processing |
| 3.4 | Configure Isaac ROS Object Detection for real-time inference |
| 3.5 | Implement perception nodes using Isaac ROS with standard ROS 2 patterns |

---

## Isaac ROS Ecosystem Overview

Isaac ROS provides GPU-accelerated ROS 2 packages:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac ROS Architecture                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐     NITROS     ┌──────────────────────────┐   │
│  │  Isaac Sim   │ ◄────────────► │    Isaac ROS Packages    │   │
│  │  (Sensors)   │   Zero-copy    │                          │   │
│  └──────────────┘       GPU      │  - Image Pipeline        │   │
│         │                        │  - Object Detection      │   │
│         │                        │  - Visual SLAM           │   │
│         ▼                        │  - Depth Estimation      │   │
│  ┌──────────────┐                │  - Segmentation          │   │
│  │  Hardware    │                └──────────────────────────┘   │
│  │  (Real Robot)│                          │                     │
│  └──────────────┘                          │                     │
│                                            ▼                     │
│                               ┌──────────────────────────┐       │
│                               │  Standard ROS 2 Nodes    │       │
│                               │  (Your Application)      │       │
│                               └──────────────────────────┘       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Key Point**: The same Isaac ROS packages work with both Isaac Sim and real hardware.

---

## NITROS: Zero-Copy GPU Messaging {#nitros}

### The Problem

Standard ROS 2 messaging copies data:

```
GPU (Sensor) → CPU → ROS 2 Message → CPU → GPU (Processing)
     │                   │                        │
     └── Copy #1 ────────┘                        │
                         └── Copy #2 ─────────────┘
```

For HD images at 30 fps, each copy takes milliseconds—unacceptable for real-time.

### NITROS Solution

NITROS (NVIDIA Isaac Transport for ROS) keeps data on GPU:

```
GPU (Sensor) → NITROS Message → GPU (Processing)
                    │
                    └── Zero copies, GPU memory stays on GPU
```

### Using NITROS

```python
# Standard ROS 2 subscriber receives CPU data
from sensor_msgs.msg import Image
# ... receive Image, data on CPU

# NITROS-enabled subscriber receives GPU data
from isaac_ros_nitros_bridge_interfaces.msg import NitrosBridgeTensorList
# ... receive tensor on GPU, no copy needed
```

### When NITROS Matters

| Pipeline | Without NITROS | With NITROS | Speedup |
|----------|----------------|-------------|---------|
| HD image rectification | 15ms | 3ms | 5x |
| Object detection | 25ms | 8ms | 3x |
| Full perception pipeline | 50ms | 12ms | 4x |

**Physical Grounding**: At 30 fps, each frame has 33ms budget. Without NITROS, perception alone exceeds this. With NITROS, you have margin for control.

---

## Isaac ROS Bridge Configuration {#bridge}

### Enabling ROS 2 Bridge in Isaac Sim

```python
# In Isaac Sim Python script
from omni.isaac.ros2_bridge import enable_ros2_bridge

# Enable the bridge
enable_ros2_bridge()
```

### Launch Configuration

```python
# launch/isaac_sim_bridge.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Isaac Sim ROS 2 bridge is enabled in the sim
        # Configure topic mappings

        # Image topic bridge
        Node(
            package='isaac_ros_nitros_bridge_ros2',
            executable='image_converter_node',
            name='image_bridge',
            parameters=[{
                'input_topic': '/isaac_sim/camera/image',
                'output_topic': '/camera/image_raw',
            }]
        ),
    ])
```

### Verifying Bridge

```bash
# Terminal 1: Isaac Sim running with ROS bridge

# Terminal 2: Check topics
ros2 topic list

# Expected topics:
# /camera/image_raw
# /camera/camera_info
# /scan
# /imu/data
# /odom
# /cmd_vel

# Check data flow
ros2 topic hz /camera/image_raw
# Should show ~30 Hz if camera is 30 fps
```

---

## Isaac ROS Image Pipeline {#image-pipeline}

### GPU-Accelerated Image Processing

```python
# launch/image_pipeline.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='image_pipeline_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # Rectification
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                    name='rectify_node',
                    remappings=[
                        ('image_raw', '/camera/image_raw'),
                        ('camera_info', '/camera/camera_info'),
                        ('image_rect', '/camera/image_rect'),
                    ]
                ),
                # Color conversion
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                    name='format_converter',
                    parameters=[{
                        'encoding_desired': 'rgb8',
                    }],
                    remappings=[
                        ('image_raw', '/camera/image_rect'),
                        ('image', '/camera/image_rgb'),
                    ]
                ),
                # Resize for inference
                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                    name='resize_node',
                    parameters=[{
                        'output_width': 640,
                        'output_height': 480,
                    }],
                    remappings=[
                        ('image', '/camera/image_rgb'),
                        ('camera_info', '/camera/camera_info'),
                        ('resize/image', '/camera/image_resized'),
                        ('resize/camera_info', '/camera/camera_info_resized'),
                    ]
                ),
            ],
            output='screen',
        ),
    ])
```

### Performance Comparison

| Operation | CPU (ROS 2) | GPU (Isaac ROS) |
|-----------|-------------|-----------------|
| Rectification | 8ms | 1ms |
| Color conversion | 3ms | 0.5ms |
| Resize | 5ms | 0.3ms |
| **Total** | 16ms | 1.8ms |

---

## Isaac ROS Object Detection {#detection}

### Using Pre-trained Models

Isaac ROS includes optimized detection models:

```python
# launch/object_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='detection_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                # DNN Image Encoder
                ComposableNode(
                    package='isaac_ros_dnn_image_encoder',
                    plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
                    name='dnn_encoder',
                    parameters=[{
                        'input_image_width': 640,
                        'input_image_height': 480,
                        'network_image_width': 640,
                        'network_image_height': 480,
                        'image_mean': [0.5, 0.5, 0.5],
                        'image_stddev': [0.5, 0.5, 0.5],
                    }],
                    remappings=[
                        ('image', '/camera/image_resized'),
                        ('encoded_tensor', '/tensor_pub'),
                    ]
                ),
                # TensorRT Inference
                ComposableNode(
                    package='isaac_ros_tensor_rt',
                    plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                    name='tensor_rt',
                    parameters=[{
                        'model_file_path': '/models/yolov8.onnx',
                        'engine_file_path': '/models/yolov8.engine',
                        'input_tensor_names': ['images'],
                        'input_binding_names': ['images'],
                        'output_tensor_names': ['output0'],
                        'output_binding_names': ['output0'],
                    }],
                ),
                # Detection Decoder
                ComposableNode(
                    package='isaac_ros_detectnet',
                    plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
                    name='detectnet_decoder',
                    parameters=[{
                        'label_list': ['person', 'box', 'shelf'],
                        'confidence_threshold': 0.5,
                    }],
                ),
            ],
            output='screen',
        ),
    ])
```

### Detection Output

```python
# Subscribing to detections
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            # Bounding box
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            width = bbox.size_x
            height = bbox.size_y

            # Class and confidence
            for result in detection.results:
                class_id = result.hypothesis.class_id
                confidence = result.hypothesis.score

                self.get_logger().info(
                    f'Detected {class_id} at ({center_x}, {center_y}) '
                    f'with confidence {confidence:.2f}'
                )
```

---

## Building Perception Nodes {#perception-nodes}

### Complete Perception Pipeline

```python
#!/usr/bin/env python3
"""
Perception node using Isaac ROS packages.
Subscribes to camera, runs detection, publishes results.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Subscribe to Isaac ROS detection output
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        # Subscribe to depth for 3D localization
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            sensor_qos
        )

        # Publish target locations
        self.target_pub = self.create_publisher(
            PointStamped,
            '/perception/target',
            10
        )

        self.latest_depth = None

        self.get_logger().info('Perception node started')

    def depth_callback(self, msg):
        self.latest_depth = msg

    def detection_callback(self, msg):
        if self.latest_depth is None:
            return

        for detection in msg.detections:
            # Get detection center
            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)

            # Get depth at detection center
            depth = self.get_depth_at_pixel(cx, cy)

            if depth is None or depth <= 0:
                continue

            # Convert to 3D point (simplified pinhole model)
            fx = 554.25  # From camera intrinsics
            fy = 554.25
            cx_cam = 320  # Principal point
            cy_cam = 240

            x = (cx - cx_cam) * depth / fx
            y = (cy - cy_cam) * depth / fy
            z = depth

            # Publish target
            target = PointStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'camera_link'
            target.point.x = float(z)   # Camera Z → forward
            target.point.y = float(-x)  # Camera X → right
            target.point.z = float(-y)  # Camera Y → down

            self.target_pub.publish(target)

            self.get_logger().info(
                f'Target at ({x:.2f}, {y:.2f}, {z:.2f})'
            )

    def get_depth_at_pixel(self, x, y):
        """Extract depth value at pixel coordinates."""
        if self.latest_depth is None:
            return None

        # Depth image format: 32FC1 or 16UC1
        import numpy as np
        import struct

        width = self.latest_depth.width
        height = self.latest_depth.height

        if x < 0 or x >= width or y < 0 or y >= height:
            return None

        if self.latest_depth.encoding == '32FC1':
            # 32-bit float
            idx = (y * width + x) * 4
            depth = struct.unpack('f', self.latest_depth.data[idx:idx+4])[0]
        elif self.latest_depth.encoding == '16UC1':
            # 16-bit unsigned (millimeters typically)
            idx = (y * width + x) * 2
            depth = struct.unpack('H', self.latest_depth.data[idx:idx+2])[0]
            depth = depth / 1000.0  # Convert to meters
        else:
            return None

        return depth


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## End-to-End Latency Analysis {#latency}

### Perception Pipeline Timing

```
┌─────────────────────────────────────────────────────────────────┐
│              End-to-End Perception Latency                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Isaac Sim Camera ──► NITROS Bridge ──► Image Pipeline          │
│       │                    │                  │                  │
│       └── Render: 5ms      └── 0ms           └── 2ms            │
│                                                                  │
│  ──► Detection ──► Decoder ──► Perception Node ──► Output       │
│          │            │              │                │          │
│          └── 8ms      └── 1ms       └── 2ms         Total: 18ms │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Comparison

| Configuration | Latency | Real-time at 30fps? |
|---------------|---------|---------------------|
| CPU-only pipeline | 60ms | No (budget: 33ms) |
| GPU without NITROS | 35ms | Marginal |
| Isaac ROS with NITROS | 18ms | Yes |

---

## Summary

This chapter covered Isaac ROS integration:

1. **Isaac ROS packages** provide GPU-accelerated perception for both simulation and hardware.

2. **NITROS** eliminates CPU-GPU data copies, critical for real-time performance.

3. **Isaac ROS bridge** connects Isaac Sim sensors to ROS 2 topics.

4. **Image pipeline** provides GPU-accelerated rectification, conversion, and resize.

5. **Object detection** uses TensorRT for optimized inference on camera streams.

6. **Perception nodes** combine Isaac ROS outputs into robot decisions.

---

## Self-Assessment Questions

1. **NITROS Benefit**: Your perception pipeline without NITROS takes 45ms per frame. Your control loop needs 30 fps. What's the problem and how does NITROS help?

2. **Bridge Verification**: You enabled the ROS 2 bridge but `ros2 topic list` shows no Isaac topics. What would you check?

3. **Detection Pipeline**: The detection node outputs bounding boxes but your perception node needs 3D positions. What additional sensor data do you need?

4. **Latency Budget**: Your robot needs to detect and avoid obstacles at 1 m/s with 0.5m stopping distance. What's your maximum perception latency?

5. **Same Code, Different Data**: How do you switch your perception node from Isaac Sim camera to real camera without code changes?

---

## What's Next

In [Chapter 4: Visual SLAM](/module-4/chapter-4-visual-slam), you'll implement Isaac ROS Visual SLAM for robust robot localization, enabling navigation and manipulation tasks.

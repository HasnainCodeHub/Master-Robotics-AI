---
id: chapter-3-simulated-sensors
title: "Chapter 3: Simulated Sensors"
sidebar_label: "3. Simulated Sensors"
sidebar_position: 4
---

# Chapter 3: Simulated Sensors with Realistic Noise Models

## Chapter Goal

By the end of this chapter, you will be able to **configure simulated sensors (cameras, LiDAR, IMU, depth cameras) with noise models that match the physical sensor specifications learned in Module 1**.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 3.1 | Configure a simulated camera with resolution, frame rate, and field of view matching a target sensor |
| 3.2 | Add Gaussian noise to camera images modeling realistic sensor noise |
| 3.3 | Configure simulated LiDAR with range, angular resolution, and noise |
| 3.4 | Configure simulated IMU with bias, drift, and Gaussian noise |
| 3.5 | Validate that simulated sensor output statistics match expected behavior |
| 3.6 | Connect simulated sensors to ROS 2 topics via ros_gz bridge |

---

## Why Sensor Noise Matters

Consider this scenario: Your object detection algorithm achieves 99% accuracy on clean simulated images. On real hardware with sensor noise, accuracy drops to 75%.

**The algorithm never learned to handle noise.**

Adding realistic noise to simulated sensors:
- Trains perception algorithms to be robust
- Reveals algorithm weaknesses before deployment
- Creates more representative test conditions

**Physical Grounding**: This chapter directly applies the sensor analysis from Module 1, Chapter 2. You'll configure noise parameters that match real sensor datasheets.

---

## Camera Simulation {#camera}

### Basic Camera Configuration

```xml
<model name="camera">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="camera_link">
    <sensor name="camera_sensor" type="camera">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate>

      <camera>
        <!-- Resolution -->
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>

        <!-- Depth range -->
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>

        <!-- Distortion (optional) -->
        <distortion>
          <k1>0.0</k1>
          <k2>0.0</k2>
          <k3>0.0</k3>
          <p1>0.0</p1>
          <p2>0.0</p2>
          <center>0.5 0.5</center>
        </distortion>
      </camera>

      <!-- ros_gz bridge topic -->
      <topic>/camera/image_raw</topic>
    </sensor>
  </link>
</model>
```

### Matching Real Camera Specifications

**Example: Intel RealSense D435**

| Parameter | Real Spec | SDF Configuration |
|-----------|-----------|-------------------|
| RGB Resolution | 1920x1080 | `<width>1920</width><height>1080</height>` |
| Frame Rate | 30 fps | `<update_rate>30.0</update_rate>` |
| Horizontal FOV | 69° | `<horizontal_fov>1.204</horizontal_fov>` |
| Depth Range | 0.1-10m | `<near>0.1</near><far>10</far>` |

```xml
<!-- Intel RealSense D435 configuration -->
<camera>
  <horizontal_fov>1.204</horizontal_fov>  <!-- 69 degrees -->
  <image>
    <width>1920</width>
    <height>1080</height>
    <format>R8G8B8</format>
  </image>
  <clip>
    <near>0.1</near>
    <far>10.0</far>
  </clip>
</camera>
```

### Adding Camera Noise

```xml
<sensor name="noisy_camera" type="camera">
  <camera>
    <image>
      <width>640</width>
      <height>480</height>
    </image>

    <!-- Gaussian noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- Based on sensor SNR -->
    </noise>
  </camera>
</sensor>
```

**Physical Grounding**: Noise stddev relates to sensor Signal-to-Noise Ratio (SNR):

| SNR (dB) | Noise StdDev | Visual Effect |
|----------|--------------|---------------|
| 40 | 0.01 | Barely visible |
| 30 | 0.03 | Light grain |
| 20 | 0.1 | Obvious noise |

### Camera Intrinsics

For perception algorithms requiring calibration:

```xml
<camera>
  <lens>
    <intrinsics>
      <fx>554.25</fx>    <!-- Focal length X (pixels) -->
      <fy>554.25</fy>    <!-- Focal length Y (pixels) -->
      <cx>320.0</cx>     <!-- Principal point X -->
      <cy>240.0</cy>     <!-- Principal point Y -->
      <s>0</s>           <!-- Skew -->
    </intrinsics>
  </lens>
</camera>
```

---

## LiDAR Simulation {#lidar}

### GPU LiDAR Configuration

```xml
<model name="lidar">
  <link name="lidar_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10.0</update_rate>

      <lidar>
        <!-- Horizontal scan -->
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

        <!-- Range -->
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>

      <topic>/scan</topic>
    </sensor>
  </link>
</model>
```

### Matching Real LiDAR Specifications

**Example: Velodyne VLP-16**

| Parameter | Real Spec | SDF Configuration |
|-----------|-----------|-------------------|
| Horizontal FOV | 360° | `min_angle=-3.14159, max_angle=3.14159` |
| Horizontal Resolution | 0.1°-0.4° | `samples=900` for 0.4° |
| Vertical Channels | 16 | `vertical/samples=16` |
| Vertical FOV | ±15° | `min=-0.2618, max=0.2618` |
| Range | 100m | `<max>100.0</max>` |
| Update Rate | 5-20 Hz | `<update_rate>10.0</update_rate>` |

```xml
<!-- Velodyne VLP-16 configuration -->
<lidar>
  <scan>
    <horizontal>
      <samples>900</samples>
      <min_angle>-3.14159</min_angle>
      <max_angle>3.14159</max_angle>
    </horizontal>
    <vertical>
      <samples>16</samples>
      <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
      <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
    </vertical>
  </scan>
  <range>
    <min>0.5</min>
    <max>100.0</max>
    <resolution>0.003</resolution>  <!-- 3mm -->
  </range>
</lidar>
```

### LiDAR Noise Models

```xml
<lidar>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.02</stddev>  <!-- 2cm range noise -->
  </noise>
</lidar>
```

**Physical Grounding** from Module 1:

| LiDAR Type | Typical Range Noise | Dropout Rate |
|------------|--------------------|--------------|
| Time-of-Flight | 1-3 cm | Low |
| Triangulation | 0.5-2 cm | Moderate |
| Solid State | 2-5 cm | Varies |

### Reality Gap Warning

Simulated LiDAR uses ray-casting, which differs from real time-of-flight:

| Aspect | Simulation | Reality |
|--------|------------|---------|
| Multi-path reflections | Not modeled | Can cause ghost points |
| Translucent materials | Solid | Partial penetration |
| Retroreflective surfaces | Normal | Very bright returns |
| Motion blur | Instantaneous | Distortion during motion |

---

## IMU Simulation {#imu}

### Basic IMU Configuration

```xml
<model name="imu_model">
  <link name="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100.0</update_rate>

      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>  <!-- rad/s -->
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </z>
        </angular_velocity>

        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>  <!-- m/s² -->
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>

      <topic>/imu/data</topic>
    </sensor>
  </link>
</model>
```

### Matching Real IMU Specifications

**Example: InvenSense MPU-6050**

| Parameter | Datasheet Value | SDF Conversion |
|-----------|-----------------|----------------|
| Gyro Noise Density | 0.005 °/s/√Hz | At 100Hz: 0.005 × √100 = 0.05 °/s = 0.00087 rad/s |
| Accel Noise Density | 400 µg/√Hz | At 100Hz: 400 × √100 × 9.81e-6 = 0.039 m/s² |
| Gyro Bias Stability | 5 °/hr | Very small, often ignored in short tests |

```xml
<!-- MPU-6050 noise parameters -->
<imu>
  <angular_velocity>
    <x><noise type="gaussian"><stddev>0.00087</stddev></noise></x>
    <y><noise type="gaussian"><stddev>0.00087</stddev></noise></y>
    <z><noise type="gaussian"><stddev>0.00087</stddev></noise></z>
  </angular_velocity>
  <linear_acceleration>
    <x><noise type="gaussian"><stddev>0.039</stddev></noise></x>
    <y><noise type="gaussian"><stddev>0.039</stddev></noise></y>
    <z><noise type="gaussian"><stddev>0.039</stddev></noise></z>
  </linear_acceleration>
</imu>
```

### IMU Bias and Drift

Real IMUs have bias that drifts over time. Simulation can approximate this:

```xml
<angular_velocity>
  <x>
    <noise type="gaussian">
      <mean>0.0001</mean>   <!-- Constant bias -->
      <stddev>0.001</stddev> <!-- Random noise -->
      <bias_mean>0.0001</bias_mean>
      <bias_stddev>0.00001</bias_stddev>  <!-- Bias drift -->
    </noise>
  </x>
</angular_velocity>
```

**Physical Grounding**: IMU bias causes position drift when integrated:
- 0.1 °/s gyro bias → ~6°/minute heading error
- 0.01 m/s² accel bias → ~18m/minute position error

---

## Depth Camera Simulation {#depth-camera}

### RGBD Camera Configuration

```xml
<sensor name="rgbd_camera" type="rgbd_camera">
  <always_on>true</always_on>
  <update_rate>30.0</update_rate>

  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>

    <!-- Depth noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- 5mm depth noise -->
    </noise>
  </camera>

  <topic>/camera/color/image_raw</topic>
  <depth_camera>
    <topic>/camera/depth/image_raw</topic>
  </depth_camera>
</sensor>
```

### Depth Noise Characteristics

Real depth cameras have distance-dependent noise:

| Distance | Typical Noise |
|----------|---------------|
| 0.5m | 1-2 mm |
| 2m | 5-10 mm |
| 5m | 20-50 mm |

Gazebo's Gaussian noise is distance-independent. For more realistic simulation:

```xml
<!-- Approximate distance-dependent noise with higher base noise -->
<noise>
  <type>gaussian</type>
  <stddev>0.01</stddev>  <!-- 1cm, acceptable for mid-range -->
</noise>
```

---

## ROS 2 Bridge Configuration {#ros-bridge}

### Bridge Launch File

```python
# launch/sensor_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Camera
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

                # LiDAR
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

                # IMU
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',

                # Depth camera
                '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

                # Clock
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        ),
    ])
```

### Verifying Sensor Output

```bash
# Check camera topic
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --once

# Check LiDAR
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link lidar_link

# Check IMU
ros2 topic hz /imu/data
ros2 topic echo /imu/data
```

---

## Validating Sensor Statistics {#validation}

### IMU Noise Validation

```python
#!/usr/bin/env python3
"""Validate IMU noise matches expected parameters."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class ImuValidator(Node):
    def __init__(self):
        super().__init__('imu_validator')

        self.accel_samples = []
        self.gyro_samples = []

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Report every 5 seconds
        self.timer = self.create_timer(5.0, self.report_stats)

        self.get_logger().info('Collecting IMU samples...')

    def imu_callback(self, msg):
        self.accel_samples.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.gyro_samples.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

    def report_stats(self):
        if len(self.accel_samples) < 100:
            return

        accel = np.array(self.accel_samples)
        gyro = np.array(self.gyro_samples)

        # Accelerometer statistics
        # Z should be ~9.81, X and Y should be ~0
        accel_mean = np.mean(accel, axis=0)
        accel_std = np.std(accel, axis=0)

        self.get_logger().info(
            f'Accel mean: [{accel_mean[0]:.4f}, {accel_mean[1]:.4f}, {accel_mean[2]:.4f}] m/s²'
        )
        self.get_logger().info(
            f'Accel std:  [{accel_std[0]:.4f}, {accel_std[1]:.4f}, {accel_std[2]:.4f}] m/s²'
        )

        # Gyroscope statistics (should all be ~0)
        gyro_mean = np.mean(gyro, axis=0)
        gyro_std = np.std(gyro, axis=0)

        self.get_logger().info(
            f'Gyro mean:  [{gyro_mean[0]:.6f}, {gyro_mean[1]:.6f}, {gyro_mean[2]:.6f}] rad/s'
        )
        self.get_logger().info(
            f'Gyro std:   [{gyro_std[0]:.6f}, {gyro_std[1]:.6f}, {gyro_std[2]:.6f}] rad/s'
        )

        # Compare to expected (from SDF configuration)
        expected_accel_std = 0.039  # From MPU-6050 config
        expected_gyro_std = 0.00087

        self.get_logger().info(
            f'Expected accel std: {expected_accel_std}, measured: {np.mean(accel_std):.4f}'
        )
        self.get_logger().info(
            f'Expected gyro std: {expected_gyro_std}, measured: {np.mean(gyro_std):.6f}'
        )


def main():
    rclpy.init()
    node = ImuValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Summary

This chapter covered configuring simulated sensors with realistic noise:

1. **Camera simulation** requires matching resolution, FOV, and frame rate. Add Gaussian noise based on sensor SNR.

2. **LiDAR simulation** uses ray-casting; configure range, resolution, and Gaussian range noise. Be aware of ray-casting limitations.

3. **IMU simulation** requires configuring both accelerometer and gyroscope noise from datasheet noise density values.

4. **Depth cameras** have distance-dependent noise that simple Gaussian models approximate.

5. **Validation** compares simulated sensor statistics to expected values from configuration.

---

## Reality Gap Callout

Even with noise models, simulated sensors differ from real sensors:

| Sensor | What Simulation Misses |
|--------|----------------------|
| Camera | Motion blur, lens flare, hot pixels, banding |
| LiDAR | Multi-path, translucent materials, retroreflection |
| IMU | Non-Gaussian outliers, temperature dependence |
| Depth | Edge artifacts, reflective surface failures |

**Your algorithms must be robust to these unmodeled effects.**

---

## Self-Assessment Questions

1. **Camera Configuration**: You have an Intel RealSense D435 with 87° horizontal FOV. Convert this to radians for the `<horizontal_fov>` element.

2. **LiDAR Noise**: Your LiDAR datasheet shows 3cm range accuracy (1σ). What `<stddev>` value would you use?

3. **IMU Validation**: After collecting 1000 IMU samples, your measured gyro stddev is 0.002 rad/s but you configured 0.001 rad/s. What could cause this discrepancy?

4. **Depth Camera**: Your depth camera shows increasing noise at far distances, but simulation has constant noise. How would you handle this in algorithm development?

5. **Bridge Configuration**: Your camera publishes to `/camera/image_raw` in Gazebo but your ROS 2 node subscribes to `/front_camera/image`. What would you change?

---

## What's Next

In [Chapter 4: Simulated Actuators](/module-3/chapter-4-simulated-actuators), you'll configure simulated motors and joint controllers with realistic delay, saturation, and friction—applying Module 1's actuator analysis to simulation.

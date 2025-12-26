---
id: chapter-2-topics-pubsub
title: "Chapter 2: Topics and Publishers/Subscribers"
sidebar_label: "2. Topics & Pub/Sub"
sidebar_position: 3
---

# Chapter 2: Topics and Publishers/Subscribers

## Chapter Goal

By the end of this chapter, you will be able to **implement the publish/subscribe pattern for continuous data streams**, with emphasis on sensor data handling and appropriate QoS configuration based on physical sensor characteristics.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 2.1 | Implement a publisher node that publishes sensor data at a specified rate with appropriate QoS |
| 2.2 | Implement a subscriber node that processes sensor data with callbacks and handles message timing |
| 2.3 | Select and use standard message types for common sensor data |
| 2.4 | Configure QoS profiles based on sensor characteristics |
| 2.5 | Debug topic communication issues using command-line tools |

---

## The Perception-Action Loop as a Topic Network

In Module 1, you learned that robots operate through continuous perception-action loops. In ROS 2, these loops are implemented as **topic networks**—publishers streaming data to subscribers.

```
Sensor (physical)          ROS 2 Node              ROS 2 Topic
      │                        │                        │
      │  Light/Sound/Force     │                        │
      ▼                        ▼                        ▼
  ┌───────┐             ┌───────────┐           /camera/image
  │Camera │────────────►│camera_node│──────────────────────►
  └───────┘   analog    └───────────┘    Image msg
                                              │
                                              ▼
                                        ┌───────────┐
                                        │detector   │
                                        │_node      │
                                        └───────────┘
```

Each sensor becomes a publisher. Each processing stage subscribes to inputs and publishes outputs.

---

## Publishers: Streaming Sensor Data {#publishers}

A publisher sends messages to a topic. Let's implement an IMU publisher.

### Basic Publisher Structure

```python
#!/usr/bin/env python3
"""
IMU Publisher Node

Publishes simulated IMU data at 100 Hz with Gaussian noise.
Physical grounding: Noise characteristics from M1 Chapter 2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np


class ImuPublisher(Node):
    """Publishes IMU data with realistic noise characteristics."""

    def __init__(self):
        super().__init__('imu_publisher')

        # Physical parameters from M1 sensor specifications
        self.publish_rate_hz = 100.0  # 100 Hz update rate
        self.accel_noise_stddev = 0.01  # m/s² (from datasheet)
        self.gyro_noise_stddev = 0.001  # rad/s (from datasheet)

        # QoS: Best-effort for high-rate sensor data (M1 Chapter 2)
        # - IMU data is high rate, latest value matters most
        # - Missing one sample at 100 Hz is acceptable
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publisher
        self.publisher = self.create_publisher(
            Imu,
            'imu/data',
            qos_profile
        )

        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.publish_imu)

        self.get_logger().info(
            f'IMU publisher started at {self.publish_rate_hz} Hz'
        )

    def publish_imu(self):
        """Publish IMU message with simulated noise."""
        msg = Imu()

        # Timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulated acceleration with Gaussian noise
        # True value is gravity (0, 0, 9.81), add noise
        msg.linear_acceleration.x = np.random.normal(0, self.accel_noise_stddev)
        msg.linear_acceleration.y = np.random.normal(0, self.accel_noise_stddev)
        msg.linear_acceleration.z = np.random.normal(9.81, self.accel_noise_stddev)

        # Simulated angular velocity with noise
        msg.angular_velocity.x = np.random.normal(0, self.gyro_noise_stddev)
        msg.angular_velocity.y = np.random.normal(0, self.gyro_noise_stddev)
        msg.angular_velocity.z = np.random.normal(0, self.gyro_noise_stddev)

        # Covariance matrices (diagonal, from noise stddev)
        accel_var = self.accel_noise_stddev ** 2
        gyro_var = self.gyro_noise_stddev ** 2

        msg.linear_acceleration_covariance = [
            accel_var, 0.0, 0.0,
            0.0, accel_var, 0.0,
            0.0, 0.0, accel_var
        ]

        msg.angular_velocity_covariance = [
            gyro_var, 0.0, 0.0,
            0.0, gyro_var, 0.0,
            0.0, 0.0, gyro_var
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Key Publisher Concepts

**Timer-based publishing**: The timer calls `publish_imu()` at the specified rate. This matches the physical sensor's update rate.

**Message timestamps**: Every message should have a timestamp in the header. This enables:
- Latency measurement (compare stamp to receive time)
- Synchronization with other sensors
- Detecting stale data

**Covariance matrices**: For sensor messages, covariance captures uncertainty. This connects directly to M1's noise models—subscribers can use this for sensor fusion.

---

## Subscribers: Processing Sensor Data {#subscribers}

A subscriber receives messages from a topic and processes them in callbacks.

### Basic Subscriber Structure

```python
#!/usr/bin/env python3
"""
IMU Subscriber Node

Subscribes to IMU data and monitors timing.
Physical grounding: Detects when sensor violates timing requirements from M1.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import time


class ImuSubscriber(Node):
    """Subscribes to IMU data with timing monitoring."""

    def __init__(self):
        super().__init__('imu_subscriber')

        # Expected timing (from M1 sensor specifications)
        self.expected_rate_hz = 100.0
        self.max_acceptable_delay_ms = 50.0  # 5x the 10ms period

        # QoS must match publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscription
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile
        )

        self.last_receive_time = None
        self.message_count = 0
        self.late_count = 0

        self.get_logger().info('IMU subscriber started')

    def imu_callback(self, msg: Imu):
        """Process received IMU message."""
        receive_time = time.time()
        self.message_count += 1

        # Check message latency (time since message was created)
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_ros_time = self.get_clock().now().nanoseconds * 1e-9
        latency_ms = (current_ros_time - msg_time) * 1000

        if latency_ms > self.max_acceptable_delay_ms:
            self.late_count += 1
            self.get_logger().warn(
                f'IMU message late: {latency_ms:.1f}ms '
                f'(threshold: {self.max_acceptable_delay_ms}ms)'
            )

        # Check inter-message timing
        if self.last_receive_time is not None:
            delta_ms = (receive_time - self.last_receive_time) * 1000
            expected_period_ms = 1000.0 / self.expected_rate_hz

            if delta_ms > expected_period_ms * 2:  # More than 2x expected
                self.get_logger().warn(
                    f'IMU gap detected: {delta_ms:.1f}ms '
                    f'(expected: {expected_period_ms:.1f}ms)'
                )

        self.last_receive_time = receive_time

        # Process the data (example: compute acceleration magnitude)
        accel_magnitude = (
            msg.linear_acceleration.x ** 2 +
            msg.linear_acceleration.y ** 2 +
            msg.linear_acceleration.z ** 2
        ) ** 0.5

        # Log periodically
        if self.message_count % 100 == 0:
            self.get_logger().info(
                f'Received {self.message_count} messages, '
                f'{self.late_count} late, '
                f'accel: {accel_magnitude:.2f} m/s²'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Callback Execution Model

**Callbacks are called by the executor**. Understanding this is critical:

- `rclpy.spin(node)` runs the executor
- Executor checks for new messages and calls callbacks
- Callbacks should be fast (don't block)
- Long processing should be in separate threads

**Physical Grounding**: If your callback takes 20ms and messages arrive at 100 Hz (10ms), you will fall behind. The M1 timing budget must account for callback execution time.

---

## Standard Message Types {#message-types}

ROS 2 provides standard messages for common sensor data. Use these instead of creating custom types.

### Common Sensor Messages

| Sensor (from M1) | Message Type | Key Fields |
|------------------|--------------|------------|
| IMU | `sensor_msgs/Imu` | orientation, angular_velocity, linear_acceleration |
| Camera | `sensor_msgs/Image` | height, width, encoding, data |
| Depth Camera | `sensor_msgs/PointCloud2` | points, fields, is_dense |
| LiDAR | `sensor_msgs/LaserScan` | ranges, angle_min/max, range_min/max |
| Encoder | `sensor_msgs/JointState` | name, position, velocity, effort |
| Force/Torque | `geometry_msgs/WrenchStamped` | force, torque |

### Command Messages

| Actuator (from M1) | Message Type | Key Fields |
|--------------------|--------------|------------|
| Mobile base | `geometry_msgs/Twist` | linear.x/y/z, angular.x/y/z |
| Joint commands | `trajectory_msgs/JointTrajectory` | joint_names, points |
| Pose goal | `geometry_msgs/PoseStamped` | position, orientation |

### Message Headers

Most sensor messages include a `std_msgs/Header`:

```python
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "camera_link"
```

- **stamp**: When the data was captured (not when published)
- **frame_id**: The coordinate frame for the data

**Physical Grounding**: The frame_id connects to URDF (Chapter 5). The timestamp enables latency tracking from M1 Chapter 4.

---

## QoS Configuration in Practice {#qos-practice}

### QoS Profiles

ROS 2 provides preset profiles:

```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# Sensor data profile: best-effort, keep last 5
sensor_qos = qos_profile_sensor_data

# System default: reliable, keep last 10
default_qos = qos_profile_system_default
```

### Custom QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

# Custom QoS for safety-critical data
safety_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    deadline=Duration(seconds=0, nanoseconds=100_000_000),  # 100ms
    lifespan=Duration(seconds=1)  # Messages valid for 1 second
)
```

### QoS Decision Framework

**From M1 sensor/actuator analysis, ask:**

1. **Can we tolerate missed messages?**
   - Yes → `BEST_EFFORT`
   - No → `RELIABLE`

2. **What data do late subscribers need?**
   - Just new data → `VOLATILE`
   - Recent history → `TRANSIENT_LOCAL`

3. **Is there a timing deadline?**
   - Yes → Set `deadline` to the maximum acceptable delay

4. **Can stale data cause problems?**
   - Yes → Set `lifespan` to data validity period

---

## Debugging Topic Communication {#debugging}

### Check Topic Existence

```bash
# List all topics
ros2 topic list

# Check if specific topic exists
ros2 topic list | grep imu
```

### Check Message Type

```bash
# Show topic info
ros2 topic info /imu/data

# Show message type definition
ros2 interface show sensor_msgs/msg/Imu
```

### Monitor Data

```bash
# Echo messages
ros2 topic echo /imu/data

# Check rate
ros2 topic hz /imu/data

# Check bandwidth
ros2 topic bw /imu/data
```

### Diagnose QoS Issues

```bash
# Verbose topic info shows QoS
ros2 topic info /imu/data --verbose
```

Common issues:
- **No messages received**: QoS mismatch (e.g., `RELIABLE` subscriber, `BEST_EFFORT` publisher)
- **Intermittent messages**: Network issues or publisher crashes
- **Old data**: Check timestamps, might be replayed data

---

## Command Velocity Publisher Example {#cmd-vel-example}

Publishing actuator commands requires different considerations than sensor data.

```python
#!/usr/bin/env python3
"""
Velocity Command Publisher

Publishes velocity commands with safety constraints.
Physical grounding: Rate limits from M1 Chapter 3 actuator specs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    """Publishes velocity commands with physical safety limits."""

    def __init__(self):
        super().__init__('velocity_publisher')

        # Physical limits from M1 actuator specifications
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.command_rate_hz = 10.0  # Commands at 10 Hz

        # QoS for commands:
        # - Best-effort: if a command is missed, next one comes soon
        # - Short lifespan: stale commands are dangerous
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=0, nanoseconds=200_000_000)  # 200ms
        )

        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_profile
        )

        # Timer for periodic commands
        self.timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self.publish_command
        )

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.get_logger().info('Velocity publisher started')

    def set_velocity(self, linear: float, angular: float):
        """Set target velocity with safety clamping."""
        self.target_linear = max(-self.max_linear_vel,
                                  min(self.max_linear_vel, linear))
        self.target_angular = max(-self.max_angular_vel,
                                   min(self.max_angular_vel, angular))

    def publish_command(self):
        """Publish velocity command."""
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular

        self.publisher.publish(msg)
```

**Physical Grounding**:
- Maximum velocities come from M1 actuator specifications
- Short lifespan ensures stale commands don't move the robot
- 10 Hz rate matches typical control loop requirements

---

## Summary

This chapter covered implementing publishers and subscribers for sensor data:

1. **Publishers** stream data at rates matching physical sensor characteristics, with timestamps and covariance from M1 noise models.

2. **Subscribers** process data in callbacks, monitoring timing to detect when physical requirements are violated.

3. **Standard message types** exist for common sensors and actuators—use them.

4. **QoS configuration** directly implements M1 physical constraints: deadline for timing, reliability for data criticality.

5. **Debugging tools** (`ros2 topic hz`, `echo`, `info --verbose`) help diagnose communication issues.

---

## Self-Assessment Questions

1. **QoS Design**: You have a LiDAR producing scans at 10 Hz. A mapping algorithm needs all scans. What QoS would you configure for publisher and subscriber?

2. **Callback Timing**: Your callback takes 50ms to process each message. Messages arrive at 20 Hz (50ms apart). What will happen? How would you fix it?

3. **Message Selection**: You need to publish force sensor data (3-axis force, 3-axis torque). What standard message type would you use?

4. **Timestamp Usage**: Your subscriber receives a message with a timestamp 100ms in the past. What might cause this, and how would you handle it?

5. **Debug Scenario**: A subscriber receives no messages, but `ros2 topic echo` shows the topic has data. What is the most likely cause?

---

## What's Next

In [Chapter 3: Services](/module-2/chapter-3-services), you'll implement the request/response pattern for discrete operations like calibration and gripper control.

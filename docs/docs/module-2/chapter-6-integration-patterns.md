---
id: chapter-6-integration-patterns
title: "Chapter 6: Integration Patterns"
sidebar_label: "6. Integration Patterns"
sidebar_position: 7
---

# Chapter 6: Integration Patterns and Python Agents

## Chapter Goal

By the end of this chapter, you will be able to **synthesize all ROS 2 communication patterns into coherent Python agents** that implement complete perception-action loops using topics, services, and actions together.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 6.1 | Design a node architecture that uses topics, services, and actions appropriately |
| 6.2 | Implement a behavior node that coordinates multiple communication patterns |
| 6.3 | Handle concurrent operations with proper callback management |
| 6.4 | Implement lifecycle patterns for robust node initialization and shutdown |
| 6.5 | Create launch files that configure and start multi-node systems |

---

## From Patterns to Systems

You've learned three communication patterns:
- **Topics**: Continuous data streams (sensors, commands)
- **Services**: Discrete operations (configuration, queries)
- **Actions**: Long-running tasks (navigation, manipulation)

Real robots use **all three together**. This chapter shows how to combine them.

---

## Multi-Pattern Node Architecture {#architecture}

### Example: Pick-and-Place Behavior

A pick-and-place operation uses all patterns:

```
┌──────────────────────────────────────────────────────────────────┐
│                     PickAndPlaceNode                              │
├──────────────────────────────────────────────────────────────────┤
│  SUBSCRIPTIONS (Topics)                                          │
│    /camera/image        → Object detection input                 │
│    /joint_states        → Current arm position                   │
│    /gripper/state       → Gripper status                         │
├──────────────────────────────────────────────────────────────────┤
│  PUBLISHERS (Topics)                                             │
│    /cmd_vel             → Base motion commands                   │
│    /visualization       → Debug visualization                    │
├──────────────────────────────────────────────────────────────────┤
│  SERVICE CLIENTS                                                 │
│    /gripper/command     → Open/close gripper                     │
│    /arm/set_speed       → Configure arm speed                    │
├──────────────────────────────────────────────────────────────────┤
│  ACTION CLIENTS                                                  │
│    /navigate_to_pose    → Move base to location                  │
│    /move_arm            → Execute arm trajectory                 │
└──────────────────────────────────────────────────────────────────┘
```

### Pattern Selection Summary

| Need | Pattern | Example |
|------|---------|---------|
| Monitor sensor continuously | Topic subscription | Camera images |
| Send continuous commands | Topic publishing | Velocity commands |
| Configure before operation | Service call | Set arm speed |
| Execute discrete action | Service call | Open gripper |
| Execute long operation | Action call | Navigate to target |

---

## Integrated Behavior Node {#behavior-node}

```python
#!/usr/bin/env python3
"""
Pick and Place Behavior Node

Integrates topics, services, and actions for pick-and-place.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, JointState
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from example_interfaces.action import Fibonacci  # Placeholder

import asyncio


class PickAndPlaceNode(Node):
    """Coordinates pick-and-place using multiple ROS 2 patterns."""

    def __init__(self):
        super().__init__('pick_and_place')

        # Callback groups for concurrent execution
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()

        # State
        self.current_joint_state = None
        self.detected_object_pose = None
        self.gripper_closed = False

        # === SUBSCRIPTIONS ===
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.sensor_cb_group
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10,
            callback_group=self.sensor_cb_group
        )

        # === SERVICE CLIENTS ===
        self.gripper_client = self.create_client(
            SetBool,
            'gripper/command'
        )

        # === ACTION CLIENTS ===
        self.navigate_client = ActionClient(
            self,
            Fibonacci,  # Placeholder for NavigateToPose
            'navigate_to_pose',
            callback_group=self.action_cb_group
        )

        self.move_arm_client = ActionClient(
            self,
            Fibonacci,  # Placeholder for MoveArm
            'move_arm',
            callback_group=self.action_cb_group
        )

        self.get_logger().info('Pick and place node initialized')

    # === TOPIC CALLBACKS ===

    def joint_state_callback(self, msg: JointState):
        """Process joint state updates."""
        self.current_joint_state = msg

    def image_callback(self, msg: Image):
        """Process camera images for object detection."""
        # In real system: run detection, update self.detected_object_pose
        pass

    # === SERVICE CALLS ===

    async def control_gripper(self, close: bool) -> bool:
        """Open or close gripper."""
        if not self.gripper_client.service_is_ready():
            self.get_logger().error('Gripper service not available')
            return False

        request = SetBool.Request()
        request.data = close

        future = self.gripper_client.call_async(request)
        result = await future

        if result.success:
            self.gripper_closed = close
            self.get_logger().info(f'Gripper {"closed" if close else "opened"}')
        else:
            self.get_logger().error(f'Gripper command failed: {result.message}')

        return result.success

    # === ACTION CALLS ===

    async def navigate_to(self, x: float, y: float) -> bool:
        """Navigate to position."""
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        goal = Fibonacci.Goal()
        goal.order = int(x * 100)  # Placeholder encoding

        self.get_logger().info(f'Navigating to ({x}, {y})')

        goal_handle = await self.navigate_client.send_goal_async(goal)

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        result = await goal_handle.get_result_async()
        return result.status == 4  # SUCCEEDED

    async def move_arm_to(self, pose: PoseStamped) -> bool:
        """Move arm to pose."""
        if not self.move_arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm server not available')
            return False

        goal = Fibonacci.Goal()
        goal.order = 10  # Placeholder

        goal_handle = await self.move_arm_client.send_goal_async(goal)

        if not goal_handle.accepted:
            return False

        result = await goal_handle.get_result_async()
        return result.status == 4

    # === HIGH-LEVEL BEHAVIOR ===

    async def execute_pick_and_place(
        self,
        pick_location: tuple,
        place_location: tuple
    ) -> bool:
        """
        Execute complete pick and place sequence.

        This demonstrates how to coordinate multiple patterns:
        1. Navigate to pick location (action)
        2. Detect object (topic data)
        3. Move arm to object (action)
        4. Close gripper (service)
        5. Move arm up (action)
        6. Navigate to place location (action)
        7. Move arm down (action)
        8. Open gripper (service)
        9. Move arm to home (action)
        """
        self.get_logger().info('Starting pick and place sequence')

        # 1. Navigate to pick location
        self.get_logger().info('Step 1: Navigate to pick location')
        if not await self.navigate_to(*pick_location):
            self.get_logger().error('Failed to navigate to pick location')
            return False

        # 2. Wait for object detection (from topic callback)
        self.get_logger().info('Step 2: Waiting for object detection')
        # In real system: wait until self.detected_object_pose is valid

        # 3. Move arm to object
        self.get_logger().info('Step 3: Moving arm to object')
        object_pose = PoseStamped()  # Would come from detection
        if not await self.move_arm_to(object_pose):
            self.get_logger().error('Failed to move arm to object')
            return False

        # 4. Close gripper
        self.get_logger().info('Step 4: Closing gripper')
        if not await self.control_gripper(close=True):
            self.get_logger().error('Failed to close gripper')
            return False

        # 5. Move arm up (retract)
        self.get_logger().info('Step 5: Retracting arm')
        # ... similar action call

        # 6. Navigate to place location
        self.get_logger().info('Step 6: Navigate to place location')
        if not await self.navigate_to(*place_location):
            # Error handling: we have object, need safe behavior
            self.get_logger().error('Navigation failed while holding object')
            return False

        # 7. Move arm down
        self.get_logger().info('Step 7: Moving arm to place position')
        # ... similar action call

        # 8. Open gripper
        self.get_logger().info('Step 8: Opening gripper')
        if not await self.control_gripper(close=False):
            self.get_logger().error('Failed to open gripper')
            return False

        # 9. Move arm to home
        self.get_logger().info('Step 9: Moving arm to home')
        # ... similar action call

        self.get_logger().info('Pick and place complete!')
        return True


async def main_async():
    rclpy.init()
    node = PickAndPlaceNode()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run executor in background
    executor_task = asyncio.get_event_loop().run_in_executor(
        None, executor.spin
    )

    # Execute behavior
    try:
        success = await node.execute_pick_and_place(
            pick_location=(1.0, 0.5),
            place_location=(2.0, 0.5)
        )
        print(f'Pick and place result: {success}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
```

---

## Callback Groups and Executors {#executors}

### The Problem

By default, callbacks execute sequentially. If a service call blocks for 2 seconds, no topic callbacks run during that time.

### Callback Groups

```python
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup
)

# Mutually exclusive: only one callback at a time
sensor_group = MutuallyExclusiveCallbackGroup()

# Reentrant: callbacks can run concurrently
action_group = ReentrantCallbackGroup()

# Assign to subscriptions/clients
self.create_subscription(..., callback_group=sensor_group)
self.create_service(..., callback_group=action_group)
```

### Executors

```python
from rclpy.executors import (
    SingleThreadedExecutor,  # Default, sequential
    MultiThreadedExecutor    # Concurrent execution
)

# For concurrent operation, use multi-threaded
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

**Physical Grounding**: If sensor callbacks must run at 100 Hz (10 ms) but action execution takes 5 seconds, you **must** use concurrent execution. Otherwise, you violate M1 timing requirements.

---

## Launch Files {#launch-files}

Launch files start and configure multiple nodes.

### Basic Launch File

```python
# launch/pick_and_place.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': '...'}]
        ),

        # Pick and place behavior
        Node(
            package='my_robot',
            executable='pick_and_place_node',
            name='pick_and_place',
            parameters=[
                {'max_velocity': 0.5},
                {'gripper_timeout': 5.0}
            ],
            remappings=[
                ('camera/image', '/front_camera/image_raw'),
                ('joint_states', '/arm/joint_states')
            ]
        ),

        # Gripper controller
        Node(
            package='my_robot',
            executable='gripper_service',
            name='gripper_controller'
        ),

        # Navigation action server
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='navigator'
        ),
    ])
```

### Parameters

```python
# In node
self.declare_parameter('max_velocity', 0.5)
max_vel = self.get_parameter('max_velocity').value

# In launch file
parameters=[{'max_velocity': 1.0}]

# From command line
ros2 run my_robot my_node --ros-args -p max_velocity:=1.0
```

### Remappings

Connect nodes with different topic names:

```python
remappings=[
    ('input_topic', '/actual_topic_name'),
    ('output_topic', '/different_name')
]
```

---

## Lifecycle Patterns {#lifecycle}

Robust nodes handle startup and shutdown gracefully.

### Initialization Pattern

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        self.initialized = False

        # Create interfaces
        self.setup_interfaces()

        # Wait for dependencies
        self.create_timer(1.0, self.check_initialization)

    def setup_interfaces(self):
        """Create all ROS interfaces."""
        self.sensor_sub = self.create_subscription(...)
        self.gripper_client = self.create_client(...)

    def check_initialization(self):
        """Check if all dependencies are ready."""
        if self.initialized:
            return

        # Check service availability
        if not self.gripper_client.service_is_ready():
            self.get_logger().info('Waiting for gripper service...')
            return

        # Check for sensor data
        if self.last_sensor_msg is None:
            self.get_logger().info('Waiting for sensor data...')
            return

        # All ready
        self.initialized = True
        self.get_logger().info('Node fully initialized')
```

### Shutdown Pattern

```python
def shutdown(self):
    """Graceful shutdown."""
    self.get_logger().info('Shutting down...')

    # Cancel any active goals
    if self.active_goal_handle:
        self.active_goal_handle.cancel_goal_async()

    # Safe state (e.g., stop motion)
    self.publish_zero_velocity()

    # Open gripper if holding object
    if self.gripper_closed:
        self.control_gripper(close=False)

    self.get_logger().info('Shutdown complete')
```

---

## Timing Analysis {#timing-analysis}

Verify your system meets M1 physical constraints.

### Measuring Callback Latency

```python
import time

def image_callback(self, msg):
    start = time.time()

    # Processing...
    self.process_image(msg)

    elapsed = time.time() - start
    if elapsed > 0.030:  # 30ms budget
        self.get_logger().warn(f'Image callback took {elapsed*1000:.1f}ms')
```

### End-to-End Latency

```python
def image_callback(self, msg):
    # Message timestamp
    msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # Current time
    now = self.get_clock().now().nanoseconds * 1e-9

    # Transport latency
    transport_latency = now - msg_time

    if transport_latency > 0.050:  # 50ms threshold
        self.get_logger().warn(
            f'High transport latency: {transport_latency*1000:.1f}ms'
        )
```

---

## Summary

This chapter synthesized all ROS 2 patterns:

1. **Multi-pattern nodes** combine topics, services, and actions for complex behaviors.

2. **Callback groups and executors** enable concurrent operation—essential when actions block but sensors must still be processed.

3. **Launch files** configure and start multi-node systems with parameters and remappings.

4. **Lifecycle patterns** ensure robust initialization and graceful shutdown.

5. **Timing analysis** verifies that your implementation meets M1 physical constraints.

---

## Module 2 Complete

Congratulations on completing Module 2: ROS 2 Fundamentals!

You can now:
- Implement robotic communication using topics, services, and actions
- Configure QoS based on physical sensor/actuator requirements
- Describe robots using URDF
- Build integrated Python agents that implement perception-action loops

## What's Next

In [Module 3: Simulation and Digital Twin](/module-3), you'll connect these ROS 2 nodes to physics-simulated robots in Gazebo. The code you wrote in this module will work unchanged—only the source of sensor data changes from simulated sensors.

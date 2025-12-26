---
id: chapter-4-actions
title: "Chapter 4: Actions"
sidebar_label: "4. Actions"
sidebar_position: 5
---

# Chapter 4: Actions for Long-Running Tasks

## Chapter Goal

By the end of this chapter, you will be able to **implement the action pattern for long-running tasks** with feedback and cancellation support—essential for navigation, manipulation, and any task that takes significant time.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 4.1 | Implement an action server that executes long-running tasks with progress feedback |
| 4.2 | Implement an action client that monitors progress and handles completion/failure/cancellation |
| 4.3 | Design action interfaces with appropriate goal, result, and feedback messages |
| 4.4 | Implement cancellation handling that safely stops physical operations |
| 4.5 | Choose between services and actions based on operation characteristics |

---

## Why Actions Exist

Services work for quick operations (< 5 seconds), but robots often need to:
- Navigate to a goal (30+ seconds)
- Execute a manipulation sequence (10+ seconds)
- Perform a scan or survey (minutes)

For these operations, you need:
- **Progress feedback**: "50% complete, 10 meters remaining"
- **Cancellation**: "Stop what you're doing"
- **Non-blocking execution**: Client continues while server executes

**Actions provide all three.**

---

## Action Anatomy {#action-anatomy}

An action has three message types:

```
┌──────────────────────────────────────────────────────────┐
│                        ACTION                             │
├──────────────────────────────────────────────────────────┤
│  GOAL (client → server)                                  │
│    What the client wants done                            │
│    Example: target_position, max_speed                   │
├──────────────────────────────────────────────────────────┤
│  FEEDBACK (server → client, periodic)                    │
│    Progress updates during execution                     │
│    Example: current_position, distance_remaining         │
├──────────────────────────────────────────────────────────┤
│  RESULT (server → client, once at end)                   │
│    Final outcome when action completes                   │
│    Example: final_position, total_time, success          │
└──────────────────────────────────────────────────────────┘
```

### Action State Machine

```
     ┌─────────┐
     │ PENDING │ Goal received, not yet accepted
     └────┬────┘
          │ accept
          ▼
     ┌─────────┐
     │ EXECUTING│ Goal accepted, in progress
     └────┬────┘
          │
    ┌─────┴─────┬─────────────┐
    │           │             │
    ▼           ▼             ▼
┌────────┐ ┌────────┐   ┌──────────┐
│SUCCEEDED│ │ ABORTED│   │ CANCELED │
└────────┘ └────────┘   └──────────┘
  Normal     Error        Client
  completion occurred     requested
```

---

## Action Server Implementation {#action-server}

Let's implement a navigation action that moves to a target position.

```python
#!/usr/bin/env python3
"""
Navigation Action Server

Executes navigation goals with progress feedback.
Physical grounding: Motion timing from M1 actuator characteristics.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import Fibonacci  # We'll use this as template
from geometry_msgs.msg import Point

import time
import math


class NavigationServer(Node):
    """Action server for navigation goals."""

    def __init__(self):
        super().__init__('navigation_server')

        # Physical parameters from M1
        self.max_velocity = 0.5  # m/s
        self.position_tolerance = 0.1  # m
        self.feedback_rate = 10.0  # Hz

        # Current position (simulated)
        self.current_x = 0.0
        self.current_y = 0.0

        # Use reentrant callback group for concurrent handling
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Using Fibonacci as placeholder
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('Navigation action server ready')

    def goal_callback(self, goal_request):
        """Decide whether to accept or reject a goal."""
        self.get_logger().info('Received navigation goal')

        # Could reject if robot is in error state, etc.
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Decide whether to accept cancellation request."""
        self.get_logger().info('Received cancel request')

        # Always accept cancellation (safe stop is always possible)
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the navigation goal."""
        self.get_logger().info('Executing navigation goal')

        # Parse goal (using order as distance for this example)
        target_distance = goal_handle.request.order
        target_x = self.current_x + target_distance
        target_y = self.current_y

        # Calculate expected duration
        distance = abs(target_distance)
        expected_time = distance / self.max_velocity

        self.get_logger().info(
            f'Navigating {distance:.1f}m, ETA: {expected_time:.1f}s'
        )

        # Execute with feedback
        feedback_msg = Fibonacci.Feedback()
        start_time = time.time()

        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation canceled')

                # Safe stop (would ramp down velocity in real system)
                result = Fibonacci.Result()
                result.sequence = [int(self.current_x * 100)]
                return result

            # Calculate progress
            elapsed = time.time() - start_time
            progress = min(elapsed / expected_time, 1.0)

            # Update simulated position
            self.current_x = progress * target_distance

            # Check if goal reached
            remaining = abs(target_x - self.current_x)
            if remaining < self.position_tolerance:
                break

            # Publish feedback
            feedback_msg.partial_sequence = [
                int(progress * 100),  # Progress percentage
                int(remaining * 100)  # Remaining distance (cm)
            ]
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'Progress: {progress*100:.0f}%, remaining: {remaining:.2f}m'
            )

            # Wait before next feedback
            time.sleep(1.0 / self.feedback_rate)

        # Success
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [int(self.current_x * 100), int(self.current_y * 100)]

        self.get_logger().info('Navigation succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()

    # Use multi-threaded executor for concurrent operation
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Key Server Concepts

**Goal callback**: Decides whether to accept the goal. Reject if:
- Robot is in error state
- Goal is invalid
- Another goal is already executing (if not supported)

**Cancel callback**: Decides whether to accept cancellation. Usually accept—safe stop should always be possible.

**Execute callback**: The main execution loop. Must:
- Check `is_cancel_requested` frequently
- Publish feedback periodically
- Call `succeed()`, `abort()`, or `canceled()` at the end

---

## Action Client Implementation {#action-client}

```python
#!/usr/bin/env python3
"""
Navigation Action Client

Sends navigation goals and monitors progress.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci


class NavigationClient(Node):
    """Action client for navigation goals."""

    def __init__(self):
        super().__init__('navigation_client')

        self._action_client = ActionClient(
            self,
            Fibonacci,
            'navigate_to_pose'
        )

        self._goal_handle = None

    def send_goal(self, distance: float):
        """Send navigation goal and set up callbacks."""
        self.get_logger().info(f'Sending navigation goal: {distance}m')

        # Wait for server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = int(distance)

        # Send goal with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when server accepts/rejects the goal."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Set up result callback
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when feedback is received."""
        feedback = feedback_msg.feedback.partial_sequence
        if len(feedback) >= 2:
            progress = feedback[0]
            remaining_cm = feedback[1]
            self.get_logger().info(
                f'Feedback: {progress}% complete, {remaining_cm}cm remaining'
            )

    def result_callback(self, future):
        """Called when action completes."""
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Navigation succeeded: {result.sequence}')
        elif status == 5:  # CANCELED
            self.get_logger().info('Navigation was canceled')
        elif status == 6:  # ABORTED
            self.get_logger().error('Navigation aborted')

    def cancel_goal(self):
        """Cancel the current goal."""
        if self._goal_handle is None:
            return

        self.get_logger().info('Canceling goal')
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Called when cancel request is processed."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancel accepted')
        else:
            self.get_logger().warn('Goal cancel rejected')


def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()

    # Send a goal
    client.send_goal(5.0)  # Navigate 5 meters

    # Spin to process callbacks
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.cancel_goal()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Safe Cancellation {#cancellation}

Cancellation in robotics must be **safe**—you can't instantly stop a moving robot.

### Physical Cancellation Requirements

**From M1 Actuator Knowledge**:

| System | Stop Distance | Stop Time | Safe Cancel Behavior |
|--------|---------------|-----------|---------------------|
| Mobile base at 1 m/s | 0.5-1.0 m | 0.5-1.0 s | Decelerate to stop |
| Robot arm moving | 0.1-0.2 m | 0.2-0.5 s | Complete current segment or stop |
| Gripper closing | N/A | Instant OK | Stop motor immediately |

### Cancel Implementation Pattern

```python
async def execute_callback(self, goal_handle):
    """Execute with safe cancellation."""

    while not goal_reached:
        # Check for cancellation
        if goal_handle.is_cancel_requested:
            # Safe stop sequence
            self.get_logger().info('Initiating safe stop')

            # 1. Command deceleration
            self.command_velocity(0.0)

            # 2. Wait for robot to stop (physical time)
            decel_time = self.current_velocity / self.max_deceleration
            await asyncio.sleep(decel_time)

            # 3. Confirm stopped
            if self.current_velocity < 0.01:
                goal_handle.canceled()
                return self.create_result(success=False, reason='canceled')
            else:
                goal_handle.abort()
                return self.create_result(success=False, reason='stop_failed')

        # Normal execution...
```

---

## Custom Action Types {#custom-actions}

Define action interfaces for your specific operations.

### Example: NavigateToPose.action

```
# Goal
geometry_msgs/PoseStamped target_pose
float32 max_velocity
bool allow_backward
---
# Result
bool success
string message
geometry_msgs/PoseStamped final_pose
float32 total_distance
float32 total_time
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 estimated_time_remaining
float32 current_velocity
```

### Design Guidelines

**Goal**: What the client wants
- Include all parameters needed to execute
- Include constraints (max velocity, allowed deviation)

**Feedback**: Progress updates
- Include current state
- Include estimates (time remaining, distance remaining)
- Keep small (sent frequently)

**Result**: Final outcome
- Include success/failure status
- Include final measurements
- Include error information if failed

---

## Services vs Actions Decision {#decision-framework}

```
Does operation take more than ~5 seconds?
├── No → Use SERVICE
└── Yes → Does client need progress updates?
    ├── No → Could still use ACTION (for cancellation)
    └── Yes → Use ACTION

Can operation be safely interrupted?
├── No → Use SERVICE (must complete)
└── Yes → Use ACTION (supports cancellation)
```

### Comparison Table

| Aspect | Service | Action |
|--------|---------|--------|
| Duration | Short (\<5s) | Any length |
| Progress | No feedback | Continuous feedback |
| Cancellation | Not supported | Supported |
| Complexity | Simple | More complex |
| Use case | Config, queries | Navigation, manipulation |

---

## Summary

This chapter covered actions for long-running robot operations:

1. **Actions** provide goal/feedback/result communication for operations that take significant time.

2. **Action servers** execute goals asynchronously, publishing periodic feedback and handling cancellation.

3. **Action clients** send goals, receive feedback callbacks, and can request cancellation.

4. **Safe cancellation** must respect physical constraints—robots can't stop instantly.

5. **Services vs Actions**: short/no-feedback → service; long/feedback-needed → action.

---

## Self-Assessment Questions

1. **Action Design**: Design an action interface for a pick-and-place operation. What goal, feedback, and result fields would you include?

2. **Cancellation Safety**: A robot arm is moving at 1 m/s when cancel is requested. If maximum deceleration is 2 m/s², how long must the cancel handler wait before reporting completion?

3. **Feedback Rate**: Navigation feedback is published at 10 Hz. The client displays progress on a screen refreshing at 30 Hz. Is there a problem? What about if feedback is at 1 Hz?

4. **State Handling**: Your action server receives a new goal while executing a previous goal. What are your options, and which is safest?

5. **Pattern Selection**: A robot performs a 3-second homing sequence that cannot be interrupted once started. Service or action? Why?

---

## What's Next

In [Chapter 5: URDF and Robot Description](/module-2/chapter-5-urdf-robot-description), you'll learn to describe robot structure and sensor placement using URDF, connecting to the physical robot models from Module 1.

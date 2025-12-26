---
id: chapter-3-services
title: "Chapter 3: Services"
sidebar_label: "3. Services"
sidebar_position: 4
---

# Chapter 3: Services for Synchronous Operations

## Chapter Goal

By the end of this chapter, you will be able to **implement the service pattern for discrete request/response operations**, with emphasis on configuration, calibration, and discrete actuator commands.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 3.1 | Implement a service server that handles requests with proper error handling |
| 3.2 | Implement a service client with timeout handling and response validation |
| 3.3 | Design service interfaces for discrete robot operations |
| 3.4 | Choose between topics and services based on communication requirements |

---

## When to Use Services

Topics are for continuous data streams. Services are for **discrete operations**:

| Use Topics When | Use Services When |
|-----------------|-------------------|
| Data flows continuously | Operation happens once |
| Many subscribers need data | One requester, one response |
| Latest data matters | Specific result matters |
| Loss is acceptable | Operation must complete |

**Physical Grounding Examples**:

| Operation | Pattern | Why |
|-----------|---------|-----|
| IMU readings | Topic | Continuous, 100 Hz stream |
| Calibrate IMU | Service | Discrete, wait for completion |
| Motor velocity | Topic | Continuous commands |
| Home the motor | Service | Discrete, wait until homed |
| Camera images | Topic | Continuous stream |
| Set camera exposure | Service | One-time configuration |

---

## Service Server Implementation {#service-server}

A service server waits for requests and returns responses.

### Gripper Service Example

```python
#!/usr/bin/env python3
"""
Gripper Service Server

Provides open/close operations for a gripper.
Physical grounding: Timing based on M1 actuator response characteristics.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time


class GripperService(Node):
    """Service server for gripper operations."""

    def __init__(self):
        super().__init__('gripper_service')

        # Physical parameters from M1 actuator specifications
        self.gripper_travel_time_sec = 2.0  # Time to open/close fully
        self.gripper_state = False  # False = open, True = closed

        # Create service server
        self.srv = self.create_service(
            SetBool,
            'gripper/command',
            self.gripper_callback
        )

        self.get_logger().info('Gripper service ready')

    def gripper_callback(self, request, response):
        """
        Handle gripper command.

        Args:
            request.data: True = close, False = open
            response.success: True if operation completed
            response.message: Status description
        """
        target_state = request.data
        action = "close" if target_state else "open"

        self.get_logger().info(f'Gripper command: {action}')

        # Check if already in desired state
        if target_state == self.gripper_state:
            response.success = True
            response.message = f'Gripper already {action}ed'
            return response

        # Simulate gripper motion (physical time from M1 specs)
        # In real system, this would command the actuator and wait
        self.get_logger().info(
            f'Moving gripper ({self.gripper_travel_time_sec}s)...'
        )
        time.sleep(self.gripper_travel_time_sec)

        # Update state
        self.gripper_state = target_state

        response.success = True
        response.message = f'Gripper {action}ed successfully'
        self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperService()

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

### Service Callback Requirements

1. **Must return a response**: Every callback must return the response object
2. **Should handle errors**: Set `response.success = False` on error
3. **Can take time**: Unlike topic callbacks, service callbacks can block

**Physical Grounding**: The 2-second gripper travel time comes from M1 actuator specifications. A pneumatic gripper might take 0.5s; a slow servo gripper might take 3s. Know your hardware.

---

## Service Client Implementation {#service-client}

A service client sends requests and waits for responses.

### Synchronous Client

```python
#!/usr/bin/env python3
"""
Gripper Service Client

Calls gripper service with timeout handling.
Physical grounding: Timeout based on M1 actuator timing + safety margin.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class GripperClient(Node):
    """Service client for gripper operations."""

    def __init__(self):
        super().__init__('gripper_client')

        # Create client
        self.client = self.create_client(SetBool, 'gripper/command')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')

        self.get_logger().info('Gripper client ready')

    def send_command(self, close: bool, timeout_sec: float = 5.0) -> bool:
        """
        Send gripper command and wait for response.

        Args:
            close: True to close, False to open
            timeout_sec: Maximum time to wait (from M1 specs + margin)

        Returns:
            True if operation succeeded
        """
        request = SetBool.Request()
        request.data = close

        action = "close" if close else "open"
        self.get_logger().info(f'Sending command: {action}')

        # Send request
        future = self.client.call_async(request)

        # Wait for response with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            self.get_logger().error(
                f'Service call timed out after {timeout_sec}s'
            )
            return False

        response = future.result()

        if response.success:
            self.get_logger().info(f'Success: {response.message}')
        else:
            self.get_logger().error(f'Failed: {response.message}')

        return response.success


def main(args=None):
    rclpy.init(args=args)
    client = GripperClient()

    # Example usage: open, close, close again
    client.send_command(close=False)  # Open
    client.send_command(close=True)   # Close
    client.send_command(close=True)   # Already closed

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Timeout Selection

**Physical Grounding**: The timeout must be longer than the physical operation:

| Operation | Physical Time | Recommended Timeout |
|-----------|---------------|---------------------|
| Gripper open/close | 2.0s | 5.0s (2.5x margin) |
| Motor home | 5.0s | 15.0s (3x margin) |
| Sensor calibration | 10.0s | 30.0s (3x margin) |

Too short: False failures when operation is still in progress
Too long: System hangs on real failures

---

## Custom Service Types {#custom-services}

Standard services (`std_srvs`) cover basic cases. For complex operations, define custom types.

### Defining a Service Interface

Create file `srv/CalibrateIMU.srv`:

```
# Request
bool full_calibration  # True for full, False for quick
---
# Response
bool success
string message
float64 accel_bias_x
float64 accel_bias_y
float64 accel_bias_z
float64 gyro_bias_x
float64 gyro_bias_y
float64 gyro_bias_z
```

**Structure**:
- Request fields above `---`
- Response fields below `---`

### Using Custom Services

After building the package:

```python
from my_robot_interfaces.srv import CalibrateIMU

# Server
self.srv = self.create_service(
    CalibrateIMU,
    'imu/calibrate',
    self.calibrate_callback
)

def calibrate_callback(self, request, response):
    if request.full_calibration:
        # Full calibration takes longer
        response.accel_bias_x = self.measure_accel_bias_x()
        # ... more measurements
    else:
        # Quick calibration
        response.accel_bias_x = 0.0

    response.success = True
    response.message = 'Calibration complete'
    return response
```

---

## Error Handling Patterns {#error-handling}

Robust service implementations handle errors gracefully.

### Server-Side Error Handling

```python
def gripper_callback(self, request, response):
    try:
        # Attempt operation
        if not self.gripper_connected:
            raise RuntimeError("Gripper not connected")

        self.execute_gripper_motion(request.data)

        response.success = True
        response.message = "Operation completed"

    except RuntimeError as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        self.get_logger().error(str(e))

    except Exception as e:
        response.success = False
        response.message = "Internal error"
        self.get_logger().error(f"Unexpected error: {e}")

    return response  # Always return response
```

### Client-Side Error Handling

```python
def send_command(self, close: bool) -> bool:
    # Check service availability
    if not self.client.service_is_ready():
        self.get_logger().error("Service not available")
        return False

    request = SetBool.Request()
    request.data = close

    try:
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error("Service call failed (no response)")
            return False

        response = future.result()
        return response.success

    except Exception as e:
        self.get_logger().error(f"Service call exception: {e}")
        return False
```

---

## Topics vs Services: Decision Framework {#decision-framework}

Use this flowchart to decide:

```
Is data continuous/streaming?
├── Yes → Use TOPIC
└── No → Is specific response needed?
    ├── No → Use TOPIC (one-way, fire-and-forget)
    └── Yes → Does operation take significant time?
        ├── No → Use SERVICE
        └── Yes → Need progress feedback?
            ├── No → Use SERVICE
            └── Yes → Use ACTION (Chapter 4)
```

### Decision Examples

| Scenario | Analysis | Choice |
|----------|----------|--------|
| Publish robot pose at 50 Hz | Continuous stream | Topic |
| Get current robot pose once | Need specific response, instant | Service |
| Command gripper open | Need confirmation, \<5s | Service |
| Navigate to goal | Long duration, need progress | Action |
| Emergency stop | One-way, fire-and-forget | Topic |
| Set motor PID gains | Configuration, need confirmation | Service |

---

## Summary

This chapter covered services for discrete operations:

1. **Services** provide request/response communication for operations that happen once and need confirmation.

2. **Service servers** handle requests in callbacks that can block—use for operations like gripper commands and calibration.

3. **Service clients** send requests and wait with timeouts—timeouts should be based on physical operation duration plus safety margin.

4. **Custom service types** enable rich request/response structures for complex operations.

5. **Error handling** is critical—both server and client must handle failures gracefully.

6. **Topics vs Services**: continuous/streaming → topics; discrete/response-needed → services.

---

## Self-Assessment Questions

1. **Service Design**: Design a service interface for a robot arm "home" operation that moves all joints to zero position. What fields should be in request and response?

2. **Timeout Selection**: A calibration procedure takes 10-15 seconds depending on sensor noise. What timeout would you set for the client? Why?

3. **Error Handling**: Your gripper service receives a "close" command but the gripper is already holding an object at maximum force. How should the server respond?

4. **Pattern Selection**: You need to implement an emergency stop. Should you use a topic or service? Justify your choice.

5. **Blocking Behavior**: A service callback takes 30 seconds to complete. What happens to other callbacks (topic subscriptions) on the same node during this time?

---

## What's Next

In [Chapter 4: Actions](/module-2/chapter-4-actions), you'll implement the action pattern for long-running tasks that need progress feedback and cancellation support.

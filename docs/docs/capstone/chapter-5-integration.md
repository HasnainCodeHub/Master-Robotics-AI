---
id: chapter-5-integration
title: "Chapter 5: Integration & Testing"
sidebar_label: "5. Integration"
sidebar_position: 6
---

# Chapter 5: Integration and Testing

## Chapter Goal

By the end of this chapter, you will have **integrated all subsystems into a complete VLA-controlled humanoid** and validated the system with comprehensive end-to-end testing.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 5.1 | Integrate VLA pipeline with execution systems |
| 5.2 | Implement the behavior coordinator |
| 5.3 | Create comprehensive test scenarios |
| 5.4 | Measure and optimize system performance |
| 5.5 | Prepare the demonstration |

---

## System Integration {#integration}

### Complete Integration Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      INTEGRATED HUMANOID SYSTEM                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                     VLA COMMAND PIPELINE                              │  │
│  │                                                                       │  │
│  │  Audio ──► Whisper ──► LLM Planner ──► Grounding ──► Safety Filter   │  │
│  │                              │               │             │          │  │
│  │                              │               │             │          │  │
│  │                         VLM Scene ◄───── Perception ◄─────┘          │  │
│  │                                                                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                    BEHAVIOR COORDINATOR                               │  │
│  │                                                                       │  │
│  │   Approved Action ──► Task Sequencer ──► Subsystem Dispatcher        │  │
│  │                              │                    │                   │  │
│  │                              │         ┌─────────┴──────────┐        │  │
│  │                              │         │                    │        │  │
│  │                              ▼         ▼                    ▼        │  │
│  │                         Navigation  Manipulation      Speech Output  │  │
│  │                                                                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Behavior Coordinator Implementation

```python
#!/usr/bin/env python3
"""Behavior coordinator connecting VLA to execution."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from humanoid_msgs.action import Pick, Place
from geometry_msgs.msg import Pose, PoseStamped
import json
import asyncio


class BehaviorCoordinator(Node):
    """Coordinate execution of VLA-generated task plans."""

    def __init__(self):
        super().__init__('behavior_coordinator')

        self.callback_group = ReentrantCallbackGroup()

        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'humanoid/navigate',
            callback_group=self.callback_group
        )
        self.pick_client = ActionClient(
            self, Pick, 'humanoid/pick',
            callback_group=self.callback_group
        )
        self.place_client = ActionClient(
            self, Place, 'humanoid/place',
            callback_group=self.callback_group
        )

        # Subscribers
        self.action_sub = self.create_subscription(
            String, '/safety/approved_action',
            self.action_callback, 10,
            callback_group=self.callback_group
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/behavior/status', 10
        )
        self.speech_pub = self.create_publisher(
            String, '/tts/input', 10
        )

        # State
        self.current_task = None
        self.held_object = None
        self.active_arm = 'right'

        # Task queue
        self.task_queue = asyncio.Queue()

        # Start task executor
        self.executor_task = asyncio.create_task(self.task_executor())

        self.get_logger().info('Behavior coordinator ready')

    def action_callback(self, msg: String):
        """Receive approved action from safety filter."""
        action = json.loads(msg.data)
        self.get_logger().info(f"Received action: {action['action_name']}")

        # Queue for execution
        asyncio.run_coroutine_threadsafe(
            self.task_queue.put(action),
            asyncio.get_event_loop()
        )

    async def task_executor(self):
        """Execute tasks from queue."""
        while True:
            action = await self.task_queue.get()

            try:
                success = await self.execute_action(action)

                if success:
                    self.publish_status('completed', f"Action complete: {action['action_name']}")
                    self.speak("Task completed successfully.")
                else:
                    self.publish_status('failed', f"Action failed: {action['action_name']}")
                    self.speak("I was unable to complete that task.")

            except Exception as e:
                self.get_logger().error(f"Execution error: {e}")
                self.publish_status('error', str(e))
                self.speak("An error occurred during execution.")

    async def execute_action(self, action: dict) -> bool:
        """Execute a single action."""
        action_name = action['action_name']
        goal = action['goal']

        self.publish_status('executing', f"Executing: {action_name}")

        if action_name == 'navigate_to_pose':
            return await self.execute_navigation(goal)

        elif action_name == 'pick':
            return await self.execute_pick(goal)

        elif action_name == 'place':
            return await self.execute_place(goal)

        elif action_name == 'pick_and_place':
            return await self.execute_pick_and_place(goal)

        else:
            self.get_logger().warn(f"Unknown action: {action_name}")
            return False

    async def execute_navigation(self, goal: dict) -> bool:
        """Execute navigation action."""
        self.speak("Moving to target location.")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return False

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.dict_to_pose_stamped(goal)

        result = await self.nav_client.send_goal_async(nav_goal)

        if not result.accepted:
            return False

        final_result = await result.get_result_async()
        return final_result.status == 4  # SUCCEEDED

    async def execute_pick(self, goal: dict) -> bool:
        """Execute pick action."""
        target_name = goal.get('target_name', 'object')
        self.speak(f"Picking up the {target_name}.")

        if not self.pick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Pick server not available")
            return False

        pick_goal = Pick.Goal()
        pick_goal.target_pose = self.dict_to_pose(goal['target_pose'])
        pick_goal.arm = self.active_arm
        pick_goal.object_width = goal.get('object_width', 0.05)

        result = await self.pick_client.send_goal_async(pick_goal)

        if not result.accepted:
            return False

        final_result = await result.get_result_async()

        if final_result.result.success:
            self.held_object = target_name
            return True
        return False

    async def execute_place(self, goal: dict) -> bool:
        """Execute place action."""
        location_name = goal.get('location_name', 'target location')
        self.speak(f"Placing object at {location_name}.")

        if not self.place_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Place server not available")
            return False

        place_goal = Place.Goal()
        place_goal.target_pose = self.dict_to_pose(goal['target_pose'])
        place_goal.arm = self.active_arm

        result = await self.place_client.send_goal_async(place_goal)

        if not result.accepted:
            return False

        final_result = await result.get_result_async()

        if final_result.result.success:
            self.held_object = None
            return True
        return False

    async def execute_pick_and_place(self, goal: dict) -> bool:
        """Execute combined pick and place sequence."""
        # Navigate to pick location if needed
        if goal.get('pick_requires_navigation', False):
            self.speak("Moving to pick up the object.")
            nav_success = await self.execute_navigation({
                'pose': goal['approach_pose']
            })
            if not nav_success:
                return False

        # Pick
        pick_success = await self.execute_pick({
            'target_pose': goal['pick_pose'],
            'target_name': goal.get('object_name', 'object'),
            'object_width': goal.get('object_width', 0.05)
        })
        if not pick_success:
            return False

        # Navigate to place location if needed
        if goal.get('place_requires_navigation', False):
            self.speak("Moving to the drop-off location.")
            nav_success = await self.execute_navigation({
                'pose': goal['place_approach_pose']
            })
            if not nav_success:
                # Try to return to start with object
                self.speak("Navigation failed. Returning to start position.")
                return False

        # Place
        place_success = await self.execute_place({
            'target_pose': goal['place_pose'],
            'location_name': goal.get('location_name', 'target')
        })

        return place_success

    def dict_to_pose(self, d: dict) -> Pose:
        """Convert dictionary to Pose message."""
        pose = Pose()
        pose.position.x = d['position']['x']
        pose.position.y = d['position']['y']
        pose.position.z = d['position']['z']
        pose.orientation.x = d['orientation']['x']
        pose.orientation.y = d['orientation']['y']
        pose.orientation.z = d['orientation']['z']
        pose.orientation.w = d['orientation']['w']
        return pose

    def dict_to_pose_stamped(self, d: dict) -> PoseStamped:
        """Convert dictionary to PoseStamped message."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = d.get('frame_id', 'map')
        pose_stamped.pose = self.dict_to_pose(d['pose'])
        return pose_stamped

    def publish_status(self, state: str, message: str):
        """Publish behavior status."""
        status = {'state': state, 'message': message}
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def speak(self, text: str):
        """Send text to speech synthesis."""
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)


def main():
    rclpy.init()
    node = BehaviorCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## End-to-End Pipeline {#end-to-end}

### Complete Launch File

```python
# launch/humanoid_full.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    humanoid_dir = get_package_share_directory('humanoid_bringup')

    return LaunchDescription([
        # ========== PERCEPTION ==========
        # Isaac Sim bridge (run separately)

        # VSLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(humanoid_dir, 'launch', 'vslam.launch.py')
            )
        ),

        # Object detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(humanoid_dir, 'launch', 'detection.launch.py')
            )
        ),

        # ========== NAVIGATION ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(humanoid_dir, 'launch', 'navigation.launch.py')
            )
        ),

        # ========== MANIPULATION ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(humanoid_dir, 'launch', 'moveit.launch.py')
            )
        ),

        # Pick/Place action servers
        Node(
            package='humanoid_manipulation',
            executable='pick_action_server',
            name='pick_action_server',
        ),
        Node(
            package='humanoid_manipulation',
            executable='place_action_server',
            name='place_action_server',
        ),

        # ========== VLA PIPELINE ==========
        # Audio capture
        Node(
            package='humanoid_vla',
            executable='audio_capture',
            name='audio_capture',
            parameters=[{'device_id': 0}]
        ),

        # Speech recognition
        Node(
            package='humanoid_vla',
            executable='speech_recognition',
            name='speech_recognition',
            parameters=[{'model_size': 'base'}]
        ),

        # Task planner
        Node(
            package='humanoid_vla',
            executable='task_planner',
            name='task_planner',
        ),

        # VLM (optional, resource intensive)
        Node(
            package='humanoid_vla',
            executable='vlm_node',
            name='vlm_node',
            condition=IfCondition(LaunchConfiguration('enable_vlm', default='false'))
        ),

        # Grounding
        Node(
            package='humanoid_vla',
            executable='grounding',
            name='grounding',
        ),

        # Safety filter
        Node(
            package='humanoid_vla',
            executable='safety_filter',
            name='safety_filter',
            parameters=[os.path.join(humanoid_dir, 'config', 'safety.yaml')]
        ),

        # ========== BEHAVIOR ==========
        Node(
            package='humanoid_behavior',
            executable='behavior_coordinator',
            name='behavior_coordinator',
        ),

        # Text to speech
        Node(
            package='humanoid_speech',
            executable='tts_node',
            name='tts_node',
        ),
    ])
```

---

## Test Scenarios {#testing}

### Test Suite Implementation

```python
#!/usr/bin/env python3
"""End-to-end test suite for humanoid system."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from dataclasses import dataclass
from typing import List, Optional
import asyncio


@dataclass
class TestResult:
    """Result of a single test."""
    name: str
    passed: bool
    duration: float
    message: str
    details: dict


class HumanoidTestSuite(Node):
    """Comprehensive test suite for humanoid system."""

    def __init__(self):
        super().__init__('test_suite')

        # Publishers
        self.command_pub = self.create_publisher(
            String, '/speech/text', 10
        )

        # Subscribers
        self.status_sub = self.create_subscription(
            String, '/behavior/status', self.status_callback, 10
        )

        self.last_status = None
        self.test_results: List[TestResult] = []

    def status_callback(self, msg: String):
        """Track behavior status."""
        self.last_status = json.loads(msg.data)

    async def wait_for_completion(self, timeout: float = 60.0) -> bool:
        """Wait for task completion."""
        start = time.time()

        while time.time() - start < timeout:
            if self.last_status:
                state = self.last_status.get('state')
                if state in ['completed', 'failed', 'error']:
                    return state == 'completed'
            await asyncio.sleep(0.1)

        return False

    def send_command(self, command: str):
        """Send voice command (simulated)."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.last_status = None
        self.get_logger().info(f'Sent command: "{command}"')

    async def run_test(self, name: str, command: str, timeout: float = 60.0) -> TestResult:
        """Run a single test."""
        self.get_logger().info(f'Running test: {name}')

        start = time.time()
        self.send_command(command)

        passed = await self.wait_for_completion(timeout)
        duration = time.time() - start

        result = TestResult(
            name=name,
            passed=passed,
            duration=duration,
            message=self.last_status.get('message', '') if self.last_status else 'Timeout',
            details=self.last_status or {}
        )

        self.test_results.append(result)
        return result

    async def run_all_tests(self):
        """Run complete test suite."""
        self.get_logger().info('Starting test suite')

        # Test 1: Simple pick
        await self.run_test(
            'simple_pick',
            'Pick up the red mug',
            timeout=30.0
        )

        # Test 2: Pick and place
        await self.run_test(
            'pick_and_place',
            'Put the blue box on the shelf',
            timeout=60.0
        )

        # Test 3: Navigation + manipulation
        await self.run_test(
            'navigate_and_pick',
            'Go to the table and pick up the book',
            timeout=90.0
        )

        # Test 4: Complex command
        await self.run_test(
            'bring_object',
            'Bring me the green bottle from the counter',
            timeout=120.0
        )

        # Test 5: Safety rejection
        await self.run_test(
            'safety_rejection',
            'Move at maximum speed through the doorway',
            timeout=10.0
        )
        # Expect this to fail (rejected by safety)
        self.test_results[-1].passed = not self.test_results[-1].passed

        # Test 6: Ambiguous reference handling
        await self.run_test(
            'ambiguous_reference',
            'Pick up the mug',  # Multiple mugs in scene
            timeout=30.0
        )

        self.print_results()

    def print_results(self):
        """Print test results summary."""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('TEST RESULTS')
        self.get_logger().info('=' * 60)

        passed = 0
        failed = 0

        for result in self.test_results:
            status = 'PASS' if result.passed else 'FAIL'
            self.get_logger().info(
                f'{status}: {result.name} ({result.duration:.1f}s)'
            )
            if not result.passed:
                self.get_logger().info(f'  Message: {result.message}')

            if result.passed:
                passed += 1
            else:
                failed += 1

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Passed: {passed}/{len(self.test_results)}')
        self.get_logger().info(f'Failed: {failed}/{len(self.test_results)}')
        self.get_logger().info('=' * 60)


async def main():
    rclpy.init()
    test_suite = HumanoidTestSuite()

    # Wait for system to be ready
    await asyncio.sleep(5.0)

    await test_suite.run_all_tests()

    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
```

---

## Performance Measurement {#performance}

### Latency Profiler

```python
#!/usr/bin/env python3
"""Profile end-to-end latency of humanoid system."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json
from collections import defaultdict


class LatencyProfiler(Node):
    """Measure latency at each pipeline stage."""

    def __init__(self):
        super().__init__('latency_profiler')

        self.stage_times = {}
        self.command_start_time = None

        # Subscribe to all stage outputs
        self.subscriptions = [
            self.create_subscription(
                String, '/speech/text', self.speech_callback, 10
            ),
            self.create_subscription(
                String, '/task_planner/plan', self.plan_callback, 10
            ),
            self.create_subscription(
                String, '/grounding/action', self.grounding_callback, 10
            ),
            self.create_subscription(
                String, '/safety/approved_action', self.safety_callback, 10
            ),
            self.create_subscription(
                String, '/behavior/status', self.behavior_callback, 10
            ),
        ]

        # Command publisher for testing
        self.command_pub = self.create_publisher(String, '/audio/raw', 10)

        self.get_logger().info('Latency profiler ready')

    def start_command(self):
        """Start timing a command."""
        self.command_start_time = time.time()
        self.stage_times = {'command_start': 0}

    def record_stage(self, stage: str):
        """Record time for a stage."""
        if self.command_start_time is not None:
            elapsed = (time.time() - self.command_start_time) * 1000
            self.stage_times[stage] = elapsed

    def speech_callback(self, msg: String):
        self.record_stage('speech_recognized')

    def plan_callback(self, msg: String):
        self.record_stage('plan_generated')

    def grounding_callback(self, msg: String):
        self.record_stage('grounded')

    def safety_callback(self, msg: String):
        self.record_stage('safety_checked')

    def behavior_callback(self, msg: String):
        status = json.loads(msg.data)
        if status.get('state') in ['executing', 'completed', 'failed']:
            self.record_stage(f"behavior_{status['state']}")
            self.print_latency_report()

    def print_latency_report(self):
        """Print latency breakdown."""
        self.get_logger().info('\n' + '-' * 50)
        self.get_logger().info('LATENCY BREAKDOWN')
        self.get_logger().info('-' * 50)

        prev_time = 0
        for stage, cumulative in self.stage_times.items():
            delta = cumulative - prev_time
            self.get_logger().info(f'{stage}: {delta:.0f}ms (total: {cumulative:.0f}ms)')
            prev_time = cumulative

        self.get_logger().info('-' * 50)

    async def run_latency_test(self, command: str, iterations: int = 5):
        """Run latency test multiple times."""
        latencies = defaultdict(list)

        for i in range(iterations):
            self.get_logger().info(f'Iteration {i+1}/{iterations}')

            self.start_command()

            # Send command
            msg = String()
            msg.data = command
            self.command_pub.publish(msg)

            # Wait for completion
            await asyncio.sleep(10.0)

            # Record latencies
            for stage, time_ms in self.stage_times.items():
                latencies[stage].append(time_ms)

        # Print statistics
        self.get_logger().info('\n' + '=' * 50)
        self.get_logger().info('LATENCY STATISTICS')
        self.get_logger().info('=' * 50)

        for stage, times in latencies.items():
            if times:
                avg = np.mean(times)
                std = np.std(times)
                self.get_logger().info(f'{stage}: {avg:.0f}ms +/- {std:.0f}ms')
```

### System Health Monitor

```python
class SystemHealthMonitor(Node):
    """Monitor overall system health."""

    def __init__(self):
        super().__init__('health_monitor')

        self.component_status = {}
        self.last_update = {}

        # Components to monitor
        self.components = [
            '/visual_slam/tracking/odometry',
            '/detections',
            '/scan',
            '/joint_states',
            '/behavior/status',
        ]

        # Create subscribers
        for topic in self.components:
            self.create_subscription(
                String, topic, lambda msg, t=topic: self.update_component(t), 10
            )

        # Health check timer
        self.create_timer(1.0, self.check_health)

    def update_component(self, topic: str):
        """Update component last seen time."""
        self.last_update[topic] = time.time()

    def check_health(self):
        """Check system health."""
        current_time = time.time()
        stale_threshold = 5.0  # seconds

        healthy = True
        status = []

        for topic in self.components:
            last = self.last_update.get(topic, 0)
            age = current_time - last

            if age > stale_threshold:
                status.append(f'STALE: {topic} ({age:.1f}s)')
                healthy = False
            else:
                status.append(f'OK: {topic}')

        # Log status
        if not healthy:
            self.get_logger().warn('System health issues detected:')
            for s in status:
                if 'STALE' in s:
                    self.get_logger().warn(f'  {s}')
        else:
            self.get_logger().debug('All components healthy')
```

---

## Demonstration Preparation {#demonstration}

### Demo Script

```python
#!/usr/bin/env python3
"""Demonstration script for humanoid system."""

import rclpy
from rclpy.node import Node
import time


class DemoRunner(Node):
    """Run demonstration scenarios."""

    def __init__(self):
        super().__init__('demo_runner')

        self.speech_pub = self.create_publisher(String, '/speech/text', 10)

    def announce(self, text: str):
        """Announce via TTS."""
        msg = String()
        msg.data = text
        # Would publish to TTS
        self.get_logger().info(f'Announcement: {text}')

    def send_command(self, command: str):
        """Send voice command."""
        msg = String()
        msg.data = command
        self.speech_pub.publish(msg)
        self.get_logger().info(f'Command: {command}')

    async def run_demo(self):
        """Run full demonstration."""

        self.announce("Welcome to the Physical AI Humanoid demonstration.")
        await asyncio.sleep(3)

        # Demo 1: Voice-controlled pick
        self.announce("First, I will demonstrate voice-controlled manipulation.")
        await asyncio.sleep(2)

        self.announce("Watch as I respond to the command: Pick up the red mug.")
        await asyncio.sleep(1)

        self.send_command("Pick up the red mug")
        await asyncio.sleep(15)

        # Demo 2: Navigation and manipulation
        self.announce("Next, I will navigate to a location and manipulate an object.")
        await asyncio.sleep(2)

        self.announce("Command: Go to the shelf and pick up the book.")
        await asyncio.sleep(1)

        self.send_command("Go to the shelf and pick up the book")
        await asyncio.sleep(45)

        # Demo 3: Safety
        self.announce("Now I will demonstrate safety constraints.")
        await asyncio.sleep(2)

        self.announce("Watch what happens when I receive an unsafe command.")
        await asyncio.sleep(1)

        self.send_command("Move at maximum speed through the room")
        await asyncio.sleep(5)

        self.announce("The safety system rejected the unsafe command and provided feedback.")
        await asyncio.sleep(3)

        # Demo 4: Complex task
        self.announce("Finally, a complex multi-step task.")
        await asyncio.sleep(2)

        self.announce("Command: Bring me the green bottle from the counter.")
        await asyncio.sleep(1)

        self.send_command("Bring me the green bottle from the counter")
        await asyncio.sleep(60)

        # Conclusion
        self.announce("This concludes the demonstration. Thank you for watching!")
```

---

## Summary

This chapter covered system integration and testing:

1. **Complete integration** connecting VLA pipeline to execution systems.

2. **Behavior coordinator** sequencing navigation and manipulation actions.

3. **End-to-end tests** validating all demonstration scenarios.

4. **Performance measurement** profiling latency and system health.

5. **Demonstration preparation** with scripted scenarios.

---

## Final Checklist

Before demonstration, verify:

### System Integration
- [ ] All nodes launch without errors
- [ ] Topics connected correctly (use `ros2 topic list`)
- [ ] Action servers responding (use `ros2 action list`)
- [ ] TF tree complete (use `ros2 run tf2_tools view_frames`)

### Functionality
- [ ] Voice commands transcribed correctly
- [ ] Task plans generated for test commands
- [ ] Objects detected and localized
- [ ] Navigation reaches goals
- [ ] Pick and place succeeds
- [ ] Safety rejects unsafe commands

### Performance
- [ ] End-to-end latency < 5 seconds
- [ ] No dropped frames or timeouts
- [ ] Recovery from failures works

### Demo Readiness
- [ ] Scene set up with objects
- [ ] Robot at starting position
- [ ] Audio capture working
- [ ] TTS audible

---

## Capstone Complete

Congratulations on completing the Integrated Humanoid System capstone!

You have built a complete voice-controlled humanoid robot system that:

- Accepts natural language commands via speech
- Understands commands using LLM task planning
- Perceives the environment with cameras and LiDAR
- Navigates safely to goals
- Manipulates objects with dual arms
- Enforces safety constraints
- Handles failures gracefully

This system represents the integration of all skills from Modules 1-5 into a production-ready robotic architecture.

---

## Next Steps

To continue your Physical AI journey:

1. **Extend capabilities**: Add new action types, objects, or scenarios
2. **Improve reliability**: Enhance failure detection and recovery
3. **Optimize performance**: Reduce latency, improve accuracy
4. **Transfer to hardware**: Apply sim-to-real techniques from Module 4
5. **Share your work**: Document and open-source your implementation

The skills you've developed form the foundation for building intelligent, embodied AI systems that can safely operate in the real world.

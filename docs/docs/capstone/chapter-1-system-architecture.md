---
id: chapter-1-system-architecture
title: "Chapter 1: System Architecture"
sidebar_label: "1. System Architecture"
sidebar_position: 2
---

# Chapter 1: System Architecture Design

## Chapter Goal

By the end of this chapter, you will have **designed a complete system architecture for the integrated humanoid system**, including component diagrams, ROS 2 node graphs, interface contracts, and latency budget allocation.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 1.1 | Design component hierarchy for integrated robotic system |
| 1.2 | Define ROS 2 node graph and communication patterns |
| 1.3 | Specify interface contracts between subsystems |
| 1.4 | Allocate latency budgets across the pipeline |
| 1.5 | Document failure modes and recovery responsibilities |

---

## System Overview {#overview}

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         HUMANOID SYSTEM ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                      COMMAND INTERFACE LAYER                          │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                   │  │
│  │  │   Audio     │  │   Speech    │  │    LLM      │                   │  │
│  │  │   Capture   │─►│   (Whisper) │─►│   Planner   │                   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘                   │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        INTELLIGENCE LAYER                             │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                   │  │
│  │  │    VLM      │  │  Grounding  │  │   Safety    │                   │  │
│  │  │   Scene     │─►│   Layer     │─►│   Filter    │                   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘                   │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        EXECUTION LAYER                                │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                   │  │
│  │  │ Navigation  │  │ Manipulation│  │   Behavior  │                   │  │
│  │  │  (Nav2)     │  │  (MoveIt2)  │  │ Coordinator │                   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘                   │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        PERCEPTION LAYER                               │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │  │
│  │  │   VSLAM     │  │   Object    │  │   Depth     │  │    IMU      │  │  │
│  │  │             │  │  Detection  │  │ Processing  │  │  Fusion     │  │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│                                    ▼                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                         HARDWARE LAYER                                │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │  │
│  │  │  Cameras    │  │   LiDAR     │  │    IMU      │  │  Actuators  │  │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Layer Responsibilities

| Layer | Primary Responsibility | Timing Constraint |
|-------|----------------------|-------------------|
| Command Interface | Convert voice to task plan | 200ms speech + 500ms LLM |
| Intelligence | Ground symbols, verify safety | 50ms grounding + 10ms safety |
| Execution | Execute navigation and manipulation | Real-time control |
| Perception | Provide world model | 30 Hz updates |
| Hardware | Interface with sensors/actuators | Hardware-determined |

---

## ROS 2 Node Graph {#node-graph}

### Complete Node Architecture

```python
#!/usr/bin/env python3
"""ROS 2 node graph for integrated humanoid system."""

# ============================================================================
# COMMAND INTERFACE NODES
# ============================================================================

# Node: audio_capture
# Responsibility: Capture microphone audio, detect speech
# Publishes: /audio/raw (AudioData), /audio/speech_detected (Bool)
# Subscribes: None
# Parameters: sample_rate (16000), device_id

# Node: speech_recognition
# Responsibility: Transcribe audio to text using Whisper
# Publishes: /speech/text (String), /speech/confidence (Float32)
# Subscribes: /audio/raw (AudioData)
# Parameters: model_size ("base"), language ("en")

# Node: task_planner
# Responsibility: Convert text command to task plan using LLM
# Publishes: /task_planner/plan (TaskPlan)
# Subscribes: /speech/text (String)
# Parameters: model ("gpt-4"), temperature (0)

# ============================================================================
# INTELLIGENCE NODES
# ============================================================================

# Node: vlm_scene_understanding
# Responsibility: Answer queries about camera images
# Publishes: /vlm/scene_description (SceneDescription)
# Subscribes: /camera/rgb (Image)
# Service: /vlm/query (VLMQuery)
# Parameters: model ("llava-1.5-7b")

# Node: grounding
# Responsibility: Map symbols to physical quantities
# Publishes: /grounding/action (GroundedAction)
# Subscribes: /task_planner/plan (TaskPlan), /detections (DetectionArray)
# Service: /grounding/resolve_object (ResolveObject)
# Parameters: None

# Node: safety_filter
# Responsibility: Validate actions against safety constraints
# Publishes: /safety/approved_action (GroundedAction), /safety/rejection (Rejection)
# Subscribes: /grounding/action (GroundedAction)
# Parameters: workspace_limits, max_velocity, max_force

# ============================================================================
# EXECUTION NODES
# ============================================================================

# Node: behavior_coordinator
# Responsibility: Coordinate navigation and manipulation sequences
# Publishes: /status/current_task (String)
# Subscribes: /safety/approved_action (GroundedAction)
# Action Client: /navigate_to_pose, /moveit/execute_trajectory

# Node: navigation (Nav2 stack)
# Responsibility: Plan and execute mobile base motion
# Action Server: /navigate_to_pose (NavigateToPose)
# Subscribes: /scan (LaserScan), /odom (Odometry)

# Node: manipulation (MoveIt2)
# Responsibility: Plan and execute arm motion
# Action Server: /moveit/execute_trajectory (ExecuteTrajectory)
# Subscribes: /joint_states (JointState)

# ============================================================================
# PERCEPTION NODES
# ============================================================================

# Node: vslam (Isaac ROS)
# Responsibility: Visual-inertial odometry and mapping
# Publishes: /visual_slam/tracking/odometry (Odometry)
# Subscribes: /camera/left/image (Image), /camera/right/image (Image), /imu (Imu)

# Node: object_detection (Isaac ROS)
# Responsibility: Detect and localize objects
# Publishes: /detections (DetectionArray)
# Subscribes: /camera/rgb (Image), /camera/depth (Image)

# Node: depth_processing
# Responsibility: Generate point clouds from depth
# Publishes: /points (PointCloud2)
# Subscribes: /camera/depth (Image), /camera/info (CameraInfo)
```

### Node Communication Diagram

```
┌─────────────┐     /audio/raw      ┌─────────────┐    /speech/text    ┌─────────────┐
│   audio_    │────────────────────►│   speech_   │───────────────────►│   task_     │
│   capture   │                     │ recognition │                    │   planner   │
└─────────────┘                     └─────────────┘                    └─────────────┘
                                                                              │
                                                                    /task_planner/plan
                                                                              │
                                                                              ▼
┌─────────────┐    /camera/rgb      ┌─────────────┐   /detections   ┌─────────────┐
│   camera_   │────────────────────►│   object_   │────────────────►│  grounding  │
│   driver    │                     │  detection  │                 │             │
└─────────────┘                     └─────────────┘                 └─────────────┘
                                                                          │
                                                                   /grounding/action
                                                                          │
                                                                          ▼
                                                                   ┌─────────────┐
                                                                   │   safety_   │
                                                                   │   filter    │
                                                                   └─────────────┘
                                                                          │
                                                              /safety/approved_action
                                                                          │
                                                                          ▼
                                                                   ┌─────────────┐
                                                                   │  behavior_  │
                                                                   │ coordinator │
                                                                   └─────────────┘
                                                                     │         │
                                                    NavigateToPose   │         │  ExecuteTrajectory
                                                                     ▼         ▼
                                                              ┌─────────┐ ┌─────────┐
                                                              │  Nav2   │ │ MoveIt2 │
                                                              └─────────┘ └─────────┘
```

---

## Interface Contracts {#interfaces}

### Task Plan Message

```python
# custom_msgs/msg/TaskPlan.msg

# Header
string command_id
string original_command
builtin_interfaces/Time timestamp

# Actions to execute
Action[] actions

# Metadata
string success_criteria
string[] abort_conditions
float32 confidence

---
# custom_msgs/msg/Action.msg

string action_type  # "navigate_to", "pick", "place", etc.
string target       # Object reference or null
ActionParameters parameters
string[] preconditions
string[] effects

---
# custom_msgs/msg/ActionParameters.msg

geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float32 speed
float32 force
```

### Grounded Action Message

```python
# custom_msgs/msg/GroundedAction.msg

# Original action
Action original_action

# Grounded values
geometry_msgs/PoseStamped target_pose
string target_object_id
float32 target_confidence

# Execution parameters
string action_server  # e.g., "/navigate_to_pose"
string action_type    # e.g., "nav2_msgs/action/NavigateToPose"
string goal_json      # Serialized goal message
```

### Detection Array Message

```python
# custom_msgs/msg/DetectionArray.msg

std_msgs/Header header
Detection[] detections

---
# custom_msgs/msg/Detection.msg

string class_name
int32 instance_id
float32 confidence
BoundingBox2D bbox_2d
geometry_msgs/Point position_3d
string color
```

### Service Definitions

```python
# custom_msgs/srv/ResolveObject.srv

# Request
string reference  # e.g., "red mug", "the box on the left"

---

# Response
bool success
Detection detected_object
string error_message
string[] ambiguous_candidates  # If multiple matches
```

---

## Latency Budget {#latency}

### End-to-End Latency Target

**Goal**: < 3 seconds from voice command to motion start

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    LATENCY BUDGET ALLOCATION                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Speech Capture    ──────────── 2000 ms (user speaking)                 │
│         │                                                               │
│         ▼                                                               │
│  Speech Recognition ─────────── 200 ms (Whisper base)                   │
│         │                                                               │
│         ▼                                                               │
│  LLM Planning ──────────────── 500 ms (GPT-4 or local)                  │
│         │                                                               │
│         ▼                                                               │
│  VLM Query (if needed) ─────── 2000 ms (optional)                       │
│         │                                                               │
│         ▼                                                               │
│  Grounding ─────────────────── 50 ms (perception lookup)                │
│         │                                                               │
│         ▼                                                               │
│  Safety Check ──────────────── 10 ms (constraint validation)            │
│         │                                                               │
│         ▼                                                               │
│  Motion Planning ───────────── 200 ms (MoveIt2 or Nav2)                 │
│         │                                                               │
│         ▼                                                               │
│  Execution Start ───────────── 0 ms (command sent)                      │
│                                                                         │
│  ═══════════════════════════════════════════════════════════════════   │
│  TOTAL (without VLM): ~960 ms command processing + 2000 ms speech      │
│  TOTAL (with VLM): ~2960 ms command processing + 2000 ms speech        │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Per-Component Requirements

| Component | Budget | Justification |
|-----------|--------|---------------|
| Audio capture | Real-time | Hardware constraint |
| Whisper (base) | 200 ms | Tested on 2s audio |
| LLM planning | 500 ms | API call with caching |
| VLM query | 2000 ms | Only when needed |
| Object detection | 33 ms | 30 Hz pipeline |
| Grounding | 50 ms | Lookup + transform |
| Safety filter | 10 ms | Constraint checks |
| Nav2 planning | 100 ms | Local planner |
| MoveIt2 planning | 200 ms | OMPL RRT |

### Latency Monitoring

```python
class LatencyMonitor(Node):
    """Monitor and report latency at each pipeline stage."""

    def __init__(self):
        super().__init__('latency_monitor')

        self.stage_times = {}
        self.budgets = {
            'speech': 200,
            'llm': 500,
            'vlm': 2000,
            'grounding': 50,
            'safety': 10,
            'planning': 200,
        }

    def record_stage(self, stage: str, latency_ms: float):
        """Record latency for a stage."""
        self.stage_times[stage] = latency_ms

        budget = self.budgets.get(stage, float('inf'))
        if latency_ms > budget:
            self.get_logger().warn(
                f'{stage} exceeded budget: {latency_ms:.0f}ms > {budget}ms'
            )

    def get_total_latency(self) -> float:
        """Get total pipeline latency."""
        return sum(self.stage_times.values())
```

---

## Failure Modes and Recovery {#failures}

### Failure Responsibility Matrix

| Component | Failure Mode | Detection | Recovery Owner |
|-----------|--------------|-----------|----------------|
| Audio capture | No audio | Timeout | Audio node |
| Speech recognition | Low confidence | Threshold | Speech node |
| LLM planner | Timeout | Timer | Planner node |
| LLM planner | Invalid output | Schema validation | Planner node |
| Grounding | Object not found | Empty result | Grounding node |
| Grounding | Ambiguous | Multiple matches | Behavior coordinator |
| Safety | Rejection | Safety filter | Behavior coordinator |
| Navigation | Blocked | Costmap | Behavior coordinator |
| Manipulation | Grasp failure | Force feedback | Behavior coordinator |

### Recovery Strategy Assignment

```python
RECOVERY_STRATEGIES = {
    # Audio failures
    'audio_timeout': {
        'owner': 'audio_capture',
        'strategy': 'prompt_user',
        'message': "I didn't hear anything. Could you repeat that?"
    },

    # Speech failures
    'low_confidence': {
        'owner': 'speech_recognition',
        'strategy': 'request_clarification',
        'message': "I'm not sure I understood. Did you say '{transcription}'?"
    },

    # LLM failures
    'llm_timeout': {
        'owner': 'task_planner',
        'strategy': 'retry_with_fallback',
        'max_retries': 2,
        'fallback': 'simplified_planning'
    },
    'invalid_plan': {
        'owner': 'task_planner',
        'strategy': 'regenerate',
        'max_retries': 1
    },

    # Grounding failures
    'object_not_found': {
        'owner': 'behavior_coordinator',
        'strategy': 'search_or_ask',
        'search_patterns': ['rotate_in_place', 'move_to_last_known'],
        'fallback': 'request_help'
    },
    'ambiguous_reference': {
        'owner': 'behavior_coordinator',
        'strategy': 'disambiguate',
        'method': 'ask_user'
    },

    # Safety failures
    'safety_rejection': {
        'owner': 'behavior_coordinator',
        'strategy': 'report_and_suggest',
        'include_suggestion': True
    },

    # Execution failures
    'navigation_blocked': {
        'owner': 'behavior_coordinator',
        'strategy': 'replan_or_wait',
        'wait_timeout': 10.0,
        'fallback': 'report_blocked'
    },
    'grasp_failure': {
        'owner': 'behavior_coordinator',
        'strategy': 'retry_with_adjustment',
        'max_retries': 2,
        'adjustment': 'offset_approach'
    }
}
```

---

## Configuration Management {#configuration}

### Launch File Structure

```python
# humanoid_bringup/launch/full_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Perception layer
        IncludeLaunchDescription(
            'isaac_ros_visual_slam/launch/visual_slam.launch.py'
        ),
        IncludeLaunchDescription(
            'isaac_ros_detection/launch/detection.launch.py'
        ),

        # Command interface
        Node(
            package='humanoid_command',
            executable='audio_capture',
            name='audio_capture',
            parameters=[{'sample_rate': 16000}]
        ),
        Node(
            package='humanoid_command',
            executable='speech_recognition',
            name='speech_recognition',
            parameters=[{'model_size': 'base'}]
        ),
        Node(
            package='humanoid_command',
            executable='task_planner',
            name='task_planner',
            parameters=[{'model': 'gpt-4'}]
        ),

        # Intelligence layer
        Node(
            package='humanoid_intelligence',
            executable='grounding',
            name='grounding'
        ),
        Node(
            package='humanoid_intelligence',
            executable='safety_filter',
            name='safety_filter',
            parameters=[{
                'max_velocity': 0.5,
                'max_force': 50.0,
                'workspace_min': [0.1, -0.6, 0.0],
                'workspace_max': [0.9, 0.6, 1.2]
            }]
        ),

        # Execution layer
        Node(
            package='humanoid_execution',
            executable='behavior_coordinator',
            name='behavior_coordinator'
        ),
        IncludeLaunchDescription(
            'nav2_bringup/launch/navigation.launch.py'
        ),
        IncludeLaunchDescription(
            'moveit2/launch/move_group.launch.py'
        ),
    ])
```

### Parameter Files

```yaml
# config/humanoid_params.yaml

humanoid_system:
  ros__parameters:
    # Latency budgets (ms)
    latency:
      speech_budget: 200
      llm_budget: 500
      grounding_budget: 50
      safety_budget: 10

    # Safety limits
    safety:
      max_linear_velocity: 0.5  # m/s
      max_angular_velocity: 1.0  # rad/s
      max_grip_force: 50.0  # N
      workspace:
        min: [0.1, -0.6, 0.0]
        max: [0.9, 0.6, 1.2]

    # Recovery parameters
    recovery:
      llm_max_retries: 2
      grasp_max_retries: 2
      navigation_wait_timeout: 10.0
```

---

## Summary

This chapter defined the system architecture for the integrated humanoid:

1. **Layer architecture** separates concerns: command interface, intelligence, execution, perception, hardware.

2. **ROS 2 node graph** defines components, topics, and communication patterns.

3. **Interface contracts** specify message formats for interoperability.

4. **Latency budgets** allocate time across the pipeline to meet responsiveness targets.

5. **Failure modes** are mapped to recovery owners and strategies.

6. **Configuration** is managed through launch files and parameter files.

---

## Design Review Checklist

Before proceeding, verify:

- [ ] All major components have defined ROS 2 nodes
- [ ] Topics and services are documented
- [ ] Latency budget totals < 3 seconds
- [ ] Every failure mode has a recovery owner
- [ ] Configuration is parameterized, not hardcoded

---

## What's Next

In [Chapter 2: Humanoid Robot Setup](/capstone/chapter-2-robot-setup), you'll configure the humanoid robot model in Isaac Sim with all required sensors and actuators.

---
id: chapter-1-ros2-architecture
title: "Chapter 1: ROS 2 Architecture"
sidebar_label: "1. ROS 2 Architecture"
sidebar_position: 2
---

# Chapter 1: ROS 2 Architecture and Physical Grounding

## Chapter Goal

By the end of this chapter, you will understand **how ROS 2's architecture addresses the physical communication challenges identified in Module 1**, and you will have a working development environment ready for the hands-on chapters that follow.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 1.1 | Explain why ROS 2's DDS-based architecture addresses real-time robotic communication needs |
| 1.2 | Configure and verify a ROS 2 development environment with rclpy |
| 1.3 | Explain QoS policies and map them to physical sensor/actuator requirements |
| 1.4 | Navigate the ROS 2 computation graph using command-line tools |

---

## Connecting Module 1 to ROS 2

In Module 1, you learned that physical robots face challenges that software-only systems don't:

- **Sensors** produce data at fixed rates with noise and latency
- **Actuators** have limited response times and physical constraints
- **The perception-action loop** must complete within timing budgets
- **Physical time** doesn't wait for your software

ROS 2 is designed specifically to address these challenges. Let's see how.

### The Communication Problem

A robotic system has many components that need to communicate:

```
┌─────────┐     ┌─────────┐     ┌─────────┐
│ Camera  │────►│ Detector│────►│ Planner │
└─────────┘     └─────────┘     └─────────┘
                                     │
┌─────────┐     ┌─────────┐         │
│  LiDAR  │────►│ Mapper  │◄────────┘
└─────────┘     └─────────┘
                     │
                     ▼
               ┌─────────┐     ┌─────────┐
               │ Control │────►│ Motors  │
               └─────────┘     └─────────┘
```

Each connection has requirements:
- Camera → Detector: high bandwidth, can drop frames
- LiDAR → Mapper: must receive all scans
- Control → Motors: strict timing, must not be delayed

**ROS 2 provides a framework where you declare these requirements and the system ensures they're met.**

---

## ROS 2 Architecture Overview {#architecture-overview}

### Nodes

A **node** is a process that performs computation. Nodes are:
- Single-purpose (one node per function)
- Independently restartable
- Composable (can run in same process or separate)

**Physical Grounding**: Each component of the perception-action loop from M1 Chapter 4 becomes a node. The sensor, perception, planning, and execution stages are separate nodes that communicate.

### Topics

**Topics** are named channels for streaming data:
- Publishers send messages to a topic
- Subscribers receive messages from a topic
- Many-to-many: multiple publishers, multiple subscribers

**Physical Grounding**: Sensor data streams are topics. The IMU publishes to `/imu/data` at 100 Hz. Any node needing IMU data subscribes.

### Services

**Services** are named request/response operations:
- Client sends a request
- Server processes and returns response
- One-to-one: each request goes to one server

**Physical Grounding**: Discrete operations like "calibrate sensor" or "open gripper" are services.

### Actions

**Actions** are named long-running operations with feedback:
- Client sends a goal
- Server executes, sending progress feedback
- Client can monitor and cancel

**Physical Grounding**: Navigation goals and manipulation sequences are actions—they take time, need progress updates, and may need cancellation.

### DDS Middleware

ROS 2 uses **DDS (Data Distribution Service)** as its underlying communication layer:
- Industry standard for real-time data distribution
- Quality of Service (QoS) policies
- Discovery and peer-to-peer communication
- No central point of failure

**Physical Grounding**: DDS was designed for real-time systems. The QoS policies directly address the timing requirements from M1 Chapter 4.

---

## Quality of Service (QoS) Policies {#qos-policies}

QoS policies let you specify communication requirements. ROS 2 enforces them.

### Key QoS Policies

| Policy | Options | Physical Meaning |
|--------|---------|------------------|
| **Reliability** | `RELIABLE`, `BEST_EFFORT` | Must every message arrive, or is latest-only OK? |
| **Durability** | `VOLATILE`, `TRANSIENT_LOCAL` | Should late subscribers get recent messages? |
| **Deadline** | Time duration | How often must messages arrive? |
| **Lifespan** | Time duration | How long is a message valid? |
| **History** | `KEEP_LAST(n)`, `KEEP_ALL` | How many messages to buffer? |

### Mapping Physical Requirements to QoS

**From M1 Chapter 2 (Sensors)**:

| Sensor | Rate | Loss Tolerance | QoS Setting |
|--------|------|----------------|-------------|
| Camera (30 fps) | 33 ms | Can drop frames | `BEST_EFFORT`, `KEEP_LAST(1)` |
| LiDAR (10 Hz) | 100 ms | Need all scans | `RELIABLE`, `KEEP_LAST(10)` |
| IMU (100 Hz) | 10 ms | Latest only | `BEST_EFFORT`, `KEEP_LAST(1)` |
| Emergency stop | Infrequent | Must not miss | `RELIABLE`, `TRANSIENT_LOCAL` |

**From M1 Chapter 3 (Actuators)**:

| Command | Tolerance | QoS Setting |
|---------|-----------|-------------|
| Velocity commands | Stale is dangerous | `BEST_EFFORT`, `LIFESPAN(100ms)` |
| Joint positions | Must arrive | `RELIABLE`, `DEADLINE(10ms)` |
| Gripper open/close | Must complete | Service (not topic) |

### QoS Compatibility

Publisher and subscriber QoS must be compatible:

| Publisher | Subscriber | Result |
|-----------|------------|--------|
| `RELIABLE` | `RELIABLE` | OK |
| `RELIABLE` | `BEST_EFFORT` | OK |
| `BEST_EFFORT` | `RELIABLE` | FAIL - incompatible |
| `BEST_EFFORT` | `BEST_EFFORT` | OK |

**Physical Grounding**: If a safety-critical subscriber requires `RELIABLE` but the sensor publishes `BEST_EFFORT`, the connection fails. This is intentional—it prevents misconfigured systems.

---

## Development Environment Setup {#environment-setup}

### Installing ROS 2

For this course, we use **ROS 2 Humble** on **Ubuntu 22.04**.

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### Verifying Installation

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Verify with ros2 doctor
ros2 doctor --report

# Test basic communication
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

### Creating a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build (even with empty src)
colcon build

# Source workspace
source install/setup.bash
```

### Creating a Package

```bash
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot_pkg \
    --dependencies rclpy std_msgs sensor_msgs

# Build
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

---

## Command-Line Tools {#command-line-tools}

ROS 2 provides tools to inspect running systems—essential for debugging.

### Node Inspection

```bash
# List all running nodes
ros2 node list

# Get info about a node
ros2 node info /talker
```

### Topic Inspection

```bash
# List all topics
ros2 topic list

# Show topic message type
ros2 topic info /chatter

# Echo messages (see data)
ros2 topic echo /chatter

# Check publication rate
ros2 topic hz /chatter

# Check bandwidth
ros2 topic bw /chatter
```

### Service Inspection

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /spawn

# Call a service
ros2 service call /spawn std_srvs/srv/Empty
```

### Graph Visualization

```bash
# Install rqt
sudo apt install ros-humble-rqt ros-humble-rqt-graph

# Launch graph visualizer
rqt_graph
```

**Physical Grounding**: These tools let you verify that your system matches the perception-action loop diagram from M1. You can see which nodes exist, what topics connect them, and whether data is flowing at expected rates.

---

## The ROS 2 Computation Graph {#computation-graph}

The **computation graph** is the network of nodes and their connections.

### Example: Mobile Robot

```
             ┌──────────────┐
             │  /camera_node │
             └──────┬───────┘
                    │ /camera/image
                    ▼
             ┌──────────────┐       ┌──────────────┐
             │ /detector_node│──────►│ /planner_node │
             └──────────────┘       └──────┬───────┘
                                          │ /cmd_vel
                                          ▼
┌──────────────┐                   ┌──────────────┐
│  /lidar_node │─────────────────►│/controller_node│
└──────────────┘  /scan           └──────────────┘
```

### Mapping to M1 Perception-Action Loop

| M1 Component | ROS 2 Implementation |
|--------------|---------------------|
| **Sensors** | Publisher nodes |
| **Perception** | Subscriber + processing nodes |
| **Planning** | Planning nodes (actions) |
| **Execution** | Controller nodes |
| **Actuators** | Subscriber nodes (or hardware interfaces) |

### Timing in the Graph

Each arrow represents latency:
- `/camera_node` → `/detector_node`: message serialization + network + deserialization
- Processing within each node adds computation time
- Total latency = sum of all edges + sum of all node processing

**Physical Grounding**: The latency budget from M1 Chapter 4 is now distributed across nodes and connections. QoS deadline policies can enforce that each stage meets its budget.

---

## Summary

This chapter connected Module 1's physical concepts to ROS 2's architecture:

1. **ROS 2's architecture** (nodes, topics, services, actions) maps directly to the perception-action loop components from Module 1.

2. **DDS middleware** provides real-time communication with Quality of Service guarantees.

3. **QoS policies** are the implementation of M1's physical constraints—deadline enforces timing budgets, reliability matches sensor loss tolerance.

4. **The computation graph** is your perception-action loop made concrete, with nodes connected by typed messages.

5. **Command-line tools** let you inspect and debug the running system, verifying that reality matches your design.

---

## Self-Assessment Questions

1. **DDS and Real-Time**: Explain how DDS's QoS policies address the timing requirements from M1 Chapter 4.

2. **QoS Selection**: A safety system must receive emergency stop commands with zero tolerance for missed messages. What QoS settings would you use? Why?

3. **Node Design**: Your perception-action loop from M1 has 5 stages. How would you decompose this into ROS 2 nodes? What are the tradeoffs of more vs fewer nodes?

4. **Tool Usage**: You suspect a node is publishing too slowly. Which command-line tools would you use to diagnose this?

5. **Graph Analysis**: Draw the computation graph for a robot that reads from a camera and LiDAR, fuses the data, plans a path, and sends velocity commands. Label each topic with the message type and expected QoS.

---

## What's Next

In [Chapter 2: Topics and Publishers/Subscribers](/module-2/chapter-2-topics-pubsub), you'll implement the publish/subscribe pattern for sensor data streams, putting the QoS concepts from this chapter into practice.

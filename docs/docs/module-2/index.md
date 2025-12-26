---
id: index
title: "Module 2: ROS 2 Fundamentals"
sidebar_label: Overview
sidebar_position: 1
---

# Module 2: ROS 2 Fundamentals

**Duration**: 3 weeks (21-24 hours)

## Module Goal

By the end of this module, you will be able to **design, implement, and deploy ROS 2 node architectures using Python (rclpy)** that handle sensor data streams, command actuators, coordinate multi-component systems, and describe robots using URDF—all while respecting the physical constraints established in Module 1.

This module transforms conceptual understanding into executable code. You will not just run tutorials; you will build **production-ready patterns** for robotic communication that you will use throughout the remaining modules and capstone.

## What You Will Learn

| Chapter | Title | What You'll Be Able To Do |
|---------|-------|---------------------------|
| 1 | ROS 2 Architecture | Understand DDS-based architecture and configure QoS for physical requirements |
| 2 | Topics & Pub/Sub | Implement sensor data streams with appropriate QoS settings |
| 3 | Services | Implement request/response patterns for discrete operations |
| 4 | Actions | Implement long-running tasks with feedback and cancellation |
| 5 | URDF & Robot Description | Write and validate robot descriptions with sensors and joints |
| 6 | Integration Patterns | Build Python agents that coordinate all communication patterns |

## Capabilities You Will Gain

After completing this module, you CAN DO the following:

- **M2-C1**: Design ROS 2 node architectures with explicit timing requirements
- **M2-C2**: Implement publisher/subscriber patterns with QoS based on physical sensor characteristics
- **M2-C3**: Implement service patterns for discrete operations with timeout handling
- **M2-C4**: Implement action patterns for long-running tasks with feedback and cancellation
- **M2-C5**: Write and validate URDF descriptions for articulated robots
- **M2-C6**: Build Python agents that coordinate multiple ROS 2 communication patterns

## Prerequisites (from Module 1)

This module directly applies the concepts from Module 1:

| M1 Capability | How It Applies Here |
|---------------|---------------------|
| **M1-C1** (Embodied Intelligence) | Understanding why ROS 2's design addresses real-time constraints |
| **M1-C2** (Sensor Analysis) | Configuring sensor topics with appropriate QoS for noise/latency |
| **M1-C3** (Actuator Analysis) | Designing command topics with rate limits and safety bounds |
| **M1-C4** (Physical Constraints) | Setting QoS policies, timer rates, and timeout values |
| **M1-C5** (Perception-Action Loop) | Designing node graphs that implement the complete loop |

## Pacing Guidance

| Week | Chapters | Focus |
|------|----------|-------|
| Week 1 | 1-2 | Architecture + Topics |
| Week 2 | 3-4 | Services + Actions |
| Week 3 | 5-6 | URDF + Integration |

**Note**: Chapter 4 (Actions) is the most challenging. Allow extra time if needed. Chapter 6 should feel like consolidation, not new learning.

## How This Module Connects

This module provides the implementation foundation for everything that follows:

- **Module 3 (Simulation)**: These nodes will connect to physics-simulated robots
- **Module 4 (Isaac)**: Isaac ROS packages follow these same patterns
- **Module 5 (VLA)**: VLA pipelines are implemented as ROS 2 node networks
- **Capstone**: The complete humanoid system is a ROS 2 node architecture

**Key insight**: Your code will not change between simulation and reality—only the source of sensor data and destination of commands changes.

## Ready to Begin?

Start with [Chapter 1: ROS 2 Architecture](/module-2/chapter-1-ros2-architecture) to understand how ROS 2 addresses the physical communication challenges identified in Module 1.

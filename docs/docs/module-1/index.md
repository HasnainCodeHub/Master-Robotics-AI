---
id: index
title: "Module 1: Physical AI Foundations"
sidebar_label: Overview
sidebar_position: 1
---

# Module 1: Physical AI Foundations

**Duration**: 2 weeks (14-16 hours)

## Module Goal

By the end of this module, you will be able to **analyze any robotic system and identify its embodied intelligence characteristics**: what sensors it uses to perceive, what actuators it uses to act, and what physical constraints (latency, noise, uncertainty) govern its behavior.

This is the foundational mental model for the entire course. Without this grounding, you might treat robots as software systems that happen to have hardware attached. Instead, you will learn to see them as **embodied agents where physics is a first-class concern**.

## What You Will Learn

| Chapter | Title | What You'll Be Able To Do |
|---------|-------|---------------------------|
| 1 | Embodied Intelligence | Explain why physical AI is fundamentally different from disembodied AI |
| 2 | Sensor Fundamentals | Analyze sensors, predict their noise characteristics, and select appropriate sensors for tasks |
| 3 | Actuator Fundamentals | Analyze actuators, predict their response characteristics, and match them to requirements |
| 4 | Physical Constraints | Trace the complete perception-action loop and specify physical constraints for robotic tasks |

## Capabilities You Will Gain

After completing this module, you CAN DO the following:

- **M1-C1**: Analyze a robotic system and articulate why embodied AI differs from disembodied AI
- **M1-C2**: Categorize sensors by modality and predict their noise characteristics, latency, and failure modes
- **M1-C3**: Categorize actuators by type and predict their response characteristics and safety constraints
- **M1-C4**: Identify the physical constraints (latency, noise, uncertainty) that affect a robotic task
- **M1-C5**: Trace the complete perception-action loop and identify bottlenecks

## Prerequisites

Before starting this module, you should have:

| Skill | Minimum Level | How to Verify |
|-------|---------------|---------------|
| Python Programming | Intermediate | Can you write a class that reads from a file and handles errors? |
| Basic Robotics Concepts | Conceptual | Can you name three types of sensors and what data each provides? |
| Linear Algebra | Basics | Can you multiply two 3x3 matrices? |
| Scientific Thinking | Applied | Can you explain why a 1mm ruler can't measure 0.1mm? |

## Pacing Guidance

| Week | Chapters | Focus |
|------|----------|-------|
| Week 1 | 1-2 | Conceptual foundation + Sensors |
| Week 2 | 3-4 | Actuators + Integration |

**Note**: Chapter 2 (Sensors) is the densest content. Allow extra time if needed. Chapter 4 should feel like consolidation, not new learning.

## How This Module Connects

This module establishes the mental model that every subsequent module builds upon:

- **Module 2 (ROS 2)**: The sensors and actuators you analyze here become topics you subscribe to and publish to
- **Module 3 (Simulation)**: The physical characteristics you study here become noise models in simulation
- **Module 4 (Isaac)**: The constraints you specify here become performance requirements for GPU-accelerated systems
- **Module 5 (VLA)**: The perception-action loop you learn here becomes the pipeline that voice commands flow through

## Ready to Begin?

Start with [Chapter 1: Embodied Intelligence](/module-1/chapter-1-embodied-intelligence) to understand why physical AI is fundamentally different from the AI you may have encountered in software-only contexts.

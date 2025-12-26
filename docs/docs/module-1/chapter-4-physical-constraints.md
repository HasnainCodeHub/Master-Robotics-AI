---
id: chapter-4-physical-constraints
title: "Chapter 4: Physical Constraints and the Perception-Action Loop"
sidebar_label: "4. Physical Constraints"
sidebar_position: 5
---

# Chapter 4: Physical Constraints and the Perception-Action Loop

## Chapter Goal

By the end of this chapter, you will be able to **synthesize your knowledge of sensors and actuators into a complete system view**, analyzing the timing budgets, uncertainty propagation, and physical constraints that govern real robotic systems. You will create physical constraints specifications for realistic tasks.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 4.1 | Draw and label the complete perception-action loop for a given robotic system |
| 4.2 | Calculate end-to-end latency budget for a perception-action loop |
| 4.3 | Trace how sensor uncertainty propagates through processing to action uncertainty |
| 4.4 | Identify timing constraints (hard real-time, soft real-time, best-effort) for system components |
| 4.5 | Design a physical constraints specification for a given robotic task |

---

## The Problem: When Components Work but Systems Fail

An engineer builds a robot with:
- A camera that works perfectly (30 fps, sharp images)
- An object detector that works perfectly (95% accuracy)
- A motion planner that works perfectly (optimal paths)
- A motor controller that works perfectly (precise tracking)

Each component is tested and validated individually. But when integrated, the robot repeatedly misses moving objects.

What went wrong?

- Camera: 30 fps → 33 ms between frames
- Object detection: 50 ms processing time
- Motion planning: 20 ms computation
- Motor response: 80 ms to reach target velocity
- **Total**: 33 + 50 + 20 + 80 = **183 ms**

An object moving at 1 m/s travels **18 cm** in this time. The robot sees where the object *was*, not where it *is*.

**The components work perfectly. The system fails because the engineer didn't analyze the complete perception-action loop.**

---

## The Complete Perception-Action Loop {#complete-loop}

We introduced the perception-action loop in Chapter 1. Now we'll analyze it in full detail.

### Loop Components

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ENVIRONMENT                                  │
│                                                                      │
│   Object State ──────────────────────────────► Object State         │
│   at time t                                    at time t + Δt       │
│        │                                            ▲               │
│        │ Physical                                   │ Physical      │
│        │ Process                                    │ Change        │
│        ▼                                            │               │
└────────┼────────────────────────────────────────────┼───────────────┘
         │                                            │
         │ Light, Sound,                              │ Force, Motion
         │ Force, etc.                                │
         ▼                                            │
┌─────────────────┐                          ┌─────────────────┐
│    SENSORS      │                          │   ACTUATORS     │
│                 │                          │                 │
│ • Transduction  │                          │ • Amplification │
│ • Sampling      │                          │ • Conversion    │
│ • Digitization  │                          │ • Transmission  │
│                 │                          │                 │
│ Latency: τ_s    │                          │ Latency: τ_a    │
└────────┬────────┘                          └────────▲────────┘
         │                                            │
         │ Sensor Data                       Commands │
         │ (discrete, noisy)                 (discrete)│
         ▼                                            │
┌─────────────────────────────────────────────────────┴────────────────┐
│                           PROCESSING                                  │
│                                                                       │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐           │
│  │  PERCEPTION  │───►│   PLANNING   │───►│  EXECUTION   │           │
│  │              │    │              │    │              │           │
│  │ • Detection  │    │ • Path plan  │    │ • Trajectory │           │
│  │ • Estimation │    │ • Task plan  │    │ • Control    │           │
│  │ • Fusion     │    │ • Decision   │    │ • Safety     │           │
│  │              │    │              │    │              │           │
│  │ Latency: τ_p │    │ Latency: τ_l │    │ Latency: τ_e │           │
│  └──────────────┘    └──────────────┘    └──────────────┘           │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### Latency Budget

**Total loop latency** = τ_s + τ_p + τ_l + τ_e + τ_a

| Component | Symbol | Typical Range | Example |
|-----------|--------|---------------|---------|
| Sensor | τ_s | 1-100 ms | Camera: 33 ms (at 30 fps) |
| Perception | τ_p | 10-200 ms | Object detection: 50 ms |
| Planning | τ_l | 1-100 ms | Motion planning: 20 ms |
| Execution | τ_e | 0.1-10 ms | Control loop: 1 ms |
| Actuator | τ_a | 10-200 ms | Motor response: 80 ms |

### Why Latency Matters

For a dynamic task, the system must respond faster than the environment changes:

| Task | Required Response Time | Typical Latency | Feasible? |
|------|------------------------|-----------------|-----------|
| Balancing (standing) | 50-100 ms | 30-50 ms | Yes |
| Catching thrown ball | 200-400 ms | 150-200 ms | Marginal |
| Avoiding fast obstacle | 100-200 ms | 150-200 ms | Marginal |
| Tracking walking person | 500+ ms | 150-200 ms | Yes |
| Static manipulation | Seconds | 150-200 ms | Yes |

**Physical Grounding**: The latency budget is a hard physical constraint. No amount of software optimization can make a 30 fps camera produce data faster than 33 ms per frame.

---

## Latency Budget Analysis {#latency-analysis}

Let's work through a detailed example.

### Example: Ball-Catching Robot

**Task**: Catch a ball thrown at 10 m/s from 3 meters away

**Available time**: 3 m ÷ 10 m/s = **300 ms**

**System components**:

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 8 ms | 120 fps high-speed camera |
| Image transfer | 2 ms | USB 3.0 |
| Ball detection | 15 ms | GPU-accelerated |
| Trajectory prediction | 5 ms | Kalman filter |
| Motion planning | 10 ms | Pre-computed catch poses |
| Command transmission | 1 ms | EtherCAT |
| Motor response | 60 ms | Including acceleration |

**Total**: 8 + 2 + 15 + 5 + 10 + 1 + 60 = **101 ms**

**Margin**: 300 - 101 = **199 ms** for trajectory updates

**Conclusion**: System is feasible with significant margin for multiple perception-action cycles during the catch.

### What If We Used Standard Components?

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 33 ms | 30 fps standard camera |
| Image transfer | 10 ms | USB 2.0 |
| Ball detection | 100 ms | CPU processing |
| Trajectory prediction | 20 ms | Non-optimized |
| Motion planning | 50 ms | Full replan each cycle |
| Command transmission | 5 ms | CAN bus |
| Motor response | 100 ms | Consumer motors |

**Total**: 33 + 10 + 100 + 20 + 50 + 5 + 100 = **318 ms**

**Result**: **System cannot catch the ball** (318 ms > 300 ms available)

---

## Uncertainty Propagation {#uncertainty-propagation}

Sensor noise doesn't stay at the sensor—it propagates through the entire system.

### Sources of Uncertainty

| Stage | Uncertainty Source | Typical Magnitude |
|-------|-------------------|-------------------|
| Sensor | Measurement noise | ±1-10 cm (position), ±0.1-1° (angle) |
| Perception | Detection error | ±5-20 cm (object position) |
| State estimation | Filter uncertainty | Depends on motion model |
| Planning | Model error | Path may not be executable |
| Execution | Tracking error | ±1-5 mm (industrial), ±1-5 cm (mobile) |
| Actuator | Positioning error | ±0.1-10 mm (depends on type) |

### Propagation Example

Consider a robot arm picking an object:

**Camera measures object at position (x, y, z) with uncertainty σ_cam = ±5 mm**

↓

**Transform to robot frame adds calibration error: σ_cal = ±2 mm**

Combined: σ = √(5² + 2²) = **±5.4 mm**

↓

**Inverse kinematics has numerical error: σ_ik = ±0.5 mm**

Combined: σ = √(5.4² + 0.5²) = **±5.4 mm** (IK error negligible)

↓

**Joint controllers have tracking error: σ_joint = ±0.1° per joint**

For 6 joints at 0.5 m arm length: σ_ee ≈ **±3 mm**

Combined: σ = √(5.4² + 3²) = **±6.2 mm**

↓

**Gripper positioning error: σ_grip = ±1 mm**

**Final end-effector uncertainty: σ_total = √(6.2² + 1²) ≈ ±6.3 mm**

**Physical Grounding**: If the object is 5 mm smaller than the gripper opening, the ±6.3 mm uncertainty means the grasp may fail 30%+ of the time. The camera noise propagates all the way to grasp success rate.

### Reducing Uncertainty

| Strategy | Mechanism | Cost |
|----------|-----------|------|
| Better sensors | Lower σ_cam | Higher $$$ |
| Better calibration | Lower σ_cal | Time, expertise |
| Sensor fusion | Combine multiple measurements | Complexity |
| Visual servoing | Closed-loop with vision | Slower, complex |
| Compliance | Tolerate position error | May not work for all tasks |

---

## Real-Time Requirements {#realtime-requirements}

Not all timing constraints are equal. Understanding the taxonomy helps set correct priorities.

### Hard Real-Time

**Definition**: Missing a deadline causes system failure.

**Examples**:
- Motor control loop (1 kHz): Miss deadline → unstable control, motor damage
- Balance control (100 Hz): Miss deadline → robot falls
- Safety monitoring: Miss deadline → collision not avoided

**Implementation**: Requires real-time operating system (RTOS), deterministic hardware, careful software design.

### Soft Real-Time

**Definition**: Missing occasional deadlines degrades performance but doesn't cause failure.

**Examples**:
- Path planning updates: Late plan → suboptimal path, not failure
- Display updates: Delayed frame → visual glitch, not failure
- Logging: Late log → data out of order, not failure

**Implementation**: Can use standard OS with careful priority management.

### Best-Effort

**Definition**: No deadline; complete when resources allow.

**Examples**:
- Map building in SLAM
- Learning/adaptation
- User interface updates

### Classifying System Components

| Component | Typical Requirement | Justification |
|-----------|---------------------|---------------|
| Motor current control | Hard (1-10 kHz) | Motor physics |
| Joint position control | Hard (100-1000 Hz) | Stability |
| Balance/stabilization | Hard (50-200 Hz) | Don't fall |
| Collision avoidance | Hard (10-100 Hz) | Safety |
| Path tracking | Soft (10-50 Hz) | Performance |
| Perception pipeline | Soft (10-30 Hz) | Performance |
| Motion planning | Soft (1-10 Hz) | Performance |
| Task planning | Best-effort | Correctness > speed |

---

## Physical Constraints Specification {#constraints-spec}

A **physical constraints specification** documents all the physical factors that bound what a system can achieve.

### Specification Template

```markdown
# Physical Constraints Specification: [Task Name]

## Task Description
[What the robot must do]

## Timing Constraints

### Deadlines
| Constraint | Value | Type | Justification |
|------------|-------|------|---------------|
| [name] | [time] | [hard/soft/best-effort] | [why] |

### Latency Budget
| Component | Latency | Notes |
|-----------|---------|-------|
| [component] | [time] | [details] |

**Total loop latency**: [sum]
**Required response time**: [time]
**Margin**: [difference]

## Spatial Constraints

### Workspace
- Volume: [dimensions]
- Obstacles: [description]
- Access constraints: [description]

### Precision Requirements
| Dimension | Required | Achievable | Margin |
|-----------|----------|------------|--------|
| Position | [±X mm] | [±Y mm] | [±Z mm] |
| Orientation | [±X°] | [±Y°] | [±Z°] |

## Force/Load Constraints

| Constraint | Value | Justification |
|------------|-------|---------------|
| Maximum payload | [kg] | [why] |
| Maximum contact force | [N] | [safety/task] |
| Continuous load | [kg] | [thermal limits] |

## Environmental Constraints

| Factor | Requirement | Sensor/Actuator Impact |
|--------|-------------|------------------------|
| Temperature | [range] | [effects] |
| Lighting | [conditions] | [camera implications] |
| Noise | [level] | [audio sensor implications] |

## Uncertainty Budget

| Source | Magnitude | Propagates To |
|--------|-----------|---------------|
| [source] | [±value] | [affected quantities] |

**Total uncertainty at end-effector**: [±value]

## Safety Constraints

| Constraint | Limit | Mechanism |
|------------|-------|-----------|
| Maximum velocity | [m/s] | [how enforced] |
| Maximum force | [N] | [how enforced] |
| Emergency stop time | [ms] | [how achieved] |

## Critical Constraints

[List the 3-5 constraints most likely to cause system failure if violated]

1. [Constraint]: [Why critical]
2. [Constraint]: [Why critical]
3. [Constraint]: [Why critical]
```

### Example: Pick and Place Coffee Cup

```markdown
# Physical Constraints Specification: Coffee Cup Pick and Place

## Task Description
Pick a coffee cup from a table and place it in a dishwasher rack

## Timing Constraints

### Deadlines
| Constraint | Value | Type | Justification |
|------------|-------|------|---------------|
| Cup detection | 500 ms | Soft | User waiting |
| Grasp execution | N/A | Best-effort | Static scene |
| Collision avoidance | 100 ms | Hard | Safety near humans |

### Latency Budget
| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 33 ms | 30 fps RGB-D |
| Cup detection | 80 ms | Neural network |
| Grasp planning | 200 ms | GraspNet |
| Motion planning | 150 ms | MoveIt |
| Motor response | 100 ms | 7-DOF arm |

**Total loop latency**: 563 ms
**Required response time**: Not constrained (static cup)
**Margin**: Adequate for static task

## Spatial Constraints

### Workspace
- Volume: 0.8m × 0.6m × 0.4m (table surface)
- Obstacles: Other dishes, table edges
- Access: Approach from above only

### Precision Requirements
| Dimension | Required | Achievable | Margin |
|-----------|----------|------------|--------|
| Position | ±10 mm | ±5 mm | ±5 mm |
| Orientation | ±5° | ±2° | ±3° |

## Force/Load Constraints

| Constraint | Value | Justification |
|------------|-------|---------------|
| Maximum payload | 0.5 kg | Cup + liquid |
| Maximum grip force | 20 N | Don't crush cup |
| Minimum grip force | 5 N | Don't drop cup |

## Uncertainty Budget

| Source | Magnitude | Propagates To |
|--------|-----------|---------------|
| Depth camera | ±3 mm | Cup position |
| Hand-eye calibration | ±2 mm | Grasp pose |
| Joint tracking | ±2 mm | End-effector |

**Total uncertainty at end-effector**: ±4.1 mm

## Safety Constraints

| Constraint | Limit | Mechanism |
|------------|-------|-----------|
| Max velocity | 0.5 m/s | Software limit |
| Max force | 50 N | Current limiting |
| E-stop time | 100 ms | Motor STO |

## Critical Constraints

1. **Grip force**: Too high crushes cup, too low drops it
2. **Position accuracy**: Must be within ±10 mm for reliable grasp
3. **Collision avoidance**: Human may reach into workspace
```

---

## Case Study: Analyzing a Complete System {#case-study}

Let's analyze a warehouse mobile manipulator that picks items from shelves.

### System Components

- **Mobile base**: Differential drive, odometry + LIDAR localization
- **Arm**: 6-DOF collaborative arm
- **Gripper**: Parallel jaw
- **Sensors**: 2D LIDAR, RGB-D camera, joint encoders

### Perception-Action Loop

```
LIDAR (10 Hz) ──► Localization (50 Hz) ──► Navigation (10 Hz) ──► Base control (50 Hz)
                        │
Camera (30 Hz) ──► Object detection ──► Grasp planning ──► Arm control (100 Hz)
                        │
                        ▼
                  Task coordination
```

### Timing Analysis

**Navigation loop**: 100 ms LIDAR + 20 ms localization + 50 ms planning + 20 ms control = **190 ms**
- At 1 m/s, robot travels 19 cm between perception updates
- Acceptable for warehouse navigation

**Manipulation loop**: 33 ms camera + 100 ms detection + 200 ms grasp planning + 10 ms control = **343 ms**
- Object is stationary, so latency is not critical
- But human approaching at 1.5 m/s travels 51 cm in this time → safety concern

### Uncertainty Analysis

**Mobile base position**: ±5 cm (LIDAR SLAM) + ±2 cm (drift between updates) = **±5.4 cm**

**End-effector position**: ±5.4 cm (base) + ±3 cm (arm calibration) + ±2 cm (arm tracking) = **±6.5 cm**

**Grasp success**: With ±6.5 cm uncertainty and 10 cm gripper opening, grasp reliability is marginal. **Visual servoing needed** to close the loop with the camera before final grasp.

### Critical Constraints Identified

1. **Safety response time** (100 ms) may be insufficient if detection takes 343 ms
2. **Base position uncertainty** (±5.4 cm) limits manipulation without visual servoing
3. **Navigation update rate** (190 ms) limits speed in dynamic environments

---

## Summary

This chapter integrated sensors and actuators into complete system analysis:

1. **The perception-action loop** is the fundamental architecture; its total latency determines what tasks are feasible.

2. **Latency budget analysis** adds up all delays from sensor to actuator; this determines if a system can respond fast enough.

3. **Uncertainty propagates** through the entire loop; camera noise becomes grasp failure rate.

4. **Real-time requirements** are classified as hard (must meet), soft (should meet), or best-effort (no deadline).

5. **Physical constraints specifications** document all the timing, spatial, force, and uncertainty constraints that govern a system.

---

## Self-Assessment Questions

1. **Latency Budget**: A robot must catch objects dropped from 1 meter height (falling time ≈ 450 ms). Your system has: camera 16 ms, detection 40 ms, planning 30 ms, control 2 ms, motor response 150 ms. Is the system feasible? What is the margin?

2. **Uncertainty Propagation**: A depth camera has ±10 mm accuracy. The robot arm has ±3 mm repeatability. The gripper can grasp objects that are within ±8 mm of the target position. Will the system reliably grasp objects? What is the limiting factor?

3. **Real-Time Classification**: Classify each as hard, soft, or best-effort real-time: (a) motor current control, (b) updating a map for navigation, (c) sending status to a remote operator, (d) avoiding a moving obstacle.

4. **Constraints Specification**: Write a physical constraints specification for a robot that pours liquid from a bottle into a glass. Include timing, precision, and force constraints.

5. **System Analysis**: A self-driving car has sensor latency of 50 ms and total processing latency of 200 ms. At 60 mph (27 m/s), how far does the car travel during this latency? What implications does this have for stopping distance calculations?

---

## Module 1 Complete

Congratulations on completing Module 1: Physical AI Foundations!

You now have the mental model that will guide all subsequent learning:
- **Embodied intelligence** faces constraints that software-only AI does not
- **Sensors** provide noisy, limited windows into the physical world
- **Actuators** provide limited, imperfect means of affecting the world
- **Physical constraints** govern what any robotic system can achieve

## What's Next

In [Module 2: ROS 2 Fundamentals](/module-2), you'll learn to implement these concepts in code:
- Sensors become **topics** you subscribe to
- Actuators become **commands** you publish
- The perception-action loop becomes **nodes** connected by messages
- Timing constraints become **QoS policies** and **node design**

The physical grounding you've built in Module 1 will inform every design decision in Module 2 and beyond.

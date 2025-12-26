---
id: chapter-3-actuator-fundamentals
title: "Chapter 3: Actuator Fundamentals"
sidebar_label: "3. Actuator Fundamentals"
sidebar_position: 4
---

# Chapter 3: Actuator Fundamentals — How Robots Act

## Chapter Goal

By the end of this chapter, you will understand **how robots affect their environment through actuators**, including the capabilities, limitations, response characteristics, and safety constraints of common actuator types. You will be able to evaluate actuators for specific applications and predict how they will behave under load.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 3.1 | Classify actuators by type (electric, hydraulic, pneumatic) and explain tradeoffs |
| 3.2 | For a given actuator type, describe its response characteristics (speed, force, precision) |
| 3.3 | Given an actuator specification, determine if it meets requirements for a task |
| 3.4 | Identify safety constraints and failure modes for actuators in human environments |
| 3.5 | Match actuator types to application requirements with justification |

---

## The Problem: When Commands Don't Match Motion

A robot arm is programmed to move a cup from point A to point B. The software commands a smooth trajectory. But the actual motion is:
- Jerky at low speeds (motor cogging)
- Overshoots at high speeds (inertia)
- Varies with load (the empty cup moves differently than a full one)
- Drifts over time (gear backlash accumulates)

The robot receives perfect commands, but **actuators don't produce perfect motion**. Understanding actuator characteristics is essential for predicting what a robot can actually do.

---

## Actuator Taxonomy {#actuator-taxonomy}

Actuators convert energy into motion. The main categories are defined by their energy source.

### Electric Actuators

Convert electrical energy into rotational or linear motion.

**Types**:
- DC motors (brushed, brushless)
- Servo motors
- Stepper motors
- Linear motors

**Advantages**:
- Clean (no fluids)
- Precise control
- Quiet operation
- Easy integration with electronics

**Limitations**:
- Limited force/torque density
- Heat generation limits continuous operation
- Require gearboxes for high torque

### Hydraulic Actuators

Convert fluid pressure into motion.

**Types**:
- Hydraulic cylinders (linear)
- Hydraulic motors (rotary)

**Advantages**:
- Very high force/torque density
- Smooth motion
- Self-lubricating

**Limitations**:
- Heavy (pump, reservoir, lines)
- Potential for leaks
- Requires fluid management
- Slower response than electric

### Pneumatic Actuators

Convert compressed air into motion.

**Types**:
- Pneumatic cylinders
- Pneumatic muscles (McKibben actuators)

**Advantages**:
- High speed
- Naturally compliant
- Clean (just air)
- Inherently safe (compresses under overload)

**Limitations**:
- Low precision (air is compressible)
- Requires air supply
- Limited force compared to hydraulics

### Comparison Summary

| Property | Electric | Hydraulic | Pneumatic |
|----------|----------|-----------|-----------|
| **Force density** | Low-Medium | Very High | Medium |
| **Precision** | High | Medium | Low |
| **Speed** | High | Medium | Very High |
| **Efficiency** | High | Medium | Low |
| **Complexity** | Low | High | Medium |
| **Cost** | Low-Medium | High | Low |
| **Safety** | Medium | Low | High |

---

## Electric Motors {#electric-motors}

Electric motors are the most common actuators in robotics due to their precision, cleanliness, and ease of control.

### DC Motors (Brushed)

**Operating Principle**: Current through a coil in a magnetic field creates torque. Brushes commutate current as the rotor spins.

**Characteristics**:
- Simple, low cost
- Linear torque-speed relationship
- Brushes wear over time
- Electrical noise from commutation

**Applications**: Low-cost robots, hobbyist projects, simple mechanisms

### DC Motors (Brushless / BLDC)

**Operating Principle**: Electronic commutation replaces mechanical brushes. Sensors (Hall effect or encoders) determine rotor position.

**Characteristics**:
- Higher efficiency than brushed
- No brush wear, longer life
- Higher power density
- Requires electronic controller

**Applications**: Drones, high-performance robots, electric vehicles

### Servo Motors

**Operating Principle**: Motor with integrated feedback (encoder) and controller. Commands are position, velocity, or torque setpoints.

**Characteristics**:
- Precise position/velocity control
- Built-in feedback loop
- Higher cost than basic motors
- Excellent dynamic response

**Applications**: Robot arms, CNC machines, precision mechanisms

### Stepper Motors

**Operating Principle**: Discrete steps (typically 1.8° = 200 steps/revolution). Each electrical pulse advances one step.

**Characteristics**:
- Open-loop position control (count steps)
- Can miss steps under load
- Holding torque when stationary
- Resonance at certain speeds

**Applications**: 3D printers, simple positioning, low-cost automation

### Comparison

| Property | Brushed DC | Brushless DC | Servo | Stepper |
|----------|------------|--------------|-------|---------|
| **Cost** | Low | Medium | High | Low |
| **Precision** | Low | Medium | High | Medium* |
| **Efficiency** | Medium | High | High | Low |
| **Control complexity** | Low | Medium | High | Low |
| **Torque at low speed** | Low | Medium | High | High |
| **Maintenance** | Brush wear | None | None | None |

*Steppers have high precision if steps are not missed

---

## Torque-Speed Characteristics {#torque-speed}

Every motor has a fundamental tradeoff between torque and speed.

### The Torque-Speed Curve

```
Torque
  ▲
  │
τ_stall ├─────────────•
  │                  \
  │                   \
  │                    \
  │                     \
  │                      \
  │                       •──────────
  └───────────────────────┴──────────► Speed
                        ω_no-load
```

**Key Points**:
- **Stall torque (τ_stall)**: Maximum torque at zero speed
- **No-load speed (ω_no-load)**: Maximum speed with no load
- **Operating region**: The curve between these points

**Physical Meaning**:
- High torque requires low speed (or more current → more heat)
- High speed requires low torque
- You cannot have both simultaneously from a given motor

### Gearboxes

Gearboxes trade speed for torque:
- **Gear ratio N:1** means output speed is 1/N of input, torque is N× input
- Example: 100:1 gearbox with motor at 10,000 rpm, 0.1 Nm → output at 100 rpm, 10 Nm (minus losses)

**Gearbox Considerations**:

| Factor | Impact |
|--------|--------|
| **Efficiency** | Typically 70-95%; power is lost |
| **Backlash** | Play between gear teeth; limits precision |
| **Inertia** | Reflected inertia affects dynamics |
| **Cost** | Precision gearboxes are expensive |
| **Size/weight** | Adds to mechanism bulk |

**Gearbox Types**:

| Type | Efficiency | Backlash | Cost | Notes |
|------|------------|----------|------|-------|
| Spur | 90-95% | Medium | Low | Simple, noisy |
| Planetary | 90-97% | Low | Medium | Compact, efficient |
| Harmonic drive | 80-90% | Very low | High | Zero backlash option |
| Cycloidal | 85-95% | Very low | High | High ratio, compact |

---

## Actuator Response Characteristics {#response-characteristics}

Actuators don't respond instantly to commands. Understanding response characteristics is critical for control.

### Time Constants

**Electrical time constant (τ_e)**: How fast current can change
- Typically 0.1-10 ms
- Limits how fast torque can change

**Mechanical time constant (τ_m)**: How fast speed can change
- Depends on inertia and motor characteristics
- Typically 10-200 ms

### Bandwidth

**Bandwidth** is the frequency at which response amplitude drops to 70.7% (-3dB):
- Higher bandwidth = faster response
- Motors: 10-100 Hz typical
- Hydraulics: 1-20 Hz typical
- Pneumatics: 1-10 Hz typical

### Latency Sources

| Source | Typical Latency | Notes |
|--------|-----------------|-------|
| Communication | 0.1-10 ms | CAN bus, EtherCAT, etc. |
| Controller computation | 0.1-1 ms | Drive internal loop |
| Electrical response | 0.1-10 ms | Current rise time |
| Mechanical response | 10-200 ms | Inertia, compliance |

**Physical Grounding**: A robot arm with 50 ms mechanical time constant cannot respond faster than 50 ms, regardless of how fast the software sends commands. The perception-action loop is bounded by actuator response.

---

## Compliance and Impedance {#compliance}

**Stiffness** describes how much an actuator resists displacement under external force.

### Stiff Actuation

Traditional robot arms are very stiff:
- High gear ratios → high reflected inertia
- Position is controlled precisely
- External forces cause high internal stresses

**Problems with stiff actuation**:
- Dangerous in human contact (high forces)
- Cannot adapt to contact uncertainties
- Requires precise world models

### Compliant Actuation

Compliant actuators intentionally include flexibility:

**Series Elastic Actuators (SEA)**:
- Spring between motor and output
- Spring deflection measures force
- Lower bandwidth but inherent force sensing

**Variable Stiffness Actuators**:
- Stiffness can be adjusted
- High stiffness for speed, low stiffness for safety

### Impedance Control

Rather than controlling position directly, control the relationship between position error and force:

**F = K(x_desired - x_actual) + B(v_desired - v_actual)**

Where:
- K = stiffness (virtual spring)
- B = damping (virtual damper)

**Applications**:
- Safe human-robot interaction
- Assembly tasks with contact
- Walking robots (ground contact)

---

## Safety Constraints {#safety-constraints}

Actuators can cause serious harm. Safety must be considered in every design.

### Force and Energy Limits

Robots operating near humans must limit:
- **Maximum force**: Contact force in collision
- **Maximum energy**: Kinetic energy that could be transferred
- **Maximum pressure**: Force per unit area at contact

ISO 10218 and ISO/TS 15066 provide specific limits for collaborative robots.

### Failure Modes

| Failure | Cause | Consequence |
|---------|-------|-------------|
| **Runaway** | Controller failure | Uncontrolled motion |
| **Seizure** | Mechanical jam, overheating | Sudden stop, motor damage |
| **Overshoot** | Control instability | Position exceeds limits |
| **Unexpected motion** | Software bug, sensor error | Collision |

### Safety Mechanisms

| Mechanism | Function |
|-----------|----------|
| **Hardware limits** | Physical stops, limit switches |
| **Current limiting** | Bounds maximum force |
| **Velocity limiting** | Bounds maximum speed |
| **Emergency stop** | Immediate power cut |
| **Safe torque off (STO)** | Certified motor disable |
| **Reduced mode** | Lower limits near humans |

### Safety Design Principles

1. **Defense in depth**: Multiple independent safety layers
2. **Fail-safe defaults**: Power loss → safe state
3. **Monitoring**: Continuous safety checks
4. **Tested limits**: Safety limits verified, not assumed

---

## Actuator Selection Methodology {#actuator-selection}

When selecting actuators for a robotic application:

### Step 1: Define Motion Requirements

- **Type**: Rotary or linear?
- **Range**: How far must it move?
- **Speed**: How fast? (continuous and peak)
- **Force/Torque**: How much? (continuous and peak)
- **Precision**: How accurate?

### Step 2: Calculate Load Requirements

- **Static load**: Force/torque to hold position
- **Dynamic load**: Force/torque for acceleration
- **Duty cycle**: Continuous operation vs intermittent

### Step 3: Consider Environmental Constraints

- Operating temperature range
- Exposure to dust, water, chemicals
- Size and weight limits
- Noise limits
- Power availability

### Step 4: Evaluate Safety Requirements

- Human proximity?
- Failure consequences?
- Required safety certifications?

### Step 5: Match Actuators to Requirements

1. Select actuator type (electric, hydraulic, pneumatic)
2. Size motor for torque/force requirements
3. Select gearbox for speed/torque tradeoff
4. Verify thermal capacity for duty cycle
5. Check that response characteristics meet control needs

---

## Summary

This chapter covered how robots affect their environment through actuators:

1. **Actuator types** (electric, hydraulic, pneumatic) offer different tradeoffs in force, precision, speed, and complexity.

2. **Electric motors** are most common in robotics, with DC, BLDC, servo, and stepper types offering different characteristics.

3. **Torque-speed curves** define the fundamental tradeoff for any motor; gearboxes trade speed for torque.

4. **Response characteristics** (time constants, bandwidth, latency) limit how fast actuators can respond to commands.

5. **Compliance and impedance** are critical for safe human-robot interaction and contact tasks.

6. **Safety constraints** are non-negotiable when actuators operate near humans; multiple layers of protection are required.

---

## Self-Assessment Questions

1. **Actuator Classification**: A robot must lift 50 kg repeatedly in a factory. Rank electric, hydraulic, and pneumatic actuators by suitability. Justify your ranking.

2. **Torque-Speed Tradeoff**: A motor has stall torque of 1 Nm and no-load speed of 5000 rpm. You need 10 Nm at 50 rpm. What gear ratio is required? What are the tradeoffs?

3. **Response Characteristics**: A servo motor has a mechanical time constant of 100 ms. You need the arm to track a trajectory with features changing every 50 ms. Will this work? Why or why not?

4. **Safety Analysis**: A collaborative robot arm will hand objects to workers. List three safety mechanisms that should be implemented and explain why each is necessary.

5. **Actuator Selection**: You are designing a gripper for a robot that picks strawberries. What actuator type would you select? What force limits would you set? Justify your choices.

---

## What's Next

In [Chapter 4: Physical Constraints and the Perception-Action Loop](/module-1/chapter-4-physical-constraints), you'll integrate your knowledge of sensors and actuators into a complete system view, learning to analyze timing budgets, uncertainty propagation, and the constraints that govern real robotic systems.

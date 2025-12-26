---
id: chapter-1-simulation-fundamentals
title: "Chapter 1: Simulation Fundamentals"
sidebar_label: "1. Simulation Fundamentals"
sidebar_position: 2
---

# Chapter 1: Simulation Fundamentals and Physics Engines

## Chapter Goal

By the end of this chapter, you will be able to **explain what physics simulation does (and does not) model, compare physics engine characteristics, and establish Gazebo as the primary simulation environment** for robotics development.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 1.1 | Explain what a physics engine simulates (rigid body dynamics, collision, joints) and what it simplifies |
| 1.2 | Compare physics engines (ODE, Bullet, DART) and articulate tradeoffs for robotics |
| 1.3 | Install and verify Gazebo Harmonic with ros_gz bridge functioning |
| 1.4 | Explain the simulation loop and its relationship to real-time |

---

## The Promise and Peril of Simulation

Consider this scenario: You develop a robot navigation algorithm. In simulation, it achieves 98% success rate navigating a warehouse. You deploy to hardware and it fails 40% of the time.

What happened?

The algorithm learned behaviors that work only in simulation:
- Perfect sensor readings (no noise, no dropout)
- Instant motor response (no delay, no saturation)
- Idealized physics (perfect friction, no vibration)

**This is the reality gap.** Understanding it starts with understanding what physics engines actually do.

---

## What Physics Engines Simulate {#physics-engines}

A physics engine is software that approximates physical laws numerically.

### What Is Simulated Accurately

| Phenomenon | How It's Modeled | Accuracy |
|------------|------------------|----------|
| **Rigid body dynamics** | Newton-Euler equations | High for solid objects |
| **Collision detection** | Geometric intersection | High for convex shapes |
| **Contact forces** | Constraint-based or impulse | Moderate |
| **Joint constraints** | Lagrangian mechanics | High |
| **Gravity** | Constant acceleration | Exact |
| **Friction** | Coulomb model | Moderate |

### What Is Simplified or Ignored

| Phenomenon | Reality | Simulation |
|------------|---------|------------|
| **Deformable objects** | Complex material mechanics | Often rigid or simple spring |
| **Fluid dynamics** | Navier-Stokes equations | Usually ignored or very simplified |
| **High-frequency vibration** | Continuous phenomena | Discretized away by step size |
| **Thermal effects** | Temperature-dependent properties | Usually constant properties |
| **Wear and degradation** | Material changes over time | Static properties |
| **Electrical dynamics** | Motor winding inductance | Instantaneous torque |

**Physical Grounding**: When configuring simulated sensors and actuators (Chapters 3-4), you must understand these limitations. A simulated IMU won't have thermal drift. A simulated motor won't have electrical time constants. You can add approximations, but they're never complete.

---

## Physics Engine Comparison {#engine-comparison}

Gazebo supports multiple physics engines. Each has different strengths.

### ODE (Open Dynamics Engine)

**Default in Gazebo**

| Aspect | Characteristic |
|--------|----------------|
| Speed | Fast |
| Contact handling | Adequate for most robotics |
| Accuracy | Good for rigid bodies |
| Articulated bodies | Moderate |

**Use for**: General mobile robot simulation, initial development

### Bullet

**Better contact physics**

| Aspect | Characteristic |
|--------|----------------|
| Speed | Moderate |
| Contact handling | Better friction, more stable stacking |
| Accuracy | Good with tuning |
| Articulated bodies | Good |

**Use for**: Manipulation tasks, contact-rich scenarios

### DART (Dynamic Animation and Robotics Toolkit)

**Designed for robotics**

| Aspect | Characteristic |
|--------|----------------|
| Speed | Moderate |
| Contact handling | Good |
| Accuracy | High for articulated systems |
| Articulated bodies | Excellent (skeletal models) |

**Use for**: Humanoid robots, complex kinematic chains

### Decision Framework

```
What is your primary task?
├── Mobile robot navigation → ODE (default, fast)
├── Object manipulation → Bullet (better contacts)
└── Humanoid/legged robot → DART (articulated bodies)
```

---

## Gazebo Architecture {#gazebo-architecture}

Gazebo Harmonic (the current version) has three main components:

```
┌─────────────────────────────────────────────────────────────────┐
│                      Gazebo Architecture                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐                                           │
│  │   Gazebo Server  │ ◄── Physics simulation (headless)         │
│  │   (gz sim -s)    │     - Physics engine                      │
│  │                  │     - Sensor simulation                    │
│  │                  │     - Plugin execution                     │
│  └────────┬─────────┘                                           │
│           │                                                      │
│           │ Gazebo Transport                                     │
│           │                                                      │
│  ┌────────▼─────────┐                                           │
│  │    Gazebo GUI    │ ◄── Visualization                         │
│  │   (gz sim -g)    │     - 3D rendering                        │
│  │                  │     - Model inspection                     │
│  │                  │     - Interactive tools                    │
│  └──────────────────┘                                           │
│                                                                  │
│  ┌──────────────────┐                                           │
│  │   ros_gz_bridge  │ ◄── ROS 2 integration                     │
│  │                  │     - Topic bridging                       │
│  │                  │     - Service bridging                     │
│  │                  │     - Parameter passing                    │
│  └──────────────────┘                                           │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### ros_gz Bridge

The bridge connects Gazebo topics to ROS 2 topics:

| Gazebo Topic | ROS 2 Topic | Message Type |
|--------------|-------------|--------------|
| `/camera/image` | `/camera/image_raw` | `sensor_msgs/Image` |
| `/lidar/scan` | `/scan` | `sensor_msgs/LaserScan` |
| `/imu` | `/imu/data` | `sensor_msgs/Imu` |
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` |

**Key Point**: Your ROS 2 nodes don't know they're talking to simulation. They see standard topics with standard messages.

---

## The Simulation Loop {#simulation-loop}

Understanding the simulation loop is essential for interpreting results.

### Step-by-Step Execution

```
For each simulation step:
    1. Read sensor states from physics world
    2. Update sensor plugins (add noise, compute outputs)
    3. Publish sensor data via ros_gz bridge
    4. Receive commands via ros_gz bridge
    5. Apply actuator commands to physics joints
    6. Advance physics by dt (step size)
    7. Resolve collisions and contacts
    8. Update world state
```

### Timing Concepts

**Step size (dt)**: Time advanced per physics iteration
- Smaller dt → more accurate physics → slower simulation
- Larger dt → less accurate physics → faster simulation
- Typical: 1ms for precise work, 10ms for fast simulation

**Real-Time Factor (RTF)**: Ratio of simulation time to wall time
- RTF = 1.0: simulation runs at real-time speed
- RTF > 1.0: simulation runs faster than real-time
- RTF < 1.0: simulation runs slower than real-time

### Calculating Effective Control Rate

**Physical Grounding**: If your control algorithm expects 100 Hz updates:

| Step Size | RTF | Effective Rate | Meets Requirement? |
|-----------|-----|----------------|-------------------|
| 1ms | 1.0 | 1000 Hz | Yes |
| 10ms | 1.0 | 100 Hz | Exactly |
| 10ms | 0.5 | 50 Hz (wall) | No—algorithm runs 2x slower |
| 1ms | 2.0 | 2000 Hz (sim) | Yes, but results arrive faster than real-time |

When RTF < 1.0, your perception-action loop runs slower than real-time. This matters when testing timing-sensitive behaviors.

---

## Installation and Verification {#installation}

### Installing Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic

# Install ros_gz bridge
sudo apt install ros-humble-ros-gz
```

### Verification Steps

**1. Launch Gazebo with demo world:**

```bash
gz sim shapes.sdf
```

You should see a world with basic shapes (box, sphere, cylinder).

**2. Verify ros_gz bridge:**

Terminal 1:
```bash
gz sim shapes.sdf
```

Terminal 2:
```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

Terminal 3:
```bash
ros2 topic echo /clock
```

You should see clock messages flowing.

**3. Test a complete robot:**

```bash
gz sim -r gz_rover.sdf
```

This launches a rover robot you can control via Gazebo.

---

## The Fidelity-Speed Tradeoff {#tradeoff}

Every simulation decision involves trading fidelity for speed.

```
High Fidelity ◄──────────────────────────────► High Speed
     │                                              │
     ├── 0.1ms step size                           │
     ├── Bullet physics                            │
     ├── Full sensor noise models                  │
     ├── Mesh collision geometry                   │
     │                                              │
     │                                     10ms step size ──┤
     │                                     ODE physics ──────┤
     │                                     Minimal noise ────┤
     │                                     Primitive collision ┤
     │                                              │
     └── "Research quality"          "Development speed" ──┘
```

**Recommendation**: Start with faster settings for algorithm development, then validate with higher fidelity before deployment.

---

## What Simulation Cannot Tell You {#limitations}

Even perfect simulation cannot capture:

| Limitation | Why It Matters |
|------------|----------------|
| **Manufacturing variation** | Your specific robot differs from nominal |
| **Wear and degradation** | Real robots change over time |
| **Environmental variation** | Real floors aren't perfectly flat |
| **Sensor-specific artifacts** | Each sensor has unique quirks |
| **Electromagnetic interference** | Not modeled at all |
| **Thermal effects** | Components behave differently when hot |

**The fundamental truth**: Simulation is a model, not reality. Models are useful precisely because they simplify—but simplification means some truth is lost.

---

## Summary

This chapter established the foundation for physics simulation:

1. **Physics engines** simulate rigid body dynamics, collisions, and joints accurately, but simplify deformation, fluids, and high-frequency phenomena.

2. **Engine selection** depends on task: ODE for general use, Bullet for manipulation, DART for articulated robots.

3. **Gazebo architecture** separates physics (server), visualization (GUI), and ROS integration (bridge).

4. **The simulation loop** advances physics in discrete steps; step size and RTF determine effective timing.

5. **Fidelity vs speed** is a fundamental tradeoff—optimize for your development phase.

---

## Self-Assessment Questions

1. **Physics Limitation**: A robot grasps a foam ball in simulation and it works. On real hardware, the grip fails. What physics simplification most likely caused this?

2. **Engine Selection**: You're simulating a humanoid robot walking on rough terrain with many foot contacts. Which physics engine would you choose and why?

3. **Timing Calculation**: Your simulation runs at RTF=0.8 with 5ms step size. Your controller expects 200 Hz updates. Will it work correctly? What's the actual update rate?

4. **Architecture**: Your ROS 2 node receives camera images but they seem to come from the wrong angle. Where in the Gazebo architecture would you investigate?

5. **Tradeoff Decision**: You need to run 10,000 navigation tests overnight. Would you prioritize fidelity or speed? What settings would you adjust?

---

## What's Next

In [Chapter 2: Gazebo World Building](/module-3/chapter-2-gazebo-world-building), you'll learn to create simulation environments using SDF—the language that defines worlds, models, and their physics properties.

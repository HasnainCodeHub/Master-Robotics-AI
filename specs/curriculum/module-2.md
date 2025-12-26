# Module 2: ROS 2 Fundamentals

**Specification ID**: MOD-002
**Version**: 1.0.0
**Created**: 2024-12-24
**Authority**: curriculum-architect agent
**Status**: LOCKED (Phase 1 Complete)

---

## Module Goal

**Students completing this module will be able to design, implement, and deploy ROS 2 node architectures using Python (rclpy) that handle sensor data streams, command actuators, coordinate multi-component systems, and describe robots using URDF, all while respecting the physical constraints (latency, noise, timing) established in Module 1.**

This module transforms the conceptual understanding from Module 1 into executable code. Students will not just run tutorials; they will build production-ready patterns for robotic communication that they will use throughout the remaining modules and capstone.

---

## Prerequisites

### Required Before Starting Module 2

| Domain | Minimum Requirement | Verification Question | Module 1 Source |
|--------|---------------------|----------------------|-----------------|
| **Physical AI Mental Model** | Understand embodied intelligence and why physical constraints matter | Can you explain why a robot that works in simulation might fail in reality? | Chapter 1: Embodied Intelligence |
| **Sensor Knowledge** | Can categorize sensors and predict their noise/latency characteristics | Given an IMU specification, what noise characteristics would you expect? | Chapter 2: Sensor Fundamentals |
| **Actuator Knowledge** | Can categorize actuators and understand their response limitations | Why can't a motor instantly reach commanded velocity? | Chapter 3: Actuator Fundamentals |
| **Perception-Action Loop** | Can trace data flow from sensors through processing to actuators | Draw the timing diagram for a robot reacting to an obstacle | Chapter 4: Physical Constraints |
| **Python Programming** | Intermediate level: classes, async concepts, callbacks | Can you explain what a callback function is and when to use one? | Course Prerequisite |

### Explicit Module 1 Dependencies

The following Module 1 capabilities are REQUIRED and will be directly applied:

| M1 Capability | How It Applies in Module 2 |
|---------------|----------------------------|
| **M1-C1** (Embodied Intelligence) | Understanding why ROS 2's design addresses real-time constraints |
| **M1-C2** (Sensor Analysis) | Configuring sensor topics with appropriate QoS for noise/latency |
| **M1-C3** (Actuator Analysis) | Designing command topics with rate limits and safety bounds |
| **M1-C4** (Physical Constraints) | Setting QoS policies, timer rates, and timeout values |
| **M1-C5** (Perception-Action Loop) | Designing node graphs that implement the complete loop |

### NOT Required (Will Be Taught)

- ROS 2 installation and workspace setup (covered in Chapter 1)
- DDS and QoS concepts (covered in Chapter 1)
- Any simulation platform (Module 3)
- URDF authoring (covered in Chapter 5)
- tf2 transforms (covered in Chapter 5)

---

## Capabilities Gained

After completing this module, students CAN DO the following:

| ID | Capability | Verification Method |
|----|------------|---------------------|
| **M2-C1** | Design and implement a ROS 2 node architecture for a multi-component robotic system with explicit timing requirements | Given requirements, produce a node graph diagram and implement nodes that meet timing specifications |
| **M2-C2** | Implement publisher/subscriber patterns for sensor data streams with appropriate QoS settings based on physical sensor characteristics | Implement a node that subscribes to noisy sensor data with correct QoS (reliability, durability, deadline) |
| **M2-C3** | Implement service-based request/response patterns for discrete robot operations with timeout handling | Implement a service server and client for a gripper operation with explicit timeout and error handling |
| **M2-C4** | Implement action-based patterns for long-running robot tasks with progress feedback and cancellation support | Implement an action server for a navigation goal that reports progress and handles cancellation gracefully |
| **M2-C5** | Write and validate URDF descriptions for articulated robots with proper link/joint definitions and sensor attachments | Given a robot specification, produce a valid URDF that visualizes correctly in RViz2 |
| **M2-C6** | Build Python agents (rclpy) that coordinate multiple ROS 2 communication patterns to implement perception-action loops | Implement a behavior that uses topics for sensing, services for configuration, and actions for execution |

---

## Time Allocation

**Total Duration**: 3 weeks (21-24 hours of study)

| Chapter | Suggested Time | Notes |
|---------|---------------|-------|
| Chapter 1: ROS 2 Architecture & Physical Grounding | 4-5 hours | Environment setup + conceptual foundation |
| Chapter 2: Topics & Publishers/Subscribers | 4-5 hours | Core pattern; heavily exercised |
| Chapter 3: Services for Synchronous Operations | 3-4 hours | Simpler pattern; builds on topics |
| Chapter 4: Actions for Long-Running Tasks | 4-5 hours | Most complex pattern; requires mastery |
| Chapter 5: URDF & Robot Description | 3-4 hours | Declarative; different skill type |
| Chapter 6: Integration Patterns & Python Agents | 3-4 hours | Synthesis chapter; ties all patterns together |

**Pacing Guidance**:
- Week 1: Chapters 1-2 (Architecture + Topics)
- Week 2: Chapters 3-4 (Services + Actions)
- Week 3: Chapters 5-6 (URDF + Integration)
- Chapter 4 (Actions) is the most challenging; allow extra time if needed
- Chapter 6 should feel like consolidation, not new learning

---

## Chapter Breakdown

---

### Chapter 1: ROS 2 Architecture and Physical Grounding

**Chapter Goal**: Understand ROS 2's architecture as a solution to the physical communication challenges identified in Module 1, and establish a working development environment.

**Capabilities Addressed**: M2-C1 (foundation)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 1.1 | Explain why ROS 2's DDS-based architecture addresses real-time robotic communication needs | Essay: how does DDS solve the timing problems identified in M1? |
| 1.2 | Configure and verify a ROS 2 development environment with rclpy | Verification: run `ros2 doctor` with all checks passing |
| 1.3 | Explain QoS policies and map them to physical sensor/actuator requirements | Given sensor specs from M1, select appropriate QoS settings |
| 1.4 | Navigate the ROS 2 computation graph using command-line tools | Exercise: use `ros2 node`, `ros2 topic`, `ros2 service` to inspect a running system |

**New Concepts Introduced** (limit: 4-5):
1. ROS 2 architecture (nodes, executors, DDS middleware)
2. Quality of Service (QoS) policies and profiles
3. Workspaces, packages, and colcon build system
4. ROS 2 command-line introspection tools

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | QoS for sensor data: reliability (best-effort for high-rate cameras vs reliable for critical data), durability (transient local for late-joining subscribers), deadline (enforcing sensor update rates) |
| **Actuators** | QoS for commands: reliability (must commands be guaranteed?), lifespan (stale commands should be ignored), deadline (command timeout detection) |
| **Latency/Noise/Physics** | DDS as solution to M1's timing requirements; QoS deadline as implementation of latency budgets from M1 Chapter 4; executor models and their timing implications |

**Simulation/Conceptual Mapping**:
- Conceptual: mapping M1's perception-action loop to ROS 2 node graph
- Hands-on: installing ROS 2, creating workspace, building sample packages
- Prepares for Chapter 2: topics will implement sensor/actuator data flows

**Chapter Structure**:
1. Opening: Connecting M1's physical constraints to communication design
2. ROS 2 architecture overview: nodes, executors, DDS
3. QoS policies as physical constraint implementations
4. Development environment setup
5. Command-line tools for system introspection
6. Summary and self-assessment questions

---

### Chapter 2: Topics and Publishers/Subscribers - Sensor Data Streams

**Chapter Goal**: Master the publish/subscribe pattern for continuous data streams, with emphasis on sensor data handling and appropriate QoS configuration.

**Capabilities Addressed**: M2-C2 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 2.1 | Implement a publisher node that publishes sensor data at a specified rate with appropriate QoS | Code: IMU publisher with 100Hz rate and best-effort QoS |
| 2.2 | Implement a subscriber node that processes sensor data with callbacks and handles message timing | Code: subscriber that detects stale messages and logs timing violations |
| 2.3 | Select and use standard message types for common sensor data (Imu, Image, LaserScan, JointState) | Exercise: match sensor types from M1 Chapter 2 to ROS 2 message types |
| 2.4 | Configure QoS profiles based on sensor characteristics (rate, reliability requirements) | Design: QoS specification for a multi-sensor system |
| 2.5 | Debug topic communication issues using command-line tools and rqt | Exercise: diagnose and fix QoS mismatch between publisher and subscriber |

**New Concepts Introduced** (limit: 4-5):
1. Publisher/Subscriber pattern and topic naming conventions
2. Standard sensor message types (sensor_msgs, geometry_msgs)
3. Callback execution and threading models
4. Timer-based publishing patterns
5. Topic debugging and visualization (rqt_graph, rqt_topic)

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Core focus: publishing IMU data with noise characteristics from M1; publishing camera images with frame rate constraints; publishing LiDAR scans with appropriate message types; simulating sensor noise in published data |
| **Actuators** | Context: command velocity topics (cmd_vel) for mobile robots; joint command topics for manipulators; actuator state feedback topics |
| **Latency/Noise/Physics** | Timer rates matching physical sensor rates; QoS deadline for detecting sensor failures; message timestamping for latency measurement; handling out-of-order messages |

**Simulation/Conceptual Mapping**:
- Hands-on: implement simulated IMU publisher with Gaussian noise (connecting to M1 Chapter 2 noise models)
- Hands-on: implement cmd_vel subscriber that enforces rate limits (connecting to M1 Chapter 3 actuator constraints)
- Conceptual: QoS design exercise for warehouse robot scenario

**Chapter Structure**:
1. Opening: the perception-action loop as topic network
2. Publishers: creating, configuring, publishing
3. Subscribers: callbacks, message handling, timing
4. Standard message types for sensors and actuators
5. QoS configuration for physical requirements
6. Debugging topic communication
7. Summary and self-assessment questions

---

### Chapter 3: Services for Synchronous Operations

**Chapter Goal**: Master the service pattern for discrete request/response operations, with emphasis on configuration, calibration, and discrete actuator commands.

**Capabilities Addressed**: M2-C3 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 3.1 | Implement a service server that handles requests with proper error handling | Code: gripper service with open/close/status operations |
| 3.2 | Implement a service client with timeout handling and response validation | Code: client that calls gripper service with 5-second timeout and validates response |
| 3.3 | Design service interfaces for discrete robot operations using standard or custom types | Design: service definition for robot calibration operation |
| 3.4 | Choose between topics and services based on communication requirements | Decision exercise: given 5 scenarios, choose topic or service with justification |

**New Concepts Introduced** (limit: 3-4):
1. Service pattern: servers, clients, call semantics
2. Service interface definition (.srv files)
3. Synchronous vs asynchronous service calls
4. Timeout handling and error patterns

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Services for sensor configuration: set camera exposure, trigger calibration, request sensor status; services returning sensor readings on demand |
| **Actuators** | Core focus: discrete actuator operations (gripper open/close, arm home position, emergency stop); configuration services (set joint limits, enable/disable torque) |
| **Latency/Noise/Physics** | Timeout values based on physical operation duration (gripper close time from M1 actuator specs); error handling for actuator failures; blocking semantics and their impact on control loops |

**Simulation/Conceptual Mapping**:
- Hands-on: implement gripper service with realistic timing (2-3 seconds to close based on M1 actuator knowledge)
- Hands-on: implement calibration service that simulates sensor calibration routine
- Conceptual: service design for robot reset/initialization sequence

**Chapter Structure**:
1. Opening: when synchronous communication is appropriate
2. Service servers: handling requests, returning responses
3. Service clients: calling, waiting, handling errors
4. Defining custom service types
5. Topics vs services: decision framework
6. Summary and self-assessment questions

---

### Chapter 4: Actions for Long-Running Tasks

**Chapter Goal**: Master the action pattern for long-running tasks with feedback and cancellation, essential for navigation, manipulation, and any task that takes significant time.

**Capabilities Addressed**: M2-C4 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 4.1 | Implement an action server that executes long-running tasks with progress feedback | Code: navigation action server that reports distance remaining |
| 4.2 | Implement an action client that monitors progress and handles completion/failure/cancellation | Code: client that sends goal, displays progress, and can cancel |
| 4.3 | Design action interfaces with appropriate goal, result, and feedback messages | Design: action definition for pick-and-place operation |
| 4.4 | Implement cancellation handling that safely stops physical operations | Code: action server that handles cancel request by ramping down actuators |
| 4.5 | Choose between services and actions based on operation characteristics | Decision exercise: given 5 scenarios, choose service or action with justification |

**New Concepts Introduced** (limit: 5-6):
1. Action pattern: goal, result, feedback cycle
2. Action interface definition (.action files)
3. Action server lifecycle (goal acceptance, execution, completion)
4. Action client patterns (send goal, monitor feedback, cancel)
5. Preemption and cancellation handling
6. Goal state machine (pending, active, succeeded, canceled, aborted)

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Feedback using sensor data: position feedback from encoders, distance-to-goal from localization, obstacle proximity from LiDAR; sensor-driven goal completion detection |
| **Actuators** | Core focus: long-running actuator operations (robot navigation, arm trajectory execution, gripper manipulation sequence); safe cancellation requiring actuator ramping; actuator limits affecting execution time |
| **Latency/Noise/Physics** | Feedback rate based on control loop requirements; cancellation timeout based on actuator stopping distance; physical execution time estimation; handling sensor failures mid-action |

**Simulation/Conceptual Mapping**:
- Hands-on: implement navigation action with position feedback (simulated, using timer-based position updates)
- Hands-on: implement cancellation that simulates actuator deceleration
- Conceptual: action design for manipulation sequence with multiple waypoints

**Chapter Structure**:
1. Opening: why some operations cannot be services
2. Action concepts: goal, feedback, result
3. Action servers: implementing the execution loop
4. Action clients: goal management and monitoring
5. Cancellation: safe preemption of physical operations
6. Services vs actions: decision framework
7. Summary and self-assessment questions

---

### Chapter 5: URDF and Robot Description

**Chapter Goal**: Master URDF for describing robot kinematics, sensors, and visual/collision geometry, enabling robot visualization and preparing for simulation.

**Capabilities Addressed**: M2-C5 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 5.1 | Write URDF links with visual and collision geometry for robot components | Code: URDF link for robot base with mesh visual and box collision |
| 5.2 | Write URDF joints connecting links with appropriate joint types and limits | Code: URDF with revolute joint including position/velocity/effort limits |
| 5.3 | Attach sensor frames to URDF with correct transformations | Code: URDF with camera and IMU frames at correct positions |
| 5.4 | Validate URDF and visualize in RViz2 | Exercise: load URDF in RViz2, verify transforms, identify errors |
| 5.5 | Explain the relationship between URDF, joint_states, and tf2 | Essay: trace how joint_state_publisher updates tf2 tree from URDF |

**New Concepts Introduced** (limit: 4-5):
1. URDF structure: links, joints, materials
2. Geometry types: visual vs collision, primitives vs meshes
3. Joint types (revolute, prismatic, continuous, fixed) and limits
4. tf2 transform tree and frame relationships
5. URDF tools: xacro macros, validation, visualization

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Sensor frame placement in URDF (camera optical frame, IMU frame); sensor position affects data interpretation; connecting URDF frames to sensor topics |
| **Actuators** | Joint limits from M1 actuator specifications (position range, velocity limit, effort limit); joint dynamics (damping, friction); actuator-to-joint transmission |
| **Latency/Noise/Physics** | URDF as geometric truth vs physical reality; collision geometry for safety margins; mass/inertia properties for dynamics (preview for Module 3 simulation) |

**Simulation/Conceptual Mapping**:
- Hands-on: build URDF for 2-DOF arm with joint limits from M1 actuator specs
- Hands-on: add camera and IMU frames matching M1 sensor positions
- Conceptual: URDF design for mobile manipulator (base + arm)
- Prepares for Module 3: URDF will be used in Gazebo simulation

**Chapter Structure**:
1. Opening: why robots need formal descriptions
2. URDF basics: links, joints, structure
3. Visual and collision geometry
4. Joint types and limits from actuator specifications
5. Sensor frames and tf2 transforms
6. Validation and visualization in RViz2
7. Summary and self-assessment questions

---

### Chapter 6: Integration Patterns and Python Agents

**Chapter Goal**: Synthesize all ROS 2 communication patterns into coherent Python agents that implement complete perception-action loops.

**Capabilities Addressed**: M2-C6 (primary), M2-C1 through M2-C5 (integration)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 6.1 | Design a node architecture that uses topics, services, and actions appropriately | Diagram: node graph for mobile manipulator with all communication types labeled |
| 6.2 | Implement a behavior node that coordinates multiple communication patterns | Code: node that uses topics for sensing, services for configuration, actions for execution |
| 6.3 | Handle concurrent operations with proper callback management | Code: node that processes sensor topics while action is executing |
| 6.4 | Implement lifecycle patterns for robust node initialization and shutdown | Code: node with proper startup sequence and graceful shutdown |
| 6.5 | Create launch files that configure and start multi-node systems | Code: launch file for 4-node system with parameters and remapping |

**New Concepts Introduced** (limit: 4-5):
1. Multi-pattern integration in single node
2. Callback groups and executor threading
3. Lifecycle nodes (introduction, not full coverage)
4. Launch files and system composition
5. Parameter management and configuration

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Integration: subscribing to multiple sensor topics simultaneously; sensor data fusion patterns; handling sensor failures in integrated systems |
| **Actuators** | Integration: coordinating actuator commands across topics, services, and actions; actuator safety monitoring concurrent with operation |
| **Latency/Noise/Physics** | Core focus: timing analysis of complete perception-action loop implemented in ROS 2; callback scheduling and its effect on loop latency; meeting real-time requirements from M1 Chapter 4 in ROS 2 code |

**Simulation/Conceptual Mapping**:
- Hands-on: implement reactive behavior (obstacle avoidance) using topics only
- Hands-on: implement coordinated behavior (pick object) using topics + services + actions
- Conceptual: architecture design for complete mobile manipulator system
- Prepares for Module 3: these nodes will connect to simulated robots

**Chapter Structure**:
1. Opening: from patterns to complete systems
2. Multi-pattern node design
3. Callback management and threading
4. Lifecycle patterns for robustness
5. Launch files and system composition
6. Case study: integrating all patterns for a pick-and-place behavior
7. Summary and self-assessment questions

---

## Module Assessment Strategy

### Formative Assessments (During Module)

| Chapter | Assessment Type | Description |
|---------|-----------------|-------------|
| 1 | Environment verification | Successful ROS 2 setup with passing `ros2 doctor` |
| 2 | Code exercise | Implement sensor publisher/subscriber with QoS |
| 3 | Code exercise | Implement service server and client with timeout |
| 4 | Code exercise | Implement action server with feedback and cancellation |
| 5 | URDF creation | Build and validate robot URDF in RViz2 |
| 6 | Integration exercise | Implement multi-pattern behavior node |

### Summative Assessment (End of Module)

**Module 2 Integration Project**: Given a specification for a simple mobile robot (differential drive) with IMU, camera, and gripper, students must:

1. Design the ROS 2 node architecture (node graph diagram)
2. Implement sensor processing nodes (topics with appropriate QoS)
3. Implement gripper control (service-based)
4. Implement navigation behavior (action-based) with feedback
5. Create URDF for the robot with all sensors and joints
6. Create launch file that starts the complete system
7. Document timing analysis showing how design meets physical constraints

**Success Criteria**:
- Node architecture clearly separates concerns
- Topics use QoS settings justified by physical sensor characteristics
- Services include proper timeout and error handling
- Actions provide meaningful feedback and handle cancellation
- URDF validates without errors and visualizes correctly
- Launch file successfully starts all nodes
- Timing analysis references specific M1 physical constraints

---

## Transition to Module 3

### What This Module Establishes

Students leaving Module 2 understand:
- How to implement robotic communication using ROS 2
- When to use topics vs services vs actions
- How to configure QoS based on physical requirements
- How to describe robots using URDF
- How to build integrated Python agents

### What Module 3 Requires

Module 3 (Digital Twin and Simulation) builds directly on Module 2:
- **Sensor topics** become connections to simulated sensors
- **Command topics** become connections to simulated actuators
- **URDF** becomes the robot model spawned in simulation
- **Actions** become the interface for navigation and manipulation in simulation
- **Node architectures** run unchanged whether connected to simulation or (eventually) real hardware

### Explicit Bridge

The final section of Chapter 6 should preview:
- "In Module 3, you will connect these nodes to physics-simulated robots"
- "The sensor topics you subscribed to will carry data from simulated sensors"
- "The command topics you published will move simulated actuators"
- "The URDF you wrote will define the simulated robot's physical structure"
- "Your code will not change; only the source of sensor data and destination of commands changes"

---

## Cognitive Load Validation

### Per-Chapter Concept Count

| Chapter | New Concepts | Limit | Status |
|---------|--------------|-------|--------|
| Chapter 1 | 4 | 5 | PASS |
| Chapter 2 | 5 | 6 | PASS |
| Chapter 3 | 4 | 5 | PASS |
| Chapter 4 | 6 | 6 | PASS (at limit) |
| Chapter 5 | 5 | 6 | PASS |
| Chapter 6 | 5 | 6 | PASS |

### Physical Grounding Compliance

| Chapter | Sensors | Actuators | Latency/Noise/Physics | Status |
|---------|---------|-----------|----------------------|--------|
| Chapter 1 | QoS coverage | QoS coverage | Core focus (DDS, QoS) | PASS |
| Chapter 2 | Core focus | Context | Covered (timing, rates) | PASS |
| Chapter 3 | Context | Core focus | Covered (timeouts) | PASS |
| Chapter 4 | Feedback source | Core focus | Covered (cancellation) | PASS |
| Chapter 5 | Frame placement | Joint limits | Covered (geometry) | PASS |
| Chapter 6 | Integration | Integration | Core focus (loop timing) | PASS |

### Capability Progression

```
Chapter 1: M2-C1 foundation (architecture understanding)
    |
    v
Chapter 2: M2-C2 (topics/pub-sub for sensors)
    |
    v
Chapter 3: M2-C3 (services for discrete operations)
    |
    v
Chapter 4: M2-C4 (actions for long-running tasks)
    |
    v
Chapter 5: M2-C5 (URDF robot description)
    |
    v
Chapter 6: M2-C6 (integration) + M2-C1 complete (system architecture)
```

All capabilities are addressed with clear progression. No capability is introduced without foundation.

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2024-12-24 | curriculum-architect | Initial module specification |

---

## Notes for Content Authors

### Writing Constraints

1. **Python only (rclpy)**: No C++ examples; this course focuses on Python robotics development
2. **Production patterns**: Code examples should reflect real-world practices, not minimal tutorials
3. **Physical grounding is mandatory**: Every chapter must explicitly connect to M1 sensor/actuator/physics concepts
4. **Capability-first**: If content does not enable a stated capability, it does not belong
5. **Real-world examples**: Use realistic sensor rates, actuator timings, and QoS settings based on M1 specifications
6. **Do not exceed concept limits**: If a chapter needs more than 6 new concepts, it must be split

### Style Guidance

- Reference Module 1 explicitly when applying physical constraints
- Include code that demonstrates both correct and incorrect patterns
- Show QoS configuration with explicit justification from sensor/actuator characteristics
- Use consistent naming conventions across all examples
- Include numerical values (rates in Hz, timeouts in seconds, QoS deadlines in ms)
- End each chapter with exercises that verify capability, not just concept recall

### Code Quality Requirements

- All code examples must be complete and runnable
- Error handling must be included (not just happy path)
- Type hints should be used throughout
- Docstrings should explain physical grounding of design choices
- Code should follow ROS 2 Python style guidelines

---

**Module 2 Specification Complete**

**Next Steps**:
1. Content authors can begin chapter drafts using this specification
2. Each chapter draft must be validated against the physical grounding checklist
3. Assessment materials should align with stated learning objectives
4. Review gate: chapter drafts checked against capability claims before finalization

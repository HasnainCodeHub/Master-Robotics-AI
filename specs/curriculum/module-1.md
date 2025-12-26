# Module 1: Physical AI Foundations

**Specification ID**: MOD-001
**Version**: 1.0.0
**Created**: 2024-12-24
**Authority**: curriculum-architect agent
**Status**: LOCKED (Phase 1 Complete)

---

## Module Goal

**Students completing this module will be able to analyze any robotic system and identify its embodied intelligence characteristics: what sensors it uses to perceive, what actuators it uses to act, and what physical constraints (latency, noise, uncertainty) govern its behavior.**

This is the foundational mental model for the entire course. Without this grounding, students will treat robots as software systems that happen to have hardware attached, rather than as embodied agents where physics is a first-class concern.

---

## Prerequisites

### Required Before Starting Module 1

| Domain | Minimum Requirement | Verification Question |
|--------|---------------------|----------------------|
| **Python Programming** | Intermediate level: can write classes, debug code, use packages | Can you implement a class that reads from a file and processes data with error handling? |
| **Basic Robotics Concepts** | Know what sensors and actuators are conceptually | Can you name three types of sensors and explain what data each provides? |
| **Linear Algebra Basics** | Matrix multiplication, vectors, coordinate systems | Can you multiply two 3x3 matrices and explain what a coordinate frame represents? |
| **Scientific Thinking** | Understand measurement, error, and uncertainty concepts | Can you explain why a ruler with 1mm markings cannot measure to 0.1mm precision? |

### NOT Required (Will Be Taught)

- ROS 2 (covered in Module 2)
- Any simulation platform
- Control theory
- Specific sensor hardware experience
- Actuator design or motor control

---

## Capabilities Gained

After completing this module, students CAN DO the following:

| ID | Capability | Verification Method |
|----|------------|---------------------|
| **M1-C1** | Analyze a robotic system and articulate why embodied AI differs from disembodied AI | Given a robotics scenario, correctly identify which challenges are due to embodiment vs pure computation |
| **M1-C2** | Categorize sensors by modality and predict their noise characteristics, latency, and failure modes for a given application | Given sensor specifications, evaluate suitability and predict data quality issues |
| **M1-C3** | Categorize actuators by type and predict their response characteristics, force limits, and safety constraints | Given actuator specifications, determine suitability for manipulation or locomotion task |
| **M1-C4** | Identify the physical constraints (latency budgets, noise profiles, uncertainty bounds) that affect a given robotic task | Given a task description, enumerate constraints and their implications for system design |
| **M1-C5** | Trace the complete perception-action loop for a robotic system and identify bottlenecks | Given a system description, draw the loop and identify where timing or quality constraints are critical |

---

## Time Allocation

**Total Duration**: 2 weeks (14-16 hours of study)

| Chapter | Suggested Time | Notes |
|---------|---------------|-------|
| Chapter 1: Embodied Intelligence | 3-4 hours | Conceptual foundation; requires reflection |
| Chapter 2: Sensor Fundamentals | 4-5 hours | Dense content; multiple sensor types |
| Chapter 3: Actuator Fundamentals | 3-4 hours | Builds on sensor patterns |
| Chapter 4: Physical Constraints & Integration | 3-4 hours | Synthesis chapter; ties everything together |

**Pacing Guidance**:
- Weeks are soft boundaries; students should master concepts before proceeding
- Chapter 2 (Sensors) is the densest; allow extra time if needed
- Chapter 4 should feel like consolidation, not new learning

---

## Chapter Breakdown

---

### Chapter 1: Embodied Intelligence - Why Physical AI is Different

**Chapter Goal**: Establish the fundamental distinction between embodied and disembodied AI, and build the mental model that guides all subsequent learning.

**Capabilities Addressed**: M1-C1 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 1.1 | Explain why a robot that can play chess perfectly may fail at physically moving chess pieces | Essay or discussion: identify 3+ embodiment challenges |
| 1.2 | Identify the perception-action loop in a given robotic system and label its components | Diagram exercise: label sensor, processor, actuator, environment |
| 1.3 | Distinguish between computational time and physical time constraints | Given a scenario, identify which delays are computational vs physical |
| 1.4 | Articulate why simulation success does not guarantee real-world success | List 3+ reasons why sim-to-real transfer is non-trivial |

**New Concepts Introduced** (limit: 4-5):
1. Embodied intelligence vs disembodied AI
2. The perception-action loop
3. Physical time constraints (real-time requirements)
4. The reality gap (simulation vs physical world)

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Introduction to the concept that all perception starts with physical measurement; sensors as the robot's connection to reality |
| **Actuators** | Introduction to the concept that all action requires physical force/motion; actuators as the robot's means of affecting reality |
| **Latency/Noise/Physics** | Core focus: latency as physical constraint (speed of light, mechanical response); noise as measurement reality; physics as the "rules" that cannot be violated |

**Simulation/Conceptual Mapping**:
- Conceptual comparison: chess AI vs chess-playing robot arm
- Thought experiment: "What would fail if we deployed this AI in a body?"
- No simulation required; this is foundational conceptual content

**Chapter Structure**:
1. Opening scenario: AI that wins at simulation, fails at reality
2. Defining embodied intelligence
3. The perception-action loop model
4. Why physical time is different from computational time
5. The reality gap: what simulation cannot capture
6. Summary and self-assessment questions

---

### Chapter 2: Sensor Fundamentals - How Robots Perceive

**Chapter Goal**: Build a comprehensive understanding of how robots sense their environment, with emphasis on sensor limitations, noise characteristics, and failure modes.

**Capabilities Addressed**: M1-C2 (primary), M1-C4 (partial)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 2.1 | Classify sensors as proprioceptive or exteroceptive and explain the distinction | Categorization exercise: given 10 sensors, correctly classify each |
| 2.2 | For a given sensor type, describe its physical operating principle in one paragraph | Short answer: explain how a LiDAR measures distance |
| 2.3 | Given a sensor datasheet, extract and interpret noise specifications (accuracy, precision, resolution) | Datasheet analysis: identify noise floor, update rate, failure conditions |
| 2.4 | Predict how environmental conditions (lighting, temperature, occlusion) affect sensor performance | Scenario analysis: which sensor fails first in fog? In darkness? |
| 2.5 | Select appropriate sensors for a given perception task with justification | Design exercise: choose sensors for indoor navigation, justify each choice |

**New Concepts Introduced** (limit: 5-6):
1. Proprioceptive vs exteroceptive sensing
2. Sensor modalities (vision, ranging, inertial, force/torque, tactile)
3. Noise models (Gaussian noise, systematic bias, outliers)
4. Sensor fusion motivation (why single sensors are insufficient)
5. Failure modes and graceful degradation

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Core focus of chapter: cameras (resolution, frame rate, exposure), LiDAR (range, angular resolution, reflectivity dependence), IMU (drift, bias, vibration sensitivity), encoders (resolution, slip detection), force/torque (range, overload protection) |
| **Actuators** | Context: sensors that measure actuator state (encoders, current sensors); actuator motion affecting sensor readings (motion blur, vibration) |
| **Latency/Noise/Physics** | Sensor-specific latency (camera exposure time, LiDAR scan rate, IMU sampling), noise characterization (standard deviation, bias stability), physics constraints (speed of light for ToF, lens physics for cameras) |

**Simulation/Conceptual Mapping**:
- Conceptual: sensor datasheet analysis exercise (real datasheets)
- Conceptual: failure mode analysis for each sensor type
- Prepares for Module 3: these sensors will be simulated with noise models

**Chapter Structure**:
1. Opening scenario: robot that "sees" but doesn't perceive correctly
2. Sensor taxonomy: proprioceptive vs exteroceptive
3. Vision sensors: cameras, depth cameras, event cameras
4. Ranging sensors: LiDAR, ultrasonic, radar
5. Inertial sensors: IMU, gyroscopes, accelerometers
6. Force and tactile sensors
7. Noise, calibration, and failure modes
8. Sensor selection methodology
9. Summary and self-assessment questions

---

### Chapter 3: Actuator Fundamentals - How Robots Act

**Chapter Goal**: Build a comprehensive understanding of how robots affect their environment, with emphasis on actuator limitations, response characteristics, and safety constraints.

**Capabilities Addressed**: M1-C3 (primary), M1-C4 (partial)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 3.1 | Classify actuators by type (electric, hydraulic, pneumatic) and explain tradeoffs | Comparison table: create tradeoff matrix for three actuator types |
| 3.2 | For a given actuator type, describe its response characteristics (speed, force, precision) | Short answer: why are hydraulics used for heavy lifting despite complexity? |
| 3.3 | Given an actuator specification, determine if it meets requirements for a task | Specification analysis: can this motor move this load at this speed? |
| 3.4 | Identify safety constraints and failure modes for actuators in human environments | Safety analysis: what could go wrong with this actuator near humans? |
| 3.5 | Match actuator types to application requirements with justification | Design exercise: select actuators for mobile robot vs manipulator arm |

**New Concepts Introduced** (limit: 5-6):
1. Actuator types: electric motors, hydraulics, pneumatics, soft actuators
2. Torque-speed curves and power limitations
3. Gear reduction and transmission
4. Compliance and impedance (stiff vs compliant actuation)
5. Safety constraints (force limits, emergency stops, human interaction)

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Feedback sensors for actuator control: encoders (position), tachometers (velocity), current sensors (torque), limit switches (range of motion) |
| **Actuators** | Core focus of chapter: DC motors (brushed/brushless), servo motors, stepper motors, hydraulic cylinders, pneumatic actuators, series elastic actuators |
| **Latency/Noise/Physics** | Actuator-specific latency (motor electrical time constant, mechanical inertia, hydraulic fluid dynamics), backlash and dead zones, physical limits (stall torque, maximum velocity, thermal limits) |

**Simulation/Conceptual Mapping**:
- Conceptual: torque-speed curve analysis exercise
- Conceptual: actuator selection for manipulation vs locomotion
- Prepares for Module 3: these actuators will have models in simulation

**Chapter Structure**:
1. Opening scenario: robot that commands motion but moves differently
2. Actuator taxonomy: electric, hydraulic, pneumatic
3. Electric motors: DC, servo, stepper
4. Torque-speed characteristics and gearing
5. Hydraulic and pneumatic systems
6. Compliance and soft robotics introduction
7. Safety constraints and failure modes
8. Actuator selection methodology
9. Summary and self-assessment questions

---

### Chapter 4: Physical Constraints and the Perception-Action Loop

**Chapter Goal**: Synthesize sensor and actuator knowledge into a complete perception-action loop model, with emphasis on timing constraints, uncertainty propagation, and system-level thinking.

**Capabilities Addressed**: M1-C4 (primary), M1-C5 (primary), M1-C1 through M1-C3 (integration)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 4.1 | Draw and label the complete perception-action loop for a given robotic system | Diagram exercise: create loop diagram for autonomous car or manipulator |
| 4.2 | Calculate end-to-end latency budget for a perception-action loop | Numerical exercise: given component latencies, calculate total loop time |
| 4.3 | Trace how sensor uncertainty propagates through processing to action uncertainty | Analysis: if position uncertainty is X, what is end-effector uncertainty? |
| 4.4 | Identify timing constraints (hard real-time, soft real-time, best-effort) for system components | Classification: which parts of this system have hard deadlines? |
| 4.5 | Design a physical constraints specification for a given robotic task | Specification exercise: list all constraints for "pick up a coffee cup" |

**New Concepts Introduced** (limit: 4-5):
1. End-to-end latency and timing budgets
2. Uncertainty propagation through the perception-action loop
3. Real-time requirements taxonomy (hard, soft, best-effort)
4. Physical constraints specification methodology
5. System-level vs component-level thinking

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Integration: how sensor data flows into the perception-action loop; sensor fusion points; sensor rate as a constraint |
| **Actuators** | Integration: how commands flow from processing to actuators; actuator bandwidth as a constraint; control rate requirements |
| **Latency/Noise/Physics** | Core focus of chapter: total loop latency calculation, jitter and timing variability, uncertainty accumulation, physics constraints that bound what is achievable (reaction time, mechanical limits, thermal limits) |

**Simulation/Conceptual Mapping**:
- Conceptual: latency budget calculation exercise (numerical)
- Conceptual: uncertainty propagation analysis
- Synthesis: complete perception-action loop specification for a realistic system
- Prepares for Module 2: ROS 2 will implement these loops in code

**Chapter Structure**:
1. Opening scenario: system that works in pieces but fails when integrated
2. The complete perception-action loop
3. Latency budgets: from sensor to actuator
4. Uncertainty propagation: from measurement to action
5. Real-time requirements and deadlines
6. Physical constraints specification template
7. Case study: analyzing a complete robotic system
8. Summary and self-assessment questions

---

## Module Assessment Strategy

### Formative Assessments (During Module)

| Chapter | Assessment Type | Description |
|---------|-----------------|-------------|
| 1 | Reflection questions | Identify embodiment challenges in given scenarios |
| 2 | Datasheet analysis | Extract specifications from real sensor datasheets |
| 3 | Specification matching | Match actuator specs to task requirements |
| 4 | System analysis | Draw and annotate complete perception-action loop |

### Summative Assessment (End of Module)

**Module 1 Integration Exercise**: Given a robotic system description (e.g., warehouse mobile manipulator), students must:

1. Identify and classify all sensors and actuators
2. Predict noise characteristics and failure modes for each
3. Draw the complete perception-action loop with timing annotations
4. Specify the physical constraints that govern the system
5. Identify which constraints are most critical and why

**Success Criteria**:
- All sensors correctly classified with plausible noise characteristics
- All actuators correctly classified with plausible limitations
- Perception-action loop is complete and correctly structured
- Physical constraints specification covers latency, noise, and uncertainty
- Critical constraints are identified with sound justification

---

## Transition to Module 2

### What This Module Establishes

Students leaving Module 1 understand:
- Why physical constraints are first-class concerns in robotics
- How sensors and actuators define what a robot can perceive and do
- How to analyze a robotic system's physical constraints
- Why timing, noise, and uncertainty matter at the system level

### What Module 2 Requires

Module 2 (ROS 2 Fundamentals) builds directly on Module 1:
- **Sensor concepts** become sensor topics with message types
- **Actuator concepts** become command topics and action servers
- **Timing constraints** become QoS settings and node design
- **The perception-action loop** becomes the node graph architecture

### Explicit Bridge

The final section of Chapter 4 should preview:
- "In Module 2, you will implement these concepts in ROS 2"
- "The sensors you analyzed will become topics you subscribe to"
- "The actuators you specified will become commands you publish"
- "The timing constraints you calculated will become QoS requirements"

---

## Cognitive Load Validation

### Per-Chapter Concept Count

| Chapter | New Concepts | Limit | Status |
|---------|--------------|-------|--------|
| Chapter 1 | 4 | 5 | PASS |
| Chapter 2 | 5-6 | 6 | PASS (borderline) |
| Chapter 3 | 5-6 | 6 | PASS (borderline) |
| Chapter 4 | 4-5 | 6 | PASS |

### Physical Grounding Compliance

| Chapter | Sensors | Actuators | Latency/Noise/Physics | Status |
|---------|---------|-----------|----------------------|--------|
| Chapter 1 | Introduced | Introduced | Core focus | PASS |
| Chapter 2 | Core focus | Context | Covered | PASS |
| Chapter 3 | Context | Core focus | Covered | PASS |
| Chapter 4 | Integration | Integration | Core focus | PASS |

### Capability Progression

```
Chapter 1: M1-C1 (embodied intelligence understanding)
    |
    v
Chapter 2: M1-C2 (sensor analysis) + M1-C4 partial (physical constraints - sensors)
    |
    v
Chapter 3: M1-C3 (actuator analysis) + M1-C4 partial (physical constraints - actuators)
    |
    v
Chapter 4: M1-C4 complete (physical constraints synthesis) + M1-C5 (perception-action loop)
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

1. **Do not exceed concept limits**: If a chapter needs more than 6 new concepts, it must be split
2. **Physical grounding is mandatory**: Every chapter must explicitly address sensors, actuators, AND latency/noise/physics
3. **Capability-first**: If content does not enable a stated capability, it does not belong
4. **No ROS 2 yet**: This module is conceptual; ROS 2 examples belong in Module 2
5. **Real-world examples**: Use real sensor datasheets, real actuator specifications, real system case studies

### Style Guidance

- Use concrete examples over abstract descriptions
- Include numerical values where possible (noise in mm, latency in ms)
- Reference real robots and systems students may have seen
- Avoid jargon without definition; this is foundational content
- End each chapter with questions that verify capability, not just recall

---

**Module 1 Specification Complete**

**Next Steps**:
1. Content authors can begin chapter drafts using this specification
2. Each chapter draft must be validated against the physical grounding checklist
3. Assessment materials should align with stated learning objectives
4. Review gate: chapter drafts checked against capability claims before finalization

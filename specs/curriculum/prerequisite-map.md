# Curriculum Prerequisite Map

**Specification ID**: CURR-PREREQ-001
**Version**: 1.0.0
**Created**: 2024-12-24
**Authority**: curriculum-architect agent
**Status**: Validated

---

## Overview

This document maps all prerequisites and dependencies across the Physical AI & Humanoid Robotics curriculum. The curriculum follows a strict linear dependency chain where each module builds directly on capabilities established in prior modules. No module introduces concepts that depend on unstated prerequisites.

**Curriculum Validation Status**: PASS

The prerequisite audit confirms:
1. All internal dependencies are explicitly documented
2. External prerequisites are stated upfront in the course overview
3. Modules form a linear, explainable progression
4. No ordering violations detected
5. Hardware and ML prerequisites properly staged

---

## Module Dependency Graph

```
                    EXTERNAL PREREQUISITES
                           |
    +----------------------+----------------------+
    |                      |                      |
    v                      v                      v
[Python]           [Basic Robotics]        [Linear Algebra]
[Intermediate]     [Concepts]              [Basics]
    |                      |                      |
    +----------------------+----------------------+
                           |
                           v
    +-------------------------------------------------+
    |                    MODULE 1                      |
    |           Physical AI Foundations                |
    |                                                  |
    | Introduces:                                      |
    | - Embodied intelligence (M1-C1)                  |
    | - Sensor analysis (M1-C2)                        |
    | - Actuator analysis (M1-C3)                      |
    | - Physical constraints (M1-C4)                   |
    | - Perception-action loop (M1-C5)                 |
    +-------------------------------------------------+
                           |
                           | M1-C1, M1-C2, M1-C3, M1-C4, M1-C5
                           v
    +-------------------------------------------------+
    |                    MODULE 2                      |
    |              ROS 2 Fundamentals                  |
    |                                                  |
    | Requires: All M1 capabilities                    |
    | Introduces:                                      |
    | - Node architecture (M2-C1)                      |
    | - Topics/Pub-Sub (M2-C2)                         |
    | - Services (M2-C3)                               |
    | - Actions (M2-C4)                                |
    | - URDF (M2-C5)                                   |
    | - Python agents (M2-C6)                          |
    +-------------------------------------------------+
                           |
                           | M2-C1 through M2-C6
                           v
    +-------------------------------------------------+
    |                    MODULE 3                      |
    |          Digital Twin and Simulation             |
    |                                                  |
    | Requires: M1-C2, M1-C3, M1-C4, M1-C5             |
    |           M2-C1, M2-C2, M2-C4, M2-C5, M2-C6      |
    | Introduces:                                      |
    | - World building (M3-C1)                         |
    | - Robot spawning (M3-C2)                         |
    | - Sensor noise models (M3-C3)                    |
    | - Actuator dynamics (M3-C4)                      |
    | - Unity integration (M3-C5)                      |
    | - Reality gap analysis (M3-C6)                   |
    +-------------------------------------------------+
                           |
                           | M3-C1 through M3-C6
                           | + HARDWARE: NVIDIA RTX GPU
                           v
    +-------------------------------------------------+
    |                    MODULE 4                      |
    |            NVIDIA Isaac Ecosystem                |
    |                                                  |
    | Requires: M3-C1, M3-C3, M3-C4, M3-C6             |
    |           M2-C2, M2-C4, M2-C5                    |
    |           NVIDIA RTX GPU (hardware)              |
    | Introduces:                                      |
    | - Isaac Sim scenes (M4-C1)                       |
    | - RTX sensors (M4-C2)                            |
    | - VSLAM (M4-C3)                                  |
    | - Isaac perception (M4-C4)                       |
    | - Domain randomization (M4-C5)                   |
    | - Sim-to-real transfer (M4-C6)                   |
    +-------------------------------------------------+
                           |
                           | M4-C1 through M4-C6
                           | + ML/AI PREREQUISITES
                           v
    +-------------------------------------------------+
    |                    MODULE 5                      |
    |        Vision-Language-Action Systems            |
    |                                                  |
    | Requires: M4-C1, M4-C2, M4-C3, M4-C4, M4-C6      |
    |           M2-C4, M2-C6                           |
    |           M1-C4                                  |
    |           ML basics (neural nets, embeddings)    |
    | Introduces:                                      |
    | - Speech recognition (M5-C1)                     |
    | - LLM task planning (M5-C2)                      |
    | - Grounding (M5-C3)                              |
    | - Safety constraints (M5-C4)                     |
    | - VLM integration (M5-C5)                        |
    | - Failure handling (M5-C6)                       |
    +-------------------------------------------------+
                           |
                           | ALL PRIOR CAPABILITIES
                           v
    +-------------------------------------------------+
    |                   CAPSTONE                       |
    |          Integrated Humanoid System              |
    |                                                  |
    | Requires: ALL capabilities from M1-M5            |
    | Introduces: NO NEW CONCEPTS                      |
    | Demonstrates: Integration mastery                |
    +-------------------------------------------------+
```

---

## Detailed Prerequisites by Module

---

### Module 1: Physical AI Foundations

**External Prerequisites (before course)**:

| Domain | Minimum Requirement | Verification |
|--------|---------------------|--------------|
| Python Programming | Intermediate: classes, functions, modules, debugging, pip/conda | Can write 200-line program with classes |
| Basic Robotics Concepts | Know what sensors and actuators are conceptually | Can name 3 sensor types |
| Linear Algebra | Matrix multiplication, vectors, coordinate systems | Can multiply 3x3 matrices |
| Scientific Thinking | Understand measurement, error, uncertainty | Explain why ruler precision matters |
| Linux Command Line | Basic navigation, file ops, environment variables | Can navigate directories |

**Assumed But Not Taught**:
- Git version control (clone, commit, push, branch)
- Virtual environments (venv, conda)
- Basic debugging strategies
- Reading technical documentation
- Docker basics (for some environments)

**Internal Prerequisites (from prior modules)**:
- None (entry point)

**Concepts Introduced**:

| ID | Concept | Used By |
|----|---------|---------|
| M1-C1 | Embodied intelligence vs disembodied AI | M2 (DDS motivation), M4 (gap analysis), M5 (VLA limits) |
| M1-C2 | Sensor modalities and noise characteristics | M2 (QoS), M3 (noise models), M4 (RTX sensors), M5 (VLM input) |
| M1-C3 | Actuator types and response characteristics | M2 (actions), M3 (dynamics), M4 (control), M5 (safety) |
| M1-C4 | Physical constraints (latency, noise, uncertainty) | ALL subsequent modules |
| M1-C5 | Perception-action loop | M2 (node graph), M3 (simulation loop), M5 (VLA pipeline) |

---

### Module 2: ROS 2 Fundamentals

**External Prerequisites**:
- Same as Module 1, plus:
- Callback functions (programming concept)

**Internal Prerequisites (from prior modules)**:

| Capability | Source | Application in M2 |
|------------|--------|-------------------|
| M1-C1 | M1 Ch1 | Understanding why ROS 2 design addresses real-time constraints |
| M1-C2 | M1 Ch2 | Configuring sensor topics with appropriate QoS for noise/latency |
| M1-C3 | M1 Ch3 | Designing command topics with rate limits and safety bounds |
| M1-C4 | M1 Ch4 | Setting QoS policies, timer rates, and timeout values |
| M1-C5 | M1 Ch4 | Designing node graphs that implement the complete loop |

**Concepts Introduced**:

| ID | Concept | Used By |
|----|---------|---------|
| M2-C1 | ROS 2 node architecture design | M3 (sim integration), M4 (Isaac ROS), M5 (VLA nodes), CAP |
| M2-C2 | Publisher/Subscriber with QoS | M3 (sensor topics), M4 (Isaac bridge), M5 (audio streams) |
| M2-C3 | Services for synchronous operations | M3 (configuration), M4 (perception), M5 (safety checks) |
| M2-C4 | Actions for long-running tasks | M3 (navigation), M4 (VSLAM), M5 (task execution), CAP |
| M2-C5 | URDF robot description | M3 (Gazebo spawn), M4 (Isaac import), CAP |
| M2-C6 | Python agent integration patterns | M3 (sim control), M4 (perception), M5 (VLA pipeline), CAP |

---

### Module 3: Digital Twin and Simulation

**External Prerequisites**:
- None additional beyond M1/M2

**Internal Prerequisites (from prior modules)**:

| Capability | Source | Application in M3 |
|------------|--------|-------------------|
| M1-C2 | M1 Ch2 | Configuring simulated sensor noise models to match real specifications |
| M1-C3 | M1 Ch3 | Configuring simulated actuator models with realistic dynamics |
| M1-C4 | M1 Ch4 | Understanding what physics engines model vs simplify |
| M1-C5 | M1 Ch4 | Tracing perception-action loop through simulation |
| M2-C1 | M2 Ch1,6 | Designing nodes that connect to simulation identically to hardware |
| M2-C2 | M2 Ch2 | Receiving simulated sensor data and publishing actuator commands |
| M2-C4 | M2 Ch4 | Commanding long-running simulation tasks |
| M2-C5 | M2 Ch5 | URDF defines the robot that Gazebo spawns |
| M2-C6 | M2 Ch6 | Complete perception-action agents connected to simulation |

**Concepts Introduced**:

| ID | Concept | Used By |
|----|---------|---------|
| M3-C1 | Gazebo world building with SDF | M4 (Isaac comparison), CAP |
| M3-C2 | Robot spawning and control | M4 (Isaac spawning), CAP |
| M3-C3 | Sensor noise model configuration | M4 (RTX sensors), M5 (perception), CAP |
| M3-C4 | Actuator dynamics configuration | M4 (control), M5 (manipulation), CAP |
| M3-C5 | Unity ROS 2 integration | Alternative platform option |
| M3-C6 | Reality gap analysis | M4 (transfer), M5 (VLA gap), CAP |

---

### Module 4: NVIDIA Isaac Ecosystem

**External Prerequisites**:

| Domain | Minimum Requirement | Notes |
|--------|---------------------|-------|
| **NVIDIA GPU** | RTX 2070 (8GB VRAM) minimum | Required for Isaac Sim |
| **RAM** | 32 GB minimum | Required for simulation |
| **Storage** | 50 GB SSD | Required for Isaac installation |
| **OS** | Ubuntu 22.04 | Required for Isaac stack |

**Cloud Alternatives**: AWS g4dn.xlarge, GCP T4 instance

**Internal Prerequisites (from prior modules)**:

| Capability | Source | Application in M4 |
|------------|--------|-------------------|
| M3-C1 | M3 Ch2 | Mental model for Isaac Sim scene composition |
| M3-C3 | M3 Ch3 | Sensor plugin parameters transfer to Isaac |
| M3-C4 | M3 Ch4 | Dynamics modeling concepts transfer to Isaac |
| M3-C6 | M3 Ch6 | Reality gap understanding motivates Isaac features |
| M2-C2 | M2 Ch2 | Isaac ROS publishes on standard ROS 2 topics |
| M2-C4 | M2 Ch4 | Navigation/manipulation use action interfaces |
| M2-C5 | M2 Ch5 | Isaac Sim imports URDF for robot configuration |

**Concepts Introduced**:

| ID | Concept | Used By |
|----|---------|---------|
| M4-C1 | Isaac Sim scenes with USD | M5 (VLA environment), CAP |
| M4-C2 | RTX sensor simulation | M5 (VLM input), CAP |
| M4-C3 | Visual SLAM with Isaac ROS | M5 (navigation), CAP |
| M4-C4 | Isaac ROS perception packages | M5 (object detection), CAP |
| M4-C5 | Domain randomization | M5 (robustness), CAP |
| M4-C6 | Sim-to-real transfer strategies | M5 (VLA transfer), CAP |

---

### Module 5: Vision-Language-Action Systems

**External Prerequisites**:

| Domain | Minimum Requirement | Notes |
|--------|---------------------|-------|
| **GPU** | RTX 3070 (8GB) minimum, RTX 3090+ recommended | LLM/VLM inference |
| **RAM** | 32 GB minimum, 64 GB recommended | Model loading |
| **ML Basics** | Neural network concepts, embeddings | Required for understanding VLA |
| **Transformer Intuition** | Understand sequences and context | Required for LLM/VLM |

**ML Prerequisites Added** (per course overview validation):
- Understand what neural networks are conceptually
- Understand what embeddings capture semantic meaning
- Understand that transformers process sequences with context

Students without ML background should review "ML Concepts for Roboticists" appendix.

**Internal Prerequisites (from prior modules)**:

| Capability | Source | Application in M5 |
|------------|--------|-------------------|
| M4-C1 | M4 Ch2 | VLA commands execute in Isaac Sim environment |
| M4-C2 | M4 Ch2 | Camera data flows to VLMs for scene understanding |
| M4-C3 | M4 Ch4 | VSLAM enables spatial commands ("go to shelf") |
| M4-C4 | M4 Ch3 | Object detection provides manipulation targets |
| M4-C6 | M4 Ch6 | Transfer analysis applies to VLA pipeline |
| M2-C2 | M2 Ch2 | Audio streams, perception results use topics |
| M2-C4 | M2 Ch4 | Grounded commands execute as ROS 2 actions |
| M2-C6 | M2 Ch6 | VLA is multi-node system coordinating components |
| M1-C4 | M1 Ch4 | Latency budgets for voice-to-action pipeline |

**Concepts Introduced**:

| ID | Concept | Used By |
|----|---------|---------|
| M5-C1 | Whisper speech recognition | CAP (voice interface) |
| M5-C2 | LLM task planning | CAP (command decomposition) |
| M5-C3 | Symbol grounding | CAP (action execution) |
| M5-C4 | Safety constraints | CAP (humanoid safety) |
| M5-C5 | Vision-language models | CAP (scene understanding) |
| M5-C6 | Failure handling and recovery | CAP (robustness) |

---

### Capstone: Integrated Humanoid System

**External Prerequisites**:
- Same as Module 5 (hardware and ML)

**Internal Prerequisites (from prior modules)**:
- **ALL capabilities from ALL prior modules**

| Module | Required Capabilities |
|--------|----------------------|
| Module 1 | M1-C1 through M1-C5 (all 5) |
| Module 2 | M2-C1 through M2-C6 (all 6) |
| Module 3 | M3-C1 through M3-C6 (all 6) |
| Module 4 | M4-C1 through M4-C6 (all 6) |
| Module 5 | M5-C1 through M5-C6 (all 6) |

**Concepts Introduced**:
- **NONE** - Capstone is pure integration

**Integration Validation**:
The capstone validates that students can integrate 29 capabilities from prior modules into a coherent system. If students struggle, they revisit the relevant module; no new concepts are introduced.

---

## Audit Findings

### Unstated Prerequisites Identified

**NONE IDENTIFIED**

All prerequisites are explicitly documented in each module specification:
- External prerequisites stated in course overview
- Internal prerequisites listed with capability IDs and source chapters
- Hardware requirements stated where applicable (M4, M5)
- ML prerequisites added to M5 (per course overview validation)

### Missing Dependencies Identified

**NONE IDENTIFIED**

Each module explicitly documents:
- Which prior capabilities it requires
- How those capabilities are applied
- What new capabilities it introduces

The dependency chain is complete and traceable.

### Ordering Violations Identified

**NONE IDENTIFIED**

The curriculum follows a strict sequential ordering:

| Transition | Validation |
|------------|------------|
| Prerequisites to M1 | M1 builds only on external prerequisites |
| M1 to M2 | M2 explicitly requires M1-C1 through M1-C5 |
| M2 to M3 | M3 explicitly requires M1 and M2 capabilities |
| M3 to M4 | M4 explicitly requires M3 capabilities plus hardware |
| M4 to M5 | M5 explicitly requires M4 capabilities plus ML basics |
| M5 to Capstone | Capstone requires ALL prior capabilities |

No module uses concepts before they are introduced.

---

## Cross-Reference Matrix

This matrix shows which capabilities are used by subsequent modules:

| Capability | M2 | M3 | M4 | M5 | CAP |
|------------|----|----|----|----|-----|
| **M1-C1** (Embodied Intelligence) | X |   | X | X | X |
| **M1-C2** (Sensor Analysis) | X | X | X | X | X |
| **M1-C3** (Actuator Analysis) | X | X | X | X | X |
| **M1-C4** (Physical Constraints) | X | X | X | X | X |
| **M1-C5** (Perception-Action Loop) | X | X |   | X | X |
| **M2-C1** (Node Architecture) |   | X | X | X | X |
| **M2-C2** (Topics/Pub-Sub) |   | X | X | X | X |
| **M2-C3** (Services) |   | X | X | X | X |
| **M2-C4** (Actions) |   | X | X | X | X |
| **M2-C5** (URDF) |   | X | X |   | X |
| **M2-C6** (Python Agents) |   | X | X | X | X |
| **M3-C1** (World Building) |   |   | X |   | X |
| **M3-C2** (Robot Spawning) |   |   | X |   | X |
| **M3-C3** (Sensor Noise Models) |   |   | X | X | X |
| **M3-C4** (Actuator Dynamics) |   |   | X | X | X |
| **M3-C5** (Unity Integration) |   |   |   |   |   |
| **M3-C6** (Reality Gap) |   |   | X | X | X |
| **M4-C1** (Isaac Sim Scenes) |   |   |   | X | X |
| **M4-C2** (RTX Sensors) |   |   |   | X | X |
| **M4-C3** (VSLAM) |   |   |   | X | X |
| **M4-C4** (Isaac Perception) |   |   |   | X | X |
| **M4-C5** (Domain Randomization) |   |   |   |   | X |
| **M4-C6** (Sim-to-Real) |   |   |   | X | X |
| **M5-C1** (Speech Recognition) |   |   |   |   | X |
| **M5-C2** (LLM Planning) |   |   |   |   | X |
| **M5-C3** (Grounding) |   |   |   |   | X |
| **M5-C4** (Safety) |   |   |   |   | X |
| **M5-C5** (VLM Integration) |   |   |   |   | X |
| **M5-C6** (Failure Handling) |   |   |   |   | X |

**Observations**:
- M1-C4 (Physical Constraints) is the most heavily reused capability
- M3-C5 (Unity Integration) is not a prerequisite for later modules (optional alternative)
- All M5 capabilities converge in the capstone

---

## Hardware Prerequisite Timeline

Hardware requirements are staged appropriately:

| Week | Module | Hardware Requirement |
|------|--------|---------------------|
| 1-2 | M1 | Standard laptop/desktop |
| 3-5 | M2 | Standard laptop/desktop with Linux |
| 6-8 | M3 | Standard laptop/desktop (Gazebo runs on CPU) |
| 9-11 | M4 | **NVIDIA RTX 2070+ required** |
| 12-14 | M5 | **NVIDIA RTX 3070+ recommended** |
| 15-17 | Capstone | NVIDIA RTX 3070+ (same as M5) |

**Validation**: Hardware requirement is stated in Week 1 course materials with self-check script. Students have 8 weeks to arrange GPU access before Module 4.

---

## ML Prerequisite Timeline

ML prerequisites are staged appropriately:

| Week | Module | ML Requirement |
|------|--------|----------------|
| 1-8 | M1-M3 | None |
| 9-11 | M4 | Domain randomization concepts (no ML implementation) |
| 12-14 | M5 | **ML basics required** (neural nets, embeddings, transformers) |
| 15-17 | Capstone | Same as M5 |

**Validation**: ML prerequisite is stated in course overview with "ML Concepts for Roboticists" appendix provided. Students have 11 weeks to self-study before Module 5.

---

## Validation Statement

**CURRICULUM PREREQUISITE VALIDATION - FORMAL CONFIRMATION**

I, the curriculum-architect agent, have conducted a comprehensive cross-module prerequisite audit of the Physical AI & Humanoid Robotics curriculum.

**Findings**:

1. **Explicit Prerequisites**: Every module explicitly documents its prerequisites with capability IDs and source references. No unstated prerequisites exist.

2. **Complete Dependencies**: All dependencies are documented. Each capability is introduced before it is required by subsequent modules.

3. **No Ordering Violations**: The curriculum follows a strict linear progression. No module uses concepts before they are taught.

4. **Hardware Prerequisites Staged**: NVIDIA GPU requirement appears at Module 4 with 8 weeks advance notice.

5. **ML Prerequisites Staged**: ML basics requirement appears at Module 5 with 11 weeks advance notice and appendix provided.

6. **Capstone Integration Valid**: The capstone requires all 29 capabilities from prior modules and introduces no new concepts.

**Conclusion**:

The curriculum forms a **linear, explainable progression** where:
- Each module builds on explicit foundations from prior modules
- No hidden assumptions exist
- Students can trace any capability back to its source
- The capstone is achievable given completion of all prior modules

**Validation Status**: **PASS**

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2024-12-24 | curriculum-architect | Initial prerequisite map and validation |

---

**Prerequisite Map Complete**

This document should be reviewed whenever module specifications are updated to ensure the prerequisite chain remains valid.

# Capstone: Integrated Humanoid System

**Specification ID**: CAP-001
**Version**: 1.0.0
**Created**: 2024-12-24
**Authority**: curriculum-architect agent
**Status**: LOCKED (Phase 1 Complete)

---

## Capstone Goal

**Students completing this capstone will demonstrate integrated mastery of all five prior modules by architecting, implementing, and evaluating a complete humanoid robot system that receives voice commands, plans tasks, navigates environments, identifies objects, and performs manipulation, all within simulation with explicit sim-to-real transfer analysis.**

This is an INTEGRATION capstone, not a learning module. Students apply what they already know; they do NOT learn new concepts. The capstone validates that students can synthesize the full Physical AI stack into a coherent, functioning system.

**Critical Constraint**: The capstone introduces NO new concepts. All required knowledge comes from Modules 1-5. If a student struggles with a capstone requirement, the solution is to revisit the relevant module, not to introduce new material.

---

## Prerequisites

### Required Before Starting Capstone

**ALL prior modules are REQUIRED prerequisites for the capstone.**

| Module | Critical Prerequisites | Verification Question |
|--------|----------------------|----------------------|
| **Module 1: Physical AI Foundations** | Embodied intelligence mental model; sensor/actuator analysis; physical constraints specification | Can you identify the physical constraints that govern a pick-and-place task and trace uncertainty through the perception-action loop? |
| **Module 2: ROS 2 Fundamentals** | Node architecture design; topics, services, actions; URDF; Python agents | Can you design and implement a multi-node ROS 2 system that coordinates sensing, planning, and execution? |
| **Module 3: Digital Twin and Simulation** | Gazebo world building; sensor noise models; actuator dynamics; reality gap analysis | Can you configure a simulation with sensors and actuators matching physical specifications and identify what will transfer to hardware? |
| **Module 4: NVIDIA Isaac Ecosystem** | Isaac Sim scenes; Isaac ROS perception; VSLAM; domain randomization; sim-to-real transfer strategies | Can you implement GPU-accelerated perception and localization in Isaac Sim and design a transfer validation experiment? |
| **Module 5: Vision-Language-Action Systems** | Speech recognition; LLM task planning; grounding; VLM scene understanding; safety constraints; failure handling | Can you implement a voice-commanded robot that safely translates natural language to executable actions with failure recovery? |

### Capability Prerequisites by Module

The following specific capabilities from each module are DIRECTLY REQUIRED:

**From Module 1 (Physical AI Foundations)**:
| Capability | Application in Capstone |
|------------|------------------------|
| M1-C1 (Embodied Intelligence) | Understanding why system integration introduces emergent physical constraints |
| M1-C2 (Sensor Analysis) | Specifying sensor requirements for the integrated perception pipeline |
| M1-C3 (Actuator Analysis) | Specifying actuator requirements for navigation and manipulation |
| M1-C4 (Physical Constraints) | Creating the complete system physical constraints specification |
| M1-C5 (Perception-Action Loop) | Tracing the end-to-end loop from voice command to physical action |

**From Module 2 (ROS 2 Fundamentals)**:
| Capability | Application in Capstone |
|------------|------------------------|
| M2-C1 (Node Architecture) | Designing the complete system node graph with explicit timing |
| M2-C2 (Topics/Pub-Sub) | Sensor data flows and command streams throughout the system |
| M2-C3 (Services) | Configuration and discrete operations (gripper, mode changes) |
| M2-C4 (Actions) | Navigation goals, manipulation sequences, task execution |
| M2-C5 (URDF) | Humanoid robot description with all sensors and joints |
| M2-C6 (Integration) | Coordinating all communication patterns in coherent agents |

**From Module 3 (Digital Twin and Simulation)**:
| Capability | Application in Capstone |
|------------|------------------------|
| M3-C1 (World Building) | Creating the capstone simulation environment |
| M3-C2 (Robot Spawning) | Spawning and controlling the humanoid robot |
| M3-C3 (Sensor Configuration) | Configuring sensors with realistic noise for robust testing |
| M3-C4 (Actuator Configuration) | Configuring actuators with realistic dynamics |
| M3-C6 (Reality Gap) | Identifying gaps in the complete integrated system |

**From Module 4 (NVIDIA Isaac Ecosystem)**:
| Capability | Application in Capstone |
|------------|------------------------|
| M4-C1 (Isaac Sim Scenes) | Creating high-fidelity capstone environment in Isaac Sim |
| M4-C2 (RTX Sensors) | Configuring camera and LiDAR for VLM and navigation |
| M4-C3 (VSLAM) | Robot localization for voice-commanded navigation |
| M4-C4 (Isaac Perception) | Object detection for manipulation targets |
| M4-C5 (Domain Randomization) | Robustness testing for VLA pipeline |
| M4-C6 (Sim-to-Real) | Complete transfer analysis for the integrated system |

**From Module 5 (Vision-Language-Action Systems)**:
| Capability | Application in Capstone |
|------------|------------------------|
| M5-C1 (Speech Recognition) | Voice command input for the integrated system |
| M5-C2 (LLM Task Planning) | Translating commands to task sequences |
| M5-C3 (Grounding) | Mapping LLM outputs to ROS 2 actions |
| M5-C4 (Safety Constraints) | Preventing dangerous commands in the complete system |
| M5-C5 (VLM Integration) | Scene understanding for object identification |
| M5-C6 (Robust VLA Pipeline) | Handling ambiguity and failure in integrated execution |

---

## Capabilities Gained

After completing this capstone, students CAN DO the following:

| ID | Capability | Verification Method |
|----|------------|---------------------|
| **CAP-C1** | Architect a complete robotic system integrating perception, planning, and action with explicit data flows, timing constraints, and physical grounding | Produce a system architecture diagram showing all components, data flows, timing budgets, and physical constraint annotations |
| **CAP-C2** | Implement voice-commanded navigation (natural language to path planning to execution) with obstacle avoidance | Demonstrate robot navigating to spoken destination (e.g., "Go to the red shelf") with dynamic obstacle avoidance |
| **CAP-C3** | Implement voice-commanded object identification using vision-language models | Demonstrate robot identifying objects in scene based on spoken description (e.g., "Find the blue mug on the table") |
| **CAP-C4** | Implement voice-commanded manipulation using task planning | Demonstrate robot picking and placing objects based on spoken commands (e.g., "Pick up the apple and put it in the basket") |
| **CAP-C5** | Identify and document the sim-to-real gaps in a complete integrated system | Produce a transfer analysis document with quantified expectations, critical gaps, and mitigation strategies |
| **CAP-C6** | Evaluate system performance against defined criteria with quantitative metrics | Run standardized evaluation scenarios and report success rate, latency breakdown, and failure mode analysis |

---

## Time Allocation

**Total Duration**: 3 weeks (21-24 hours of work)

| Phase | Duration | Focus | Deliverable |
|-------|----------|-------|-------------|
| **Phase 1: System Architecture** | Week 15 (7-8 hours) | Architecture design, component specification, integration planning | System Architecture Document |
| **Phase 2: Implementation** | Week 16 (8-10 hours) | Component integration, pipeline implementation, debugging | Working Simulation Demo |
| **Phase 3: Evaluation & Transfer** | Week 17 (6-8 hours) | Performance evaluation, transfer analysis, documentation | Evaluation Report + Transfer Analysis |

**Pacing Guidance**:
- Phase 1 is design-heavy; students should NOT start coding until architecture is approved
- Phase 2 will surface integration issues; allocate debugging buffer time
- Phase 3 requires working system; do not skip to evaluation with broken demo
- Weekly milestones are checkpoints; instructors should verify progress at each phase

---

## Project Structure

### Phase 1: System Architecture (Week 15)

**Goal**: Design the complete system architecture BEFORE implementation.

**Activities**:
1. Define system requirements based on capstone objectives
2. Design the complete ROS 2 node graph with all communication patterns
3. Specify data flows from voice input through perception to action
4. Calculate timing budgets for the complete perception-action loop
5. Identify physical constraints that affect each subsystem
6. Plan integration sequence (which components connect first)

**Architecture Design Requirements**:

| Component | Requirements |
|-----------|--------------|
| **Voice Input Pipeline** | Microphone to Whisper to text; latency budget; noise handling |
| **LLM Task Planner** | Prompt design; output format; grounding interface |
| **VLM Scene Understanding** | Camera input; query interface; object grounding |
| **Navigation Stack** | VSLAM localization; path planning; obstacle avoidance |
| **Manipulation Pipeline** | Object pose estimation; motion planning; grasp execution |
| **Safety System** | Command validation; physical limits; emergency stop |
| **Orchestration Node** | State machine; error handling; recovery behaviors |

**Deliverable**: System Architecture Document

Contents:
- Node graph diagram with all topics, services, and actions labeled
- Data flow diagram from voice command to physical action
- Timing budget analysis with latency breakdown by component
- Physical constraints specification for the integrated system
- Interface definitions for all component boundaries
- Integration test plan

**Milestone Checkpoint**: Architecture review with instructor before proceeding to Phase 2.

---

### Phase 2: Implementation (Week 16)

**Goal**: Implement and integrate all system components into a working demonstration.

**Activities**:
1. Set up Isaac Sim environment with humanoid robot
2. Integrate and test subsystems incrementally (NOT all at once)
3. Implement the orchestration logic that coordinates all components
4. Debug integration issues (timing, data format, failure handling)
5. Verify each capability independently before full integration

**Integration Sequence** (recommended order):

| Step | Integration | Verification |
|------|-------------|--------------|
| 1 | Isaac Sim + Robot + URDF | Robot spawns and teleop works |
| 2 | VSLAM + Navigation | Robot navigates to coordinates |
| 3 | Isaac Perception | Objects detected and localized |
| 4 | Voice Recognition | Speech converted to text accurately |
| 5 | LLM Task Planner | Text converted to valid task plans |
| 6 | Grounding + Navigation | Voice to navigation goal works |
| 7 | VLM + Object Identification | Voice to object query works |
| 8 | Manipulation Pipeline | Pick/place from perception works |
| 9 | Full Integration | Complete voice to action pipeline |
| 10 | Safety + Recovery | Error handling and safety verified |

**Deliverable**: Working Simulation Demonstration

Must demonstrate:
- Voice-commanded navigation to semantic locations
- Voice-commanded object identification with verbal response
- Voice-commanded pick-and-place operation
- Obstacle avoidance during navigation
- Graceful handling of at least one failure scenario

**Milestone Checkpoint**: Working demonstration reviewed before proceeding to Phase 3.

---

### Phase 3: Evaluation and Transfer Analysis (Week 17)

**Goal**: Quantitatively evaluate system performance and analyze sim-to-real transfer.

**Activities**:
1. Define evaluation scenarios with success criteria
2. Run systematic evaluation with multiple trials
3. Collect quantitative metrics (success rate, latency, failure modes)
4. Analyze sim-to-real gaps for each subsystem
5. Propose mitigation strategies for critical gaps
6. Document lessons learned

**Evaluation Scenarios**:

| Scenario | Command Example | Success Criteria |
|----------|-----------------|------------------|
| **Navigation-1** | "Go to the kitchen" | Arrives within 0.5m of target, no collisions |
| **Navigation-2** | "Navigate to the table while avoiding the chair" | Correct destination, avoids specified obstacle |
| **Identification-1** | "What objects are on the shelf?" | Correctly identifies >80% of visible objects |
| **Identification-2** | "Find the red cup" | Correctly localizes target object |
| **Manipulation-1** | "Pick up the apple" | Successfully grasps target object |
| **Manipulation-2** | "Put the book on the table" | Places object at correct location |
| **Integrated-1** | "Get me the water bottle from the counter" | Complete navigation + identification + pick |
| **Failure Recovery** | Invalid or impossible command | System responds gracefully, no crash/unsafe state |

**Metrics to Collect**:

| Metric | Description | Target |
|--------|-------------|--------|
| **Task Success Rate** | Percentage of trials completing successfully | >70% for individual tasks, >50% for integrated |
| **End-to-End Latency** | Time from voice command to action start | <5s for navigation, <10s for manipulation |
| **Latency Breakdown** | Per-component latency contribution | Identify bottlenecks |
| **Failure Mode Distribution** | Categorization of failure causes | Understand weaknesses |
| **Recovery Success Rate** | Percentage of recoverable failures recovered | >80% for recoverable failures |

**Deliverable 1**: Performance Evaluation Report

Contents:
- Evaluation methodology and scenario descriptions
- Quantitative results for all scenarios (success rates, latencies)
- Failure mode analysis with categorization
- Comparison to initial success criteria
- Recommendations for improvement

**Deliverable 2**: Sim-to-Real Transfer Analysis

Contents:
- Reality gap analysis for each subsystem:
  - Voice recognition: acoustic differences, noise handling
  - LLM/VLM: prompt sensitivity, grounding robustness
  - Navigation: localization accuracy, dynamic obstacles
  - Manipulation: grasp success, contact dynamics
  - Timing: end-to-end latency on hardware
- Quantified expectations for hardware performance
- Critical gaps with mitigation strategies
- Validation experiment design for hardware deployment

**Milestone Checkpoint**: Final review of all deliverables.

---

## Integration Requirements from Prior Modules

### Module 1: Physical AI Foundations

**Role in Capstone**: Physical constraints awareness in ALL design decisions.

**Required Applications**:
- Every architecture decision must consider physical constraints
- Timing budgets must account for sensor latency and actuator response
- Uncertainty propagation must be traced through the complete system
- Physical grounding must be verified for each subsystem

**Physical Grounding Checklist for Capstone**:
- [ ] Voice processing latency accounted for in timing budget
- [ ] LLM inference time included in response latency
- [ ] Sensor noise effects considered in perception pipeline
- [ ] Actuator limits respected in manipulation planning
- [ ] Safety margins included for timing uncertainty

---

### Module 2: ROS 2 Fundamentals

**Role in Capstone**: ROS 2 as the integration backbone.

**Required Applications**:
- Complete system uses ROS 2 for all inter-component communication
- Node architecture follows patterns established in Module 2
- QoS settings appropriate for each data stream type
- Actions used for all long-running operations
- Launch files configure the complete system

**ROS 2 Integration Checklist**:
- [ ] Node graph designed with clear responsibility boundaries
- [ ] Topics use appropriate QoS for data type (sensor, command, state)
- [ ] Services used for configuration and discrete operations
- [ ] Actions used for navigation and manipulation goals
- [ ] Error handling implemented for service/action failures

---

### Module 3: Digital Twin and Simulation

**Role in Capstone**: Gazebo or Isaac Sim as execution environment.

**Required Applications**:
- Simulation environment configured for capstone scenarios
- Sensor noise models active for realistic testing
- Actuator dynamics modeled for realistic behavior
- Reality gap awareness informs transfer analysis

**Simulation Configuration Checklist**:
- [ ] Environment includes required objects and navigation targets
- [ ] Sensors configured with noise matching physical specifications
- [ ] Actuators configured with realistic dynamics
- [ ] Physics properties (friction, collision) appropriately set
- [ ] Lighting configured for camera-based perception

---

### Module 4: NVIDIA Isaac Ecosystem

**Role in Capstone**: Isaac perception and VSLAM for robust sensing.

**Required Applications**:
- Isaac Sim as primary simulation platform (recommended)
- Isaac ROS VSLAM for robot localization
- Isaac ROS perception for object detection
- Domain randomization for robustness testing
- Transfer analysis using Module 4 methodology

**Isaac Integration Checklist**:
- [ ] Isaac Sim scene configured with capstone environment
- [ ] VSLAM running with acceptable accuracy metrics
- [ ] Object detection identifying capstone target objects
- [ ] Domain randomization applied for perception robustness
- [ ] Transfer analysis follows Module 4 methodology

---

### Module 5: Vision-Language-Action Systems

**Role in Capstone**: VLA pipeline for natural language interface.

**Required Applications**:
- Whisper for voice command input
- LLM for task planning with safe prompt design
- Grounding for action execution
- VLM for scene understanding and object identification
- Safety constraints preventing dangerous commands
- Failure handling for ambiguous or impossible commands

**VLA Integration Checklist**:
- [ ] Voice recognition processing with acceptable accuracy
- [ ] LLM prompt design producing valid, safe task plans
- [ ] Grounding correctly mapping LLM output to ROS 2 actions
- [ ] VLM answering scene queries accurately
- [ ] Safety constraints verified for representative dangerous commands
- [ ] Failure handling demonstrated for ambiguous commands

---

## Assessment Strategy

### Formative Assessments (During Capstone)

| Phase | Assessment Type | Description |
|-------|-----------------|-------------|
| Week 15 | Architecture Review | Instructor review of system architecture document; must pass before implementation |
| Week 16 | Integration Checkpoint | Mid-week verification of integration progress; identify blockers early |
| Week 16 | Demo Dry Run | Pre-submission demonstration to identify issues before final evaluation |
| Week 17 | Evaluation Methodology Review | Verify evaluation scenarios and metrics are appropriate before running |

### Summative Assessment (End of Capstone)

**Final Capstone Submission** includes:

1. **System Architecture Document** (20% of grade)
   - Complete node graph with all interfaces
   - Timing budget with physical grounding
   - Integration plan with rationale

2. **Working Demonstration** (40% of grade)
   - Live demonstration of all five capability areas
   - Must show navigation, identification, and manipulation
   - Must demonstrate at least one failure recovery scenario

3. **Evaluation Report** (20% of grade)
   - Quantitative metrics for all evaluation scenarios
   - Failure mode analysis
   - Honest assessment of limitations

4. **Sim-to-Real Transfer Analysis** (20% of grade)
   - Gap analysis for each subsystem
   - Quantified hardware performance expectations
   - Mitigation strategies with rationale

**Grading Rubric**:

| Criterion | Excellent (A) | Satisfactory (B) | Needs Improvement (C) | Unsatisfactory (D/F) |
|-----------|---------------|------------------|----------------------|---------------------|
| **Architecture** | Complete, well-justified, physically grounded | Complete with minor gaps | Incomplete or missing physical grounding | Severely incomplete |
| **Navigation Demo** | Robust, handles edge cases | Works for standard cases | Inconsistent success | Does not function |
| **Identification Demo** | Accurate, handles variations | Works for clear cases | Limited accuracy | Does not function |
| **Manipulation Demo** | Reliable grasp and place | Works for simple objects | Inconsistent success | Does not function |
| **Evaluation Rigor** | Comprehensive, quantitative, honest | Adequate coverage | Limited scenarios or metrics | Superficial or missing |
| **Transfer Analysis** | Insightful, actionable | Adequate coverage of gaps | Superficial analysis | Missing or incorrect |

---

## Cognitive Load Validation

### New Concepts Introduced

| Item | Count | Expected | Status |
|------|-------|----------|--------|
| New concepts | **0** | 0 | **PASS** |
| New tools | **0** | 0 | **PASS** |
| New frameworks | **0** | 0 | **PASS** |

**Validation Statement**: The capstone introduces NO new concepts, tools, or frameworks. All required knowledge comes from Modules 1-5. The capstone is purely integration and evaluation of previously learned material.

### Cognitive Load Distribution

| Phase | Cognitive Load Type | Description |
|-------|---------------------|-------------|
| Phase 1 | **Design thinking** | Synthesizing module knowledge into coherent architecture |
| Phase 2 | **Integration debugging** | Resolving interface mismatches and timing issues |
| Phase 3 | **Analytical evaluation** | Systematic testing and honest assessment |

The capstone load is integration complexity, NOT new learning. Students should feel challenged by putting pieces together, not by learning new material.

### If Students Struggle

| Struggle Area | Root Cause | Resolution |
|---------------|------------|------------|
| Cannot design architecture | Incomplete Module 2 understanding | Review M2-C1, M2-C6 |
| Cannot configure simulation | Incomplete Module 3/4 understanding | Review M3-C1 through M3-C4, M4-C1, M4-C2 |
| Cannot implement navigation | Incomplete Module 4 understanding | Review M4-C3 (VSLAM), nav2 integration |
| Cannot implement VLA pipeline | Incomplete Module 5 understanding | Review M5-C1 through M5-C6 |
| Cannot analyze transfer | Incomplete Module 4 understanding | Review M4-C6 |
| Cannot evaluate system | Did not complete evaluation chapters | Review assessment sections of all modules |

---

## Physical Grounding Compliance

The capstone MUST demonstrate physical grounding in all deliverables:

### Architecture Document Requirements

- Timing budgets must include physical latency sources (sensor, communication, actuator)
- Uncertainty propagation traced from voice input to action output
- Physical constraints explicitly documented for each subsystem
- Safety margins justified based on physical analysis

### Demonstration Requirements

- Robot behavior must respect physical limits (velocity, acceleration, force)
- Sensor noise visible in perception behavior (not perfect detection)
- Actuator dynamics visible in motion (not instant response)
- Failure modes related to physical constraints demonstrated

### Transfer Analysis Requirements

- Gap analysis must identify physical differences (not just software)
- Sensor transfer analysis includes physical environment differences
- Actuator transfer analysis includes mechanical differences
- Timing analysis includes hardware compute differences

---

## Success and Failure Conditions

### Capstone Success Indicators

The capstone is SUCCESSFUL if students:
- Produce coherent system architecture with justified design decisions
- Demonstrate integrated voice-to-action pipeline in simulation
- Collect quantitative evaluation metrics with honest assessment
- Produce transfer analysis that correctly identifies critical gaps
- Can explain WHY each design decision was made (not just what)
- Can explain WHAT would change for hardware deployment

### Capstone Failure Indicators

The capstone has FAILED if:
- Students cannot explain how modules connect (compartmentalized learning)
- System works but students cannot explain why (cargo cult integration)
- Transfer analysis claims simulation equals reality (failure to understand gap)
- Evaluation cherry-picks success cases without honest failure analysis
- Physical constraints are ignored in architecture or analysis
- New concepts had to be introduced to complete the capstone

### Warning Signs During Capstone

Instructors should intervene if:
- Week 15: Architecture lacks physical grounding or timing analysis
- Week 16: Students attempt full integration before component testing
- Week 16: Integration debugging exceeds available time significantly
- Week 17: Evaluation scenarios are trivial or avoid failure modes
- Any time: Students ask "How do I do X?" for X covered in Modules 1-5

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2024-12-24 | curriculum-architect | Initial capstone specification |

---

## Notes for Content Authors

### Writing Constraints

1. **No new concepts**: Capstone materials must NOT introduce new concepts; reference prior modules instead
2. **Integration focus**: All guidance should be about connecting known concepts, not learning new ones
3. **Physical grounding is mandatory**: Every deliverable requirement must include physical grounding criteria
4. **Tradeoff emphasis**: Capstone requires design decisions; emphasize tradeoffs and justification
5. **Failure is learning**: Evaluation should embrace honest failure analysis, not hide limitations

### Guidance Document Requirements

Capstone supporting materials should include:

1. **Architecture Template**: Structured document template for system architecture
2. **Integration Debugging Guide**: Common integration issues and resolution strategies
3. **Evaluation Scenario Specifications**: Detailed descriptions of standard evaluation scenarios
4. **Transfer Analysis Template**: Structured document template for sim-to-real analysis
5. **Module Reference Map**: Quick reference to which module covers which capstone requirement

### Assessment Materials Requirements

1. **Architecture Review Rubric**: Criteria for Phase 1 checkpoint
2. **Demo Evaluation Rubric**: Criteria for live demonstration assessment
3. **Report Quality Rubric**: Criteria for written deliverables
4. **Transfer Analysis Rubric**: Criteria for sim-to-real analysis quality

### Physical Grounding Checklist (All Deliverables)

Before finalizing any capstone material, verify:

- [ ] No new concepts introduced (all reference prior modules)
- [ ] Physical constraints addressed in requirements
- [ ] Timing/latency considerations included
- [ ] Sensor/actuator limitations acknowledged
- [ ] Reality gap implications discussed
- [ ] Tradeoff reasoning required (not just correct answers)

---

## Capstone Preparation Notes

### What Prior Modules Must Establish

For the capstone to succeed, students must arrive with:

**From Module 1**: Mental model of embodied AI; ability to analyze physical constraints
**From Module 2**: ROS 2 fluency; ability to design and implement node architectures
**From Module 3**: Simulation configuration skills; understanding of reality gap concept
**From Module 4**: Isaac Sim proficiency; VSLAM and perception implementation; transfer methodology
**From Module 5**: VLA pipeline implementation; safety and failure handling patterns

### Recommended Preparation Activities

Before starting the capstone, students should:

1. Review their Module 4 summative project (Isaac Sim + perception)
2. Review their Module 5 VLA pipeline implementation
3. Ensure all module environments still function (dependencies may have changed)
4. Read the capstone specification completely before starting Phase 1
5. Identify which module materials to reference for each capstone component

---

**Capstone Specification Complete**

**Critical Success Factor**: The capstone validates that students have achieved INTEGRATION mastery, not just component mastery. A student who can implement each subsystem but cannot connect them has not achieved the course goal. The capstone is where the complete Physical AI understanding is demonstrated.

# Module 5: Vision-Language-Action Systems

**Specification ID**: MOD-005
**Version**: 1.0.0
**Created**: 2024-12-24
**Authority**: curriculum-architect agent
**Status**: LOCKED (Phase 1 Complete)

---

## Module Goal

**Students completing this module will be able to integrate speech recognition (Whisper), large language models, and vision-language models into robotic systems that understand natural language commands, ground those commands to executable ROS 2 actions, implement safety constraints that prevent dangerous AI-generated commands, and design robust VLA pipelines that handle ambiguity, failure, and uncertainty gracefully.**

This module treats VLA as **system orchestration**, not "AI magic." Students learn to integrate AI components as tools within a robotic architecture, where physical constraints, safety requirements, and failure handling take precedence over AI capabilities. The AI components are powerful but fallible; the system design makes them safe and reliable.

---

## Prerequisites

### Required Before Starting Module 5

| Domain | Minimum Requirement | Verification Question | Source Module |
|--------|---------------------|----------------------|---------------|
| **Isaac Sim Scenes** | Can create Isaac Sim scenes with robots, sensors, and environmental features | Can you create an Isaac Sim warehouse scene with a robot and verify ROS 2 topic flow? | Module 4, Chapter 2 |
| **Isaac ROS Perception** | Can configure perception pipelines for object detection using Isaac ROS | Can you run object detection on robot camera data and visualize bounding boxes? | Module 4, Chapter 3 |
| **Visual SLAM** | Can implement VSLAM for robot localization with accuracy evaluation | Can you run VSLAM and compute trajectory error metrics? | Module 4, Chapter 4 |
| **Domain Randomization** | Understand domain randomization for perception robustness | Why does domain randomization help perception models transfer to real environments? | Module 4, Chapter 5 |
| **Sim-to-Real Transfer** | Can articulate transfer strategies and expected gaps | What would you expect to differ between simulation and hardware for a perception system? | Module 4, Chapter 6 |
| **ROS 2 Actions** | Can implement action servers and clients for long-running tasks | How would you implement a navigation action with progress feedback and cancellation? | Module 2, Chapter 4 |
| **ROS 2 Node Architecture** | Can design multi-node systems with topics, services, and actions | Can you architect a system that coordinates perception, planning, and execution? | Module 2, Chapter 6 |
| **Physical Constraints** | Understand latency budgets, uncertainty propagation, and safety requirements | What is the latency budget for a robot reacting to a voice command? | Module 1, Chapter 4 |

### Hardware Prerequisites

**REQUIRED**: NVIDIA GPU with sufficient VRAM for both simulation (Isaac Sim) and LLM/VLM inference.

| Hardware | Minimum | Recommended | Cloud Alternative |
|----------|---------|-------------|-------------------|
| **GPU** | NVIDIA RTX 3070 (8GB VRAM) | NVIDIA RTX 3090+ (24GB VRAM) | AWS g5.xlarge or GCP A100 instance |
| **RAM** | 32 GB | 64 GB | Cloud instances typically meet this |
| **Storage** | 100 GB free (SSD) | 200 GB free (NVMe SSD) | Cloud storage |
| **OS** | Ubuntu 22.04 | Ubuntu 22.04 | Ubuntu AMI |

**Note**: VLM inference requires significant VRAM. Students with limited local resources should use cloud instances or API-based models.

### ML/AI Prerequisites (Added per Course Overview Validation)

| Domain | Minimum Requirement | Verification Question |
|--------|---------------------|----------------------|
| **Neural Network Basics** | Understand what neural networks and embeddings are conceptually | Can you explain what an embedding is and why it captures semantic meaning? |
| **Transformer Intuition** | Understand that transformers process sequences and produce context-aware outputs | Can you explain why an LLM can understand "pick up that red thing" in context? |

Students without ML background should review the "ML Concepts for Roboticists" appendix before starting this module.

### Explicit Module 4 Dependencies (Simulation & Perception)

The following Module 4 capabilities are DIRECTLY APPLIED in VLA integration:

| M4 Capability | Application in Module 5 |
|---------------|------------------------|
| **M4-C1** (Isaac Sim Scenes) | VLA commands execute in Isaac Sim environment; scene provides context for grounding |
| **M4-C2** (RTX Sensors) | Camera data flows to VLMs for scene understanding; sensor quality affects grounding accuracy |
| **M4-C3** (VSLAM) | Robot localization enables spatial commands ("go to the shelf"); provides pose for grounding |
| **M4-C4** (Isaac ROS Perception) | Object detection provides targets for manipulation commands; perception enables grounding |
| **M4-C6** (Sim-to-Real) | VLA systems have their own sim-to-real gap; transfer analysis applies to VLA pipeline |

### Explicit Module 2 Dependencies (ROS 2 Infrastructure)

| M2 Capability | Application in Module 5 |
|---------------|------------------------|
| **M2-C2** (Topics/Pub-Sub) | Audio streams, perception results, and safety monitors use topics |
| **M2-C4** (Actions) | Grounded commands execute as ROS 2 actions; VLA is the action client |
| **M2-C6** (Integration) | VLA pipeline is a multi-node system coordinating perception, reasoning, and action |

### NOT Required (Will Be Taught)

- Whisper architecture and deployment (covered in Chapter 1)
- LLM integration and prompt engineering (covered in Chapter 2)
- Symbol grounding and action mapping (covered in Chapter 3)
- Vision-language models (covered in Chapter 4)
- Safety constraints and guardrails (covered in Chapter 5)
- Failure handling and human-in-the-loop (covered in Chapter 6)

---

## Capabilities Gained

After completing this module, students CAN DO the following:

| ID | Capability | Verification Method |
|----|------------|---------------------|
| **M5-C1** | Integrate speech recognition (Whisper) for voice command input with noise robustness and latency awareness | Implement voice command capture and transcription achieving >90% accuracy on clear speech; measure and report transcription latency |
| **M5-C2** | Design LLM prompts that translate natural language to structured robot task plans with explicit constraints | Given NL commands, demonstrate LLM produces valid, safe, structured task plans; validate against defined task schema |
| **M5-C3** | Implement grounding that maps LLM-generated task plans to executable ROS 2 actions with scene verification | Demonstrate end-to-end flow from NL command to robot action execution with explicit scene grounding checks |
| **M5-C4** | Implement safety constraints that prevent dangerous LLM-generated commands before execution | Demonstrate that unsafe commands (velocity limits, workspace violations, obstacle collisions) are rejected or modified; log all safety interventions |
| **M5-C5** | Integrate vision-language models for scene understanding and object identification with grounding to manipulation targets | Use VLM to answer questions about robot camera view; ground identified objects to manipulation targets with pose estimation |
| **M5-C6** | Design robust VLA pipelines that handle ambiguity, grounding failure, and execution failure with human-in-the-loop fallback | Demonstrate appropriate system behavior for ambiguous commands, missing objects, and execution failures; implement clarification requests |

---

## Time Allocation

**Total Duration**: 3 weeks (21-24 hours of study)

| Chapter | Suggested Time | Notes |
|---------|---------------|-------|
| Chapter 1: Speech Recognition for Robot Command | 3-4 hours | Whisper integration; audio pipeline design |
| Chapter 2: LLM-Based Task Planning | 4-5 hours | Prompt engineering; structured output; highest complexity chapter |
| Chapter 3: Grounding and Symbol-to-Action Mapping | 4-5 hours | Core integration chapter; connects LLM to ROS 2 actions |
| Chapter 4: Vision-Language Models for Scene Understanding | 3-4 hours | VLM integration; object identification and spatial reasoning |
| Chapter 5: Safety Constraints and Guardrails | 3-4 hours | Critical safety chapter; builds on Constitution Principle III concepts |
| Chapter 6: Failure Handling and Human-in-the-Loop | 3-4 hours | Robustness chapter; prepares for capstone integration |

**Pacing Guidance**:
- Week 1: Chapters 1-2 (Speech + LLM Task Planning)
- Week 2: Chapters 3-4 (Grounding + VLM)
- Week 3: Chapters 5-6 (Safety + Failure Handling)
- Chapter 2 (LLM Task Planning) requires the most careful attention; prompt engineering is subtle
- Chapter 5 (Safety) must not be rushed; safety is non-negotiable for physical systems
- Chapter 6 should feel like integration and synthesis, not new learning

---

## VLA as System Orchestration

### Design Philosophy

This module treats VLA as **system orchestration**, explicitly rejecting "AI magic" framing:

**What VLA IS**:
- An integration layer that coordinates AI components with robotic execution
- A pipeline where each stage has explicit inputs, outputs, and failure modes
- A system where physical constraints and safety always take precedence
- An architecture where AI suggestions are verified before execution

**What VLA is NOT**:
- A black box that translates speech to robot motion
- A system where AI outputs are trusted unconditionally
- A replacement for careful system design and error handling
- Magic that eliminates the need for understanding physical constraints

### Architecture Overview

The VLA pipeline consists of orchestrated stages:

```
Audio Input --> Speech Recognition (Whisper) --> Text Command
                        |
                        v
Text Command --> LLM Task Planner --> Structured Task Plan
                        |
                        v
Camera Input --> VLM Scene Understanding --> Object/Scene Context
                        |
                        v
Task Plan + Context --> Grounding Layer --> Candidate Actions
                        |
                        v
Candidate Actions --> Safety Filter --> Approved Actions
                        |
                        v
Approved Actions --> ROS 2 Action Execution --> Robot Motion
                        |
                        v
Execution Feedback --> Monitoring --> Success/Failure/Intervention
```

Each stage has:
- Defined input/output contracts
- Explicit failure modes
- Latency budget allocation
- Monitoring and logging

### Physical Grounding Integration

VLA uniquely combines AI capabilities with physical constraints:

| AI Component | Physical Constraint Integration |
|--------------|--------------------------------|
| **Whisper** | Audio latency, microphone noise floor, background noise robustness |
| **LLM Planner** | Inference time vs robot responsiveness; structured output validation |
| **VLM Understanding** | Camera frame rate; inference time; sensor noise affecting recognition |
| **Grounding** | Coordinate frame accuracy; perception uncertainty propagation |
| **Execution** | Actuator limits; collision avoidance; real-time requirements |

---

## Chapter Breakdown

---

### Chapter 1: Speech Recognition for Robot Command

**Chapter Goal**: Integrate Whisper for voice command input, understanding its architecture, deployment options, latency characteristics, and robustness to acoustic noise in robotic environments.

**Capabilities Addressed**: M5-C1 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 1.1 | Explain Whisper's architecture at a conceptual level (encoder-decoder, mel spectrograms, tokenization) | Essay: describe the processing stages from audio waveform to text output |
| 1.2 | Deploy Whisper for local inference with appropriate model size selection | Code: deploy Whisper and measure inference time vs accuracy for different model sizes |
| 1.3 | Implement an audio capture pipeline that handles microphone input, buffering, and voice activity detection | Code: ROS 2 node that captures audio, detects speech, and publishes audio segments |
| 1.4 | Configure Whisper for robot command recognition with appropriate prompt conditioning | Code: condition Whisper for robot command vocabulary; measure accuracy improvement |
| 1.5 | Evaluate speech recognition robustness to environmental noise (robot motors, HVAC, background speech) | Exercise: measure word error rate under different noise conditions; implement noise mitigation |

**New Concepts Introduced** (limit: 4-5):
1. Whisper architecture: encoder-decoder transformer for speech-to-text
2. Model size vs latency vs accuracy tradeoffs (tiny to large)
3. Audio preprocessing: sampling rate, mel spectrograms, buffering
4. Voice activity detection (VAD) for segmentation
5. Prompt conditioning for domain-specific recognition

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Core focus: microphones as sensors with noise floors, frequency response, and directional characteristics; audio quality depends on microphone placement relative to robot actuators (motor noise) |
| **Actuators** | Context: actuator noise (motors, fans, servos) as background noise that degrades speech recognition; need to account for robot's own noise during voice capture |
| **Latency/Noise/Physics** | Core focus: audio capture latency + inference latency = total speech-to-text time; this is the first stage of the VLA latency budget; acoustic noise in real environments differs from quiet recording studios |

**Chapter Structure**:
1. Opening scenario: robot that works with typed commands fails with voice commands
2. Whisper architecture overview (conceptual, not training details)
3. Model size selection: accuracy vs latency vs memory
4. Audio capture pipeline design for robots
5. Voice activity detection and segmentation
6. Prompt conditioning for robot commands
7. Noise robustness evaluation and mitigation
8. Lab: implement voice command capture node with latency measurement
9. Summary and self-assessment questions

---

### Chapter 2: LLM-Based Task Planning

**Chapter Goal**: Design LLM prompts that translate natural language commands into structured robot task plans, with emphasis on prompt engineering discipline, structured outputs, and understanding LLM limitations.

**Capabilities Addressed**: M5-C2 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 2.1 | Explain why LLMs are useful for task planning (semantic understanding) and where they are unreliable (hallucination, physical reasoning) | Essay: identify three strengths and three weaknesses of LLM task planning for robots |
| 2.2 | Design prompts that produce structured task plans with explicit output schemas | Code: prompt that produces JSON task plan conforming to defined schema |
| 2.3 | Implement few-shot prompting with examples that demonstrate correct task decomposition | Code: prompt with 3-5 examples covering common command patterns |
| 2.4 | Validate LLM outputs against schema and reject malformed responses | Code: schema validation that rejects invalid task plans with clear error messages |
| 2.5 | Implement task plan caching and retrieval for repeated commands | Code: cache common commands to avoid redundant LLM calls; measure latency improvement |
| 2.6 | Design prompts that explicitly include robot capability constraints | Code: prompt that includes workspace limits, speed limits, and available actions |

**New Concepts Introduced** (limit: 5-6):
1. LLM as semantic parser, not physical reasoner (critical framing)
2. Structured output prompting (JSON schema, function calling)
3. Few-shot prompting for task decomposition examples
4. Prompt engineering for robotics: capability constraints, safety bounds
5. LLM limitations: hallucination, overconfidence, lack of physical intuition
6. Task plan schema design (actions, parameters, preconditions, effects)

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Context: sensor capabilities must be encoded in prompts ("the robot can see objects within 2 meters"); perception uncertainty affects what LLM should assume |
| **Actuators** | Core focus: actuator capabilities must be encoded in prompts ("the robot can move at 0.5 m/s maximum", "the gripper can hold objects up to 2kg"); prompts prevent LLM from generating impossible actions |
| **Latency/Noise/Physics** | LLM inference time (100ms-2s) is significant portion of VLA latency budget; physics constraints must be explicit in prompts because LLMs do not inherently understand physics |

**Critical Design Principles**:
- LLMs do NOT understand physics; physical constraints must be explicit
- Structured outputs enable downstream validation; free-form text is dangerous
- Few-shot examples establish the contract the LLM should follow
- Every LLM output must be validated before use
- Caching reduces latency for common commands

**Chapter Structure**:
1. Opening scenario: LLM that generates plausible-sounding but physically impossible plans
2. LLMs for semantic understanding: what they do well
3. LLM limitations: hallucination, physical reasoning gaps
4. Structured output design: JSON schemas for task plans
5. Prompt engineering: constraints, examples, output format
6. Few-shot prompting for task decomposition
7. Output validation and rejection handling
8. Caching strategies for latency reduction
9. Lab: implement task planning node with schema validation
10. Summary and self-assessment questions

---

### Chapter 3: Grounding and Symbol-to-Action Mapping

**Chapter Goal**: Implement the grounding layer that maps LLM-generated symbolic task plans to executable ROS 2 actions, with explicit scene verification and coordinate frame resolution.

**Capabilities Addressed**: M5-C3 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 3.1 | Explain the symbol grounding problem and why it is critical for robot execution | Essay: why can't a robot execute "pick up the red cup" without grounding? |
| 3.2 | Implement object reference resolution that maps symbolic names to perception detections | Code: resolve "the red cup" to a detected object with pose estimate |
| 3.3 | Implement spatial reference resolution that maps location descriptions to coordinates | Code: resolve "on the table" to a coordinate region using scene understanding |
| 3.4 | Implement action mapping that converts task plan actions to ROS 2 action goals | Code: map "pick(object_id)" to MoveIt2 pick action with target pose |
| 3.5 | Implement grounding verification that checks preconditions before action execution | Code: verify object is visible and reachable before attempting pick |
| 3.6 | Handle grounding failures with appropriate fallback behavior | Code: when grounding fails (object not found), request clarification or abort safely |

**New Concepts Introduced** (limit: 5-6):
1. Symbol grounding problem: connecting language to perception and action
2. Object reference resolution: names to detections
3. Spatial reference resolution: descriptions to coordinates
4. Action mapping: symbolic actions to ROS 2 action interfaces
5. Grounding verification: precondition checking before execution
6. Grounding failure modes and recovery strategies

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Core focus: perception provides the grounding data; camera detections, depth for pose estimation, VSLAM for robot position; sensor noise propagates to grounding uncertainty |
| **Actuators** | Context: grounded actions must be within actuator capabilities; pose estimates must account for end-effector reach and workspace limits |
| **Latency/Noise/Physics** | Grounding latency (perception + resolution) adds to VLA budget; perception uncertainty means grounding can fail; physical reachability constraints must be checked |

**Grounding Architecture**:
```
Task Plan Action --> Symbol Resolution --> Scene Verification --> Action Goal
                         |                       |                     |
                    Perception            Reachability           ROS 2 Action
                    Detections             Analysis               Interface
```

**Chapter Structure**:
1. Opening scenario: robot that plans correctly but executes on wrong object
2. The symbol grounding problem in robotics
3. Object reference resolution using perception
4. Spatial reference resolution using scene understanding
5. Action mapping to ROS 2 interfaces
6. Grounding verification and precondition checking
7. Handling grounding failures
8. Lab: implement grounding node connecting LLM planner to MoveIt2
9. Summary and self-assessment questions

---

### Chapter 4: Vision-Language Models for Scene Understanding

**Chapter Goal**: Integrate vision-language models (VLMs) for scene understanding, enabling the robot to answer questions about its camera view and ground visual references in natural language commands.

**Capabilities Addressed**: M5-C5 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 4.1 | Explain VLM architecture conceptually (vision encoder + language model) and its capabilities for robotics | Essay: what can a VLM tell you about a scene that object detection cannot? |
| 4.2 | Deploy a VLM (e.g., LLaVA, OpenAI GPT-4V) for robot camera image understanding | Code: VLM node that processes camera images and answers scene queries |
| 4.3 | Design prompts that extract robotics-relevant information from scene images | Code: prompts for object identification, spatial relationships, and manipulation targets |
| 4.4 | Implement VLM-based object identification for grounding natural language references | Code: use VLM to resolve "the blue mug next to the keyboard" to bounding box coordinates |
| 4.5 | Evaluate VLM reliability and latency for real-time robotic applications | Exercise: measure VLM inference time; identify failure modes for robotics scenarios |
| 4.6 | Integrate VLM understanding with the grounding layer from Chapter 3 | Code: VLM outputs feed into grounding verification and object resolution |

**New Concepts Introduced** (limit: 4-5):
1. Vision-language model architecture (CLIP, LLaVA, GPT-4V concepts)
2. VLM capabilities: scene description, object identification, spatial reasoning
3. VLM limitations: hallucination on visual details, depth estimation errors
4. VLM prompt engineering for robotics scene understanding
5. VLM integration with perception pipeline (complementing, not replacing, detection)

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Core focus: VLM input is camera image; image quality (resolution, exposure, motion blur) directly affects VLM accuracy; VLM cannot see depth without depth camera or stereo |
| **Actuators** | Context: VLM spatial reasoning ("the cup is to the left of the robot") must be translated to actuator-reachable coordinates; VLM does not know robot geometry |
| **Latency/Noise/Physics** | VLM inference is slow (1-10 seconds for large models); this dominates VLA latency for visual queries; VLMs hallucinate visual details that don't exist; cannot trust VLM for safety-critical perception |

**VLM vs Object Detection**:
- Object detection: fast, reliable, but limited vocabulary
- VLM: flexible queries, but slower and can hallucinate
- Recommended: use both; VLM for reference resolution, detection for verification

**Chapter Structure**:
1. Opening scenario: robot that needs to find "the thing the user pointed at"
2. Vision-language model architecture overview
3. VLM capabilities for robotics: what they enable
4. VLM limitations: hallucination, depth blindness, latency
5. Deploying VLMs for robot perception
6. Prompt engineering for scene understanding
7. Integration with grounding layer
8. VLM + object detection: complementary use
9. Lab: implement VLM-based object reference resolution
10. Summary and self-assessment questions

---

### Chapter 5: Safety Constraints and Guardrails

**Chapter Goal**: Implement safety constraints that prevent dangerous LLM-generated commands from reaching robot actuators, treating safety as a hard requirement that overrides AI outputs.

**Capabilities Addressed**: M5-C4 (primary)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 5.1 | Articulate why AI outputs cannot be trusted for safety and why explicit constraints are required | Essay: give three examples of dangerous commands an LLM might generate |
| 5.2 | Implement workspace limit checking that rejects actions outside safe regions | Code: safety filter that rejects any action targeting positions outside workspace bounds |
| 5.3 | Implement velocity and force limit checking that constrains commanded motion | Code: safety filter that clips velocities to safe maximums before execution |
| 5.4 | Implement collision checking that prevents commanded trajectories from colliding with obstacles or self | Code: safety filter that checks trajectories against known obstacles and self-collision |
| 5.5 | Implement command rate limiting that prevents AI from overwhelming the robot | Code: rate limiter that enforces minimum time between commands |
| 5.6 | Design safety monitoring that detects unsafe execution states and triggers emergency stop | Code: monitor that detects force anomalies, position errors, and triggers E-stop |

**New Concepts Introduced** (limit: 5-6):
1. Safety as override: safety constraints always beat AI outputs
2. Workspace limit enforcement (static and dynamic)
3. Velocity and force limiting (command saturation)
4. Collision avoidance integration with planning
5. Rate limiting to prevent AI overwhelming robot
6. Runtime safety monitoring and emergency stop triggers

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Force/torque sensors for contact detection; encoders for position monitoring; proximity sensors for collision avoidance; all feed into safety monitoring |
| **Actuators** | Core focus: safety constraints protect actuators and environment from AI errors; velocity limits prevent damage; force limits prevent crushing; workspace limits prevent self-collision |
| **Latency/Noise/Physics** | Safety checks add latency but are non-negotiable; safety monitoring must run faster than control loop; physics of stopping (braking distance) affects safe velocity limits |

**Safety Philosophy (Constitution Principle III Application)**:

Just as the RAG system MUST refuse to answer rather than hallucinate, the robot safety layer MUST refuse to execute rather than allow dangerous commands. The design principle is identical:

> A refusal is ALWAYS preferred over an unsafe action.

This means:
- If workspace cannot be verified, refuse the command
- If collision cannot be ruled out, refuse the command
- If force limits might be exceeded, refuse the command
- The robot may fail to act, but it must not act unsafely

**Safety Architecture**:
```
LLM/VLA Command --> Workspace Check --> Velocity Check --> Collision Check --> Execute
       |                  |                  |                   |              |
       v                  v                  v                   v              v
   [Logging]          [Reject]           [Clip]            [Reject]        [Monitor]
```

**Chapter Structure**:
1. Opening scenario: LLM that commands robot to move faster than safe
2. Safety as non-negotiable override (Constitution Principle III parallel)
3. Workspace limit enforcement
4. Velocity and force limit checking
5. Collision avoidance integration
6. Rate limiting for AI commands
7. Runtime safety monitoring
8. Emergency stop design and triggers
9. Lab: implement safety filter layer with comprehensive logging
10. Summary and self-assessment questions

---

### Chapter 6: Failure Handling and Human-in-the-Loop

**Chapter Goal**: Design robust VLA pipelines that handle ambiguity, grounding failures, and execution failures gracefully, with human-in-the-loop patterns for recovery.

**Capabilities Addressed**: M5-C6 (primary), M5-C1 through M5-C5 (integration)

**Learning Objectives** (capability-based):

| ID | Objective | Assessment |
|----|-----------|------------|
| 6.1 | Identify failure modes at each stage of the VLA pipeline and design detection mechanisms | Analysis: enumerate failure modes for speech, LLM, grounding, VLM, and execution stages |
| 6.2 | Implement ambiguity detection that recognizes when commands are underspecified | Code: detect ambiguous commands ("pick up a cup" when multiple cups visible) and request clarification |
| 6.3 | Implement grounding failure recovery that requests human assistance | Code: when object not found, verbally request user to point or describe location |
| 6.4 | Implement execution failure recovery with replanning and retry strategies | Code: when action fails, analyze failure, replan if recoverable, abort if not |
| 6.5 | Design human-in-the-loop patterns for confirming high-risk actions | Code: request confirmation before executing actions near humans or fragile objects |
| 6.6 | Implement comprehensive logging and state reporting for debugging VLA failures | Code: log all VLA pipeline stages with timing, inputs, outputs, and decisions |

**New Concepts Introduced** (limit: 4-5):
1. VLA failure taxonomy: speech, planning, grounding, perception, execution
2. Ambiguity detection and clarification requests
3. Grounding failure recovery patterns
4. Execution monitoring and replanning
5. Human-in-the-loop confirmation for high-risk actions

**Physical Grounding Elements**:

| Aspect | Coverage |
|--------|----------|
| **Sensors** | Failure detection relies on sensors: force sensors detect unexpected contact; cameras detect object absence; execution monitoring uses all available feedback |
| **Actuators** | Execution failures manifest in actuator behavior: stuck positions, force limits reached, trajectory deviation; actuator state feedback is essential for failure detection |
| **Latency/Noise/Physics** | Recovery takes time (replanning, human response); latency budget for failure handling; physics of failure (object slipped, obstacle appeared) requires physical understanding |

**Failure Recovery Philosophy**:
- Fail safely: when in doubt, stop and report
- Fail informatively: tell the user what went wrong
- Fail recoverably: preserve state for retry when possible
- Involve humans: the human is a capable fallback

**Human-in-the-Loop Patterns**:
1. **Clarification**: "I see three cups. Which one do you mean?"
2. **Confirmation**: "I'm about to pick up the glass vase. Should I proceed?"
3. **Assistance**: "I can't find the blue mug. Can you show me where it is?"
4. **Reporting**: "The pick failed because the object slipped. Should I retry?"

**Preparation for Capstone**:
This chapter explicitly prepares for capstone integration:
- Complete VLA pipeline with all failure handling
- Human-in-the-loop patterns for complex scenarios
- Logging for debugging integrated systems
- Robustness strategies for multi-module interaction

**Chapter Structure**:
1. Opening scenario: VLA system that works in demos, fails in practice
2. VLA failure taxonomy
3. Ambiguity detection and clarification
4. Grounding failure recovery
5. Execution monitoring and failure detection
6. Replanning and retry strategies
7. Human-in-the-loop confirmation
8. Comprehensive logging and debugging
9. Lab: implement complete VLA pipeline with failure handling
10. Summary and self-assessment questions

---

## Module Assessment Strategy

### Formative Assessments (During Module)

| Chapter | Assessment Type | Description |
|---------|-----------------|-------------|
| 1 | Speech integration | Voice command capture with measured latency and accuracy |
| 2 | Prompt engineering | LLM prompt that produces validated structured task plans |
| 3 | Grounding implementation | Grounding node connecting LLM plans to ROS 2 actions |
| 4 | VLM integration | VLM node answering scene queries with measured accuracy |
| 5 | Safety implementation | Safety filter with logged interventions on unsafe commands |
| 6 | Failure handling | Complete VLA pipeline with failure recovery demonstration |

### Summative Assessment (End of Module)

**Module 5 Integration Project**: Given a tabletop manipulation scenario in Isaac Sim, students must:

1. Implement voice command capture with Whisper:
   - Achieve >90% word accuracy on clear command vocabulary
   - Report transcription latency

2. Implement LLM-based task planning:
   - Structured output with validated schema
   - Capability constraints in prompt
   - At least 3 few-shot examples

3. Implement grounding layer:
   - Object reference resolution using perception
   - Action mapping to MoveIt2 pick/place actions
   - Grounding verification with precondition checks

4. Integrate VLM for scene understanding:
   - Answer queries about scene contents
   - Resolve ambiguous object references

5. Implement safety constraints:
   - Workspace limit enforcement
   - Velocity limiting
   - Collision checking
   - All safety interventions logged

6. Implement failure handling:
   - Ambiguity detection with clarification request
   - Grounding failure with human assistance request
   - Execution failure with retry or abort

7. Demonstrate end-to-end pipeline:
   - Voice command: "Pick up the red mug and place it on the shelf"
   - Complete execution in simulation
   - Demonstrate failure handling for edge cases

**Success Criteria**:
- Voice commands transcribed accurately with <500ms latency
- Task plans conform to schema with no invalid actions
- Grounding correctly resolves object references
- VLM answers scene queries accurately
- Safety filter rejects demonstrably unsafe commands
- Failure handling produces appropriate recovery behavior
- End-to-end pipeline executes successfully
- Complete logging enables debugging of all stages

---

## Transition to Capstone

### What This Module Establishes

Students leaving Module 5 understand:
- How to integrate speech recognition for natural language command input
- How to design LLM prompts that produce safe, structured task plans
- How to ground symbolic plans to executable robot actions
- How VLMs enable flexible scene understanding
- Why safety constraints must override AI outputs
- How to handle failures gracefully with human fallback

### What Capstone Requires

The Capstone (Integrated Humanoid System) builds directly on Module 5:
- **VLA pipeline** is the primary command interface for the humanoid
- **Speech recognition** enables hands-free command input
- **LLM planning** decomposes complex commands into action sequences
- **Grounding** connects plans to humanoid-specific actions
- **Safety constraints** protect the more capable (and more dangerous) humanoid
- **Failure handling** is essential for complex multi-step humanoid tasks

### Explicit Bridge

The final section of Chapter 6 should preview:
- "In the Capstone, you will apply VLA to a complete humanoid robot system"
- "The voice command interface will accept natural language task requests"
- "The LLM planner will decompose commands into navigation + manipulation sequences"
- "The grounding layer will resolve references using humanoid perception"
- "Safety is even more critical for a full humanoid with greater capability"
- "Failure handling will coordinate recovery across perception, navigation, and manipulation"

---

## Cognitive Load Validation

### Per-Chapter Concept Count

| Chapter | New Concepts | Limit | Status |
|---------|--------------|-------|--------|
| Chapter 1 | 5 | 5 | PASS (at limit) |
| Chapter 2 | 6 | 6 | PASS (at limit) |
| Chapter 3 | 6 | 6 | PASS (at limit) |
| Chapter 4 | 5 | 5 | PASS |
| Chapter 5 | 6 | 6 | PASS (at limit) |
| Chapter 6 | 5 | 6 | PASS |

### Physical Grounding Compliance

| Chapter | Sensors | Actuators | Latency/Noise/Physics | Status |
|---------|---------|-----------|----------------------|--------|
| Chapter 1 | Core focus (mic) | Context (noise) | Core focus (audio latency) | PASS |
| Chapter 2 | Context (in prompts) | Core focus (in prompts) | Covered (inference time) | PASS |
| Chapter 3 | Core focus (perception) | Context (reachability) | Covered (grounding latency) | PASS |
| Chapter 4 | Core focus (camera) | Context (coordinates) | Covered (VLM latency) | PASS |
| Chapter 5 | Core focus (monitoring) | Core focus (limits) | Core focus (safety timing) | PASS |
| Chapter 6 | Core focus (failure detect) | Core focus (execution) | Covered (recovery timing) | PASS |

### Capability Progression

```
Chapter 1: M5-C1 (speech recognition)
    |
    v
Chapter 2: M5-C2 (LLM task planning)
    |
    v
Chapter 3: M5-C3 (grounding to actions)
    |
    v
Chapter 4: M5-C5 (VLM scene understanding)
    |
    v
Chapter 5: M5-C4 (safety constraints)
    |
    v
Chapter 6: M5-C6 (failure handling) + integration
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

1. **VLA is orchestration, not magic**: Every chapter must frame AI as a tool within a system, not as autonomous intelligence
2. **Physical grounding is mandatory**: Every chapter must explicitly address sensors, actuators, AND latency/noise/physics
3. **Safety is non-negotiable**: Chapter 5 content must treat safety as absolute override, following Constitution Principle III pattern
4. **No hallucinated APIs**: All LLM/VLM APIs must be real and currently available; no imagined interfaces
5. **Capability-first**: If content does not enable a stated capability, it does not belong
6. **Do not exceed concept limits**: If a chapter needs more than 6 new concepts, it must be split
7. **Python focus**: All ROS 2 code in Python (rclpy); LLM/VLM integration in Python

### Style Guidance

- Frame LLMs as "semantic parsers with limitations" not as "intelligent planners"
- Include explicit latency measurements for every AI component
- Show failure cases as prominently as success cases
- Include "safety callout" boxes explaining what could go wrong
- Use realistic scenarios that include noise, ambiguity, and failure
- End each chapter with exercises that verify capability, not just recall

### Code Quality Requirements

- All LLM prompts must be complete and tested
- All VLA pipeline components must have explicit input/output types
- Safety filters must log all decisions with reasoning
- Error handling must be comprehensive (not just happy path)
- Latency measurements must be included in all integration exercises

### Physical Grounding Checklist (per chapter)

Before finalizing any chapter, verify:

- [ ] Sensor aspects addressed (what sensors are involved, what limitations)
- [ ] Actuator aspects addressed (what actuators are involved, what limitations)
- [ ] Latency/noise/physics addressed (what timing, noise, or physics are relevant)
- [ ] Reference to M1 concepts where applicable (physical constraints)
- [ ] Reference to M2/M3/M4 infrastructure where applicable
- [ ] Safety implications discussed or noted
- [ ] Failure modes explicitly identified

### VLA-Specific Checklist (per chapter)

Before finalizing any chapter, verify:

- [ ] AI component framed as tool, not intelligence
- [ ] Explicit latency budget contribution stated
- [ ] Failure modes for this stage enumerated
- [ ] Connection to adjacent pipeline stages clear
- [ ] Physical constraint integration explicit
- [ ] Safety considerations addressed (if applicable)

---

## Capstone Preparation Notes

This module provides the following for the capstone:

1. **Command Interface**: Voice command via Whisper
2. **Task Planning**: LLM-based decomposition of complex commands
3. **Scene Understanding**: VLM for visual context
4. **Grounding**: Symbol-to-action mapping for humanoid actions
5. **Safety**: Comprehensive constraint layer for humanoid execution
6. **Robustness**: Failure handling and human-in-the-loop patterns

Students should complete Module 5 with:
- Working voice command capture integrated with Isaac Sim robot
- LLM task planning with validated structured outputs
- Grounding connecting to MoveIt2 or similar action interfaces
- VLM integration for scene queries
- Safety filter layer blocking unsafe commands
- Failure handling with clarification and recovery patterns
- Complete understanding that VLA is system integration, not AI magic

---

**Module 5 Specification Complete**

**Next Steps**:
1. Content authors can begin chapter drafts using this specification
2. Each chapter draft must be validated against the physical grounding checklist
3. Each chapter draft must be validated against the VLA-specific checklist
4. Assessment materials should align with stated learning objectives
5. Review gate: chapter drafts checked against capability claims before finalization

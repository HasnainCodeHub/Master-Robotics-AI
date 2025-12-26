---
id: 0006
title: Module 3 Digital Twin and Simulation Specification
stage: spec
date: 2024-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: curriculum-architect
command: Task 1.5 - Module 3 Specification
labels: ["curriculum", "module-3", "simulation", "gazebo", "unity", "digital-twin"]
links:
  spec: specs/curriculum/module-3.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/curriculum/module-3.md
tests:
  - Physical grounding compliance validated for all 6 chapters
  - Cognitive load validation passed (no chapter exceeds 6 concepts)
  - Prerequisite dependencies explicitly mapped to M1 and M2
---

## Prompt

You are the curriculum-architect agent executing Task 1.5 — Module 3 Specification (Simulation & Digital Twin).

## Context (from clarified spec)

**Physical Grounding Requirement**: Each chapter must address all three aspects: sensor data, actuator constraints, and latency/noise/physics realities

**Mathematical Depth**: Moderate with applied equations

**Platform Mastery Goal**: Production-ready skills

**Module 3 Requirements** (from spec):
- Digital Twin & Simulation
- Gazebo
- Unity
- Sensor simulation

## Your Task

Create the file: `specs/curriculum/module-3.md`

Define:
1. **Module Goal** - Single clear capability statement
2. **Capabilities Gained** - List of what students CAN DO after this module
3. **Chapter List** - Each chapter with:
   - Title
   - Learning objectives (capability-based)
   - Physical grounding elements (sensors/actuators/latency-noise-physics)
   - Simulation environment mapping (which tool: Gazebo vs Unity)
4. **Prerequisites** - Explicitly reference dependencies on Modules 1-2
5. **Time Allocation** - Suggested weeks/hours per chapter
6. **Platform Roles**:
   - Gazebo role (primary physics simulation)
   - Unity role (visualization, alternative simulation)
   - ROS 2 integration expectations

## Constraints
- Each chapter introduces 1-2 capabilities maximum
- Physical grounding is MANDATORY for every chapter
- Clearly distinguish when to use Gazebo vs Unity
- Identify "reality gap" concepts that matter for sim-to-real

## Response snapshot

Created comprehensive Module 3 specification with:

**Module Goal**: Students will create physics-based simulation environments in Gazebo and Unity, configure simulated sensors and actuators with realistic noise models matching M1 specifications, spawn robots using URDF from M2, and critically evaluate the reality gap.

**6 Capabilities Defined**:
- M3-C1: Create Gazebo worlds with SDF
- M3-C2: Spawn and control robots via ROS 2
- M3-C3: Configure sensors with realistic noise models
- M3-C4: Configure actuators with realistic dynamics
- M3-C5: Use Unity as alternative simulation with ROS 2 integration
- M3-C6: Identify and articulate reality gap

**6 Chapters**:
1. Simulation Fundamentals & Physics Engines
2. Gazebo World Building with SDF
3. Simulated Sensors with Realistic Noise Models
4. Simulated Actuators and Control Interfaces
5. Unity and ROS 2 Integration
6. Reality Gap and Sim-to-Real Foundations

**Platform Roles Defined**:
- Gazebo: Primary physics simulation (contact, dynamics, headless testing)
- Unity: Visualization, photorealistic rendering, perception training data
- Decision framework provided for choosing between platforms

**Physical Grounding Compliance**:
- All chapters address sensors, actuators, and latency/noise/physics
- Explicit M1/M2 prerequisite mapping with capability IDs
- Reality gap explicitly discussed with taxonomy

## Outcome

- **Impact**: Module 3 specification complete; defines simulation fundamentals that Module 4 (Isaac) will build upon
- **Tests**: All chapters validated against physical grounding checklist; cognitive load within limits
- **Files**: specs/curriculum/module-3.md created (575 lines)
- **Next prompts**: Module 4 specification (NVIDIA Isaac), Module 5 specification (VLA)
- **Reflection**: Strong emphasis on reality gap prepares students for sim-to-real challenges; explicit platform role distinction prevents confusion

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Physical grounding compliance PASS, Concept count PASS, Prerequisite mapping PASS
- Prompt variant (if applicable): Standard curriculum-architect task
- Next experiment (smallest change to try): Consider adding explicit hardware requirement callout for GPU-intensive simulation

---
id: 0004
title: Module 1 Physical AI Foundations Specification
stage: spec
date: 2024-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: curriculum-architect
command: Task 1.3 - Module 1 Specification
labels: ["curriculum", "module-specification", "physical-ai", "foundations"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/curriculum/module-1.md
tests:
  - null
---

## Prompt

You are the curriculum-architect agent executing Task 1.3 — Module 1 Specification (Physical AI Foundations).

## Context (from clarified spec)

**Physical Grounding Requirement**: Each chapter must address all three aspects: sensor data, actuator constraints, and latency/noise/physics realities

**Mathematical Depth**: Moderate with applied equations — practical derivations for transformations, kinematics, and control; no formal proofs

**Platform Mastery Goal**: Production-ready skills

**Module 1 Requirements** (from spec):
- Physical AI foundations
- Embodied intelligence
- Sensors
- Actuators
- Physical constraints

## Your Task

Create the file: `specs/curriculum/module-1.md`

Define:
1. **Module Goal** - Single clear capability statement
2. **Capabilities Gained** - List of what students CAN DO after this module (capability-based)
3. **Chapter List** - Each chapter with:
   - Title
   - Learning objectives (capability-based)
   - Physical grounding elements (sensors/actuators/latency-noise-physics for each)
   - Simulation or conceptual mapping (how this will be demonstrated)
4. **Prerequisites** - What students need before starting this module
5. **Time Allocation** - Suggested weeks/hours per chapter

## Constraints
- Each chapter introduces 1-2 capabilities maximum
- Physical grounding is MANDATORY for every chapter
- No actual content writing - structure and objectives only
- All objectives must be testable/verifiable

## Response snapshot

Created comprehensive Module 1 specification at `specs/curriculum/module-1.md` with:

**Module Goal**: Students will be able to analyze any robotic system and identify its embodied intelligence characteristics: sensors, actuators, and physical constraints.

**5 Capabilities Defined** (M1-C1 through M1-C5):
- M1-C1: Articulate embodied vs disembodied AI
- M1-C2: Categorize sensors with noise/latency/failure analysis
- M1-C3: Categorize actuators with response/force/safety analysis
- M1-C4: Identify physical constraints for robotic tasks
- M1-C5: Trace complete perception-action loop with bottleneck identification

**4 Chapters Structured**:
1. Embodied Intelligence (4 concepts, M1-C1)
2. Sensor Fundamentals (5-6 concepts, M1-C2)
3. Actuator Fundamentals (5-6 concepts, M1-C3)
4. Physical Constraints & Integration (4-5 concepts, M1-C4, M1-C5)

**Physical Grounding Compliance**: Every chapter explicitly covers sensors, actuators, AND latency/noise/physics as required by spec clarifications.

**Time Allocation**: 2 weeks (14-16 hours), matching course-overview.md

**Cognitive Load Validation**: All chapters pass concept limits (max 6 per chapter).

## Outcome

- Impact: Created foundational module specification that establishes physical grounding framework for entire course
- Tests: N/A (specification document)
- Files: 1 created (specs/curriculum/module-1.md)
- Next prompts: Module 2 specification (ROS 2 Fundamentals), Module 3 specification (Digital Twin & Simulation)
- Reflection: Module 1 is unique in that it IS the physical grounding content; other modules will apply this framework

## Evaluation notes (flywheel)

- Failure modes observed: None - task was well-defined with clear constraints
- Graders run and results (PASS/FAIL): Manual review needed - specification completeness
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate physical grounding coverage with content authors

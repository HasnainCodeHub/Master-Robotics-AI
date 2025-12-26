---
id: 0002
title: Course Overview Specification Created
stage: spec
date: 2024-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: curriculum-architect
command: Task 1.1 - Create course-overview.md
labels: ["curriculum", "course-overview", "specification", "physical-ai"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/curriculum/course-overview.md
tests:
  - N/A (specification document)
---

## Prompt

Execute Task 1.1 - Course Overview Specification for the Physical AI & Humanoid Robotics textbook.

Context provided:
- Target Audience: Mixed (senior undergrad, graduate, industry professionals)
- Prerequisites: Python (intermediate), basic robotics, basic ROS
- Platform Mastery Goal: Production-ready skills
- Capstone Environment: Simulation-based with conceptual sim-to-real
- Mathematical Depth: Moderate with applied equations
- Physical Grounding: Sensors + actuators + latency/noise/physics per chapter
- Non-Goals: Advanced control theory, low-level motor control, custom hardware design, training foundation models

Required Modules:
1. Physical AI foundations
2. ROS 2
3. Digital Twin & Simulation
4. NVIDIA Isaac
5. Vision-Language-Action
6. Capstone (integration)

Task: Create specs/curriculum/course-overview.md with course goal, target learner profile, explicit prerequisites, learning philosophy, high-level module list with capability outcomes, and rationale for module ordering.

## Response snapshot

Created comprehensive course-overview.md (specs/curriculum/course-overview.md) containing:

1. **Course Goal**: Single clear statement about designing, implementing, and deploying autonomous robotic systems in simulation with sim-to-real understanding.

2. **Target Learner Profile**: Three audience segments (senior undergrads, graduate students, industry professionals) with common characteristics and explicit "not for" list.

3. **Explicit Prerequisites**: Table format with domain, minimum requirement, and self-assessment questions for Python, robotics concepts, ROS, linear algebra, and Linux CLI. Also lists "assumed but not taught" and "not required" items.

4. **Learning Philosophy**: Physical AI as embodied intelligence approach with four pillars (perception is noisy, action has consequences, world is uncertain, simulation is a tool). Pedagogical approach includes capability-first design, progressive complexity, physical grounding mandate, and simulation-to-concept transfer.

5. **High-Level Module List**: Six modules (M1-M5 + Capstone) each with:
   - Duration
   - Dependencies
   - Module goal
   - Capability outcomes table (ID, capability, verification)
   - Key concepts introduced
   - Physical grounding notes

6. **Rationale for Module Ordering**: Dependency chain diagram, pedagogical justification for each module's position, and rejected alternatives with reasoning.

## Outcome

- Impact: Created foundational curriculum specification that will guide all subsequent module and chapter development
- Tests: N/A (specification document - validated against clarified requirements)
- Files: 1 created (specs/curriculum/course-overview.md)
- Next prompts: Create detailed module specifications (module-01-spec.md through module-05-spec.md), then capstone-spec.md
- Reflection: The course overview establishes clear capability-based objectives and explicit prerequisite chains. The physical grounding requirement is woven throughout as a design constraint, not an afterthought.

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward specification task with clear inputs
- Graders run and results (PASS/FAIL): Manual validation against spec.md requirements - PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Apply same capability-outcome table format to detailed module specs

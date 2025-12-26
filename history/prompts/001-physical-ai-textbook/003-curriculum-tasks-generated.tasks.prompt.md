---
id: 003
title: Curriculum Tasks Generated
stage: tasks
date: 2024-12-24
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: system
command: /sp.tasks
labels: ["tasks", "curriculum", "phase-1", "planning"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/tasks.md
tests:
  - N/A (task definition document)
---

## Prompt

/sp.tasks

Scope: PHASE 1 - Curriculum Intelligence Design
Objective: Design and lock the learning architecture for the Physical AI & Humanoid Robotics course.
NO textbook content or code implementation is allowed in this phase.

Task Groups:
1. Course-Level Structure (T001-T002)
2. Module-Level Curriculum Specs (T003-T008)
3. Prerequisite & Progression Validation (T009-T010)
4. Curriculum Freeze Declaration (T011)

## Response snapshot

Generated 11 tasks for Phase 1 - Curriculum Intelligence Design:

**Phase 1.1 - Course-Level Structure (2 tasks)**:
- T001: Create course overview spec
- T002: Validate course cognitive load

**Phase 1.2 - Module Specs (6 tasks, all parallelizable)**:
- T003: Module 1 - Physical AI Foundations
- T004: Module 2 - ROS 2
- T005: Module 3 - Simulation & Digital Twin
- T006: Module 4 - NVIDIA Isaac
- T007: Module 5 - Vision-Language-Action
- T008: Capstone project

**Phase 1.3 - Validation (2 tasks)**:
- T009: Cross-module prerequisite audit
- T010: Progressive autonomy validation

**Phase 1.4 - Freeze (1 task)**:
- T011: Curriculum freeze declaration

**Parallelization**: T003-T008 can all run in parallel (6 agents)

**Artifacts to create**: 9 files under /specs/curriculum/

## Outcome

- Impact: HIGH - Task breakdown for curriculum phase complete
- Tests: N/A (task definition document)
- Files: 1 file created (tasks.md)
- Next prompts: /sp.implement to execute tasks, or manually execute T001 first
- Reflection: Tasks correctly scope Phase 1 to curriculum-only work

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks have IDs, paths)
- Prompt variant (if applicable): User-provided Phase 1 scope
- Next experiment (smallest change to try): Execute T001 to start curriculum design

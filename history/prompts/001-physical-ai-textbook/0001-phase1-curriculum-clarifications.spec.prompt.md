---
id: "0001"
title: "Phase 1 Curriculum Intelligence Clarifications"
stage: spec
date: 2024-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: system
command: /sp.clarify
labels: ["curriculum", "phase1", "clarification", "physical-ai"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/spec.md
tests:
  - null
---

## Prompt

/sp.clarify

Scope: PHASE 1 — Curriculum Intelligence Design

Before executing Phase 1 curriculum tasks, clarify the following to eliminate
ambiguity and prevent downstream rework.

CLARIFICATION DOMAIN 1 — TARGET LEARNER & ENTRY LEVEL
- What is the assumed minimum background of the learner?
- Is this course intended for senior undergrad, graduate, industry, or mixed?
- Should the curriculum assume access to RTX workstation, Jetson, or simulation-only?

CLARIFICATION DOMAIN 2 — DEPTH VS BREADTH
- For ROS 2, Gazebo, Isaac, Unity: conceptual, hands-on, or production-ready?
- How should Unity be treated?

CLARIFICATION DOMAIN 3 — PHYSICAL AI EMPHASIS
- What qualifies as "physical grounding" per chapter?
- Must every chapter include simulated example and/or real-hardware discussion?

CLARIFICATION DOMAIN 4 — CAPSTONE EXPECTATIONS
- Fully simulated, sim + conceptual sim-to-real, or Jetson hardware?
- Step-guided, open-ended, or behavior-evaluated?

CLARIFICATION DOMAIN 5 — CURRICULUM CONSTRAINTS
- Fixed course duration? Module-to-week mapping?
- Assessments in Phase 1 or just learning objectives?

CLARIFICATION DOMAIN 6 — NON-GOALS
- Explicit exclusions: advanced control theory, low-level motor control, custom hardware, training models?
- Mathematical depth: light, moderate, or heavy?

## Response snapshot

Executed structured clarification workflow for Phase 1 Curriculum Intelligence Design. Asked 5 high-impact questions covering:

1. **Target Audience Type** → Mixed audience (senior undergrad, graduate, industry professionals)
2. **Platform Mastery Level** → Production-ready skills
3. **Capstone Execution Environment** → Simulated + conceptual sim-to-real discussion
4. **Mathematical Depth** → Moderate with applied equations
5. **Physical Grounding Definition** → All three: sensors + actuators + latency/noise/physics

All answers integrated into spec.md under Clarifications and Assumptions sections. Explicit curriculum non-goals added to Out of Scope section.

## Outcome

- Impact: High — resolved 6 critical curriculum ambiguity categories before planning phase
- Tests: N/A — clarification phase, no code tests
- Files: 1 file modified (spec.md)
- Next prompts: /sp.plan to begin implementation planning
- Reflection: Production-ready skills + simulation-only capstone is ambitious but achievable; mixed audience requires careful scaffolding

## Evaluation notes (flywheel)

- Failure modes observed: None — user provided clear answers to all questions
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard /sp.clarify
- Next experiment (smallest change to try): Consider asking about assessment design earlier if course grading is critical

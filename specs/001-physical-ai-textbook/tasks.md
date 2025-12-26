# Tasks: Physical AI & Humanoid Robotics - AI-Native Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/
**Scope**: PHASE 1 - Curriculum Intelligence Design

**Important**: This phase focuses on curriculum architecture ONLY. NO textbook content or code implementation is allowed until Phase 1 is complete and locked.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[CUR]**: Curriculum task (owned by curriculum-architect)
- Include exact file paths in descriptions

## Path Conventions

All curriculum specs live under `/specs/curriculum/`

---

## Phase 1.1: Course-Level Structure

**Purpose**: Define the overall learning system architecture

**Owner**: curriculum-architect

- [ ] T001 Create course overview spec in specs/curriculum/course-overview.md
  - Course goal and target learner profile
  - Assumed prior knowledge (explicit prerequisites)
  - Learning philosophy (Physical AI + embodied intelligence)
  - Module list with high-level capability outcomes
  - Explanation of why module order matters

- [ ] T002 Validate course cognitive load in specs/curriculum/course-overview.md
  - Modules show increasing autonomy and complexity
  - No module introduces more than one major capability leap
  - Capstone enabled by prior modules (not carrying instructional burden)

**Acceptance Criteria (T001-T002)**:
- All prerequisites are explicitly stated
- No module assumes unstated knowledge
- Capabilities stated as "students can DO…", not "students will learn…"

**Checkpoint**: Course-level architecture validated

---

## Phase 1.2: Module Curriculum Specs

**Purpose**: Define learning objectives and chapter structure for each module

**Owner**: curriculum-architect

### Module 1: Physical AI Foundations

- [ ] T003 [P] [CUR] Create Module 1 spec in specs/curriculum/module-1-physical-ai.md
  - Module goal
  - Capabilities gained by end of module
  - Chapter list with objectives
  - Physical grounding requirement for each chapter
  - Simulation or conceptual tie-in

**Acceptance Criteria (T003)**:
- Each chapter introduces 1-2 new capabilities
- Physical AI concept is explicitly grounded (sensor, actuator, constraint)

---

### Module 2: ROS 2 - The Robotic Nervous System

- [ ] T004 [P] [CUR] Create Module 2 spec in specs/curriculum/module-2-ros2.md
  - ROS 2 concepts sequence (nodes, topics, services, actions)
  - Python (rclpy) focus
  - Dependency on Module 1 concepts
  - Hands-on capability outcomes
  - URDF for humanoid robots

**Acceptance Criteria (T004)**:
- ROS concepts are capability-driven (not API lists)
- Dependencies on Module 1 are explicit

---

### Module 3: Simulation & Digital Twin

- [ ] T005 [P] [CUR] Create Module 3 spec in specs/curriculum/module-3-simulation.md
  - Gazebo and Unity roles
  - Physics concepts introduced
  - Sensor simulation expectations (LiDAR, depth cameras, IMUs)
  - Link to ROS 2 integration

**Acceptance Criteria (T005)**:
- Simulation is framed as learning tool, not visualization only
- Sensor models map to Physical AI foundations

---

### Module 4: NVIDIA Isaac Platform

- [ ] T006 [P] [CUR] Create Module 4 spec in specs/curriculum/module-4-nvidia-isaac.md
  - Isaac Sim role
  - Isaac ROS and VSLAM
  - Navigation capabilities
  - Sim-to-real transfer concepts

**Acceptance Criteria (T006)**:
- Capstone prerequisites fully enabled by this module
- Clear connection to prior modules

---

### Module 5: Vision-Language-Action (VLA)

- [ ] T007 [P] [CUR] Create Module 5 spec in specs/curriculum/module-5-vla.md
  - Whisper voice input
  - LLM-based planning
  - Translating natural language into ROS actions
  - VLA pipeline architecture

**Acceptance Criteria (T007)**:
- VLA is treated as system orchestration, not "LLM magic"
- Clear grounding in physical constraints

---

### Capstone: Integrated Humanoid Project

- [ ] T008 [P] [CUR] Create Capstone spec in specs/curriculum/capstone.md
  - Voice command → path planning → navigation → object identification → manipulation
  - Integration of all 5 modules
  - Tradeoff decisions required from learners
  - Success criteria for capstone completion

**Acceptance Criteria (T008)**:
- All capabilities from Modules 1-5 are utilized
- Requires design decisions, not just following steps

**Checkpoint**: All 6 module specs created

---

## Phase 1.3: Prerequisite & Progression Validation

**Purpose**: Audit and validate learning progression

**Owner**: curriculum-architect

- [ ] T009 Perform cross-module prerequisite audit
  - Review all module specs for unstated concept introductions
  - Document all dependencies in specs/curriculum/prerequisite-map.md
  - Verify modules form linear, explainable progression

**Acceptance Criteria (T009)**:
- No chapter introduces unstated concepts
- All dependencies are documented
- Modules form linear, explainable progression

---

- [ ] T010 Validate progressive autonomy across modules
  - Early modules are guided with explicit steps
  - Later modules require design decisions
  - Capstone requires tradeoff reasoning

**Acceptance Criteria (T010)**:
- Autonomy level documented for each module
- Clear progression from guided → independent

**Checkpoint**: Curriculum progression validated

---

## Phase 1.4: Curriculum Freeze

**Purpose**: Lock curriculum specs for downstream phases

**Owner**: chief-orchestrator

- [ ] T011 Create curriculum freeze declaration in specs/curriculum/CURRICULUM-LOCK.md
  - List all curriculum specs created
  - Confirm no unresolved ambiguity
  - Document approval from curriculum-architect
  - Record freeze date and version

**Acceptance Criteria (T011)**:
- All curriculum specs exist and are complete
- No [NEEDS CLARIFICATION] markers remain
- Curriculum architect explicitly approves readiness

**Checkpoint**: PHASE 1 COMPLETE - Curriculum locked

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1.1 (Course-Level)
    │
    ▼
Phase 1.2 (Module Specs) ←── All module specs can run in PARALLEL
    │
    ▼
Phase 1.3 (Validation)
    │
    ▼
Phase 1.4 (Freeze)
```

### Task Dependencies

| Task | Depends On | Can Parallel With |
|------|------------|-------------------|
| T001 | None | - |
| T002 | T001 | - |
| T003-T008 | T002 | Each other (all [P]) |
| T009 | T003-T008 | - |
| T010 | T009 | - |
| T011 | T010 | - |

### Parallel Opportunities

**Maximum parallelization during Phase 1.2**:
```
T003 (Module 1) ──┐
T004 (Module 2) ──┤
T005 (Module 3) ──┼── All 6 can run in parallel
T006 (Module 4) ──┤
T007 (Module 5) ──┤
T008 (Capstone) ──┘
```

---

## Implementation Strategy

### Sequential Execution (Single Agent)

1. Complete T001-T002 (Course-Level)
2. Complete T003-T008 sequentially (Module Specs)
3. Complete T009-T010 (Validation)
4. Complete T011 (Freeze)

### Parallel Execution (Multiple Agents)

1. Complete T001-T002 (Sequential - sets foundation)
2. Launch T003-T008 in parallel (6 agents can work simultaneously)
3. Wait for all module specs to complete
4. Complete T009-T010 (Sequential - cross-module audit)
5. Complete T011 (Final freeze)

---

## Phase 1 Completion Conditions

Phase 1 is complete when:

- [ ] All curriculum specs exist under `/specs/curriculum/`
- [ ] Course overview defines target learner and prerequisites
- [ ] All 5 modules + capstone have complete specs
- [ ] Prerequisite map documents all dependencies
- [ ] Progressive autonomy is validated
- [ ] Curriculum freeze declaration is signed
- [ ] NO textbook content has been written
- [ ] NO code has been implemented

---

## Artifacts to Create

| Artifact | Task | Path |
|----------|------|------|
| Course Overview | T001 | specs/curriculum/course-overview.md |
| Module 1 Spec | T003 | specs/curriculum/module-1-physical-ai.md |
| Module 2 Spec | T004 | specs/curriculum/module-2-ros2.md |
| Module 3 Spec | T005 | specs/curriculum/module-3-simulation.md |
| Module 4 Spec | T006 | specs/curriculum/module-4-nvidia-isaac.md |
| Module 5 Spec | T007 | specs/curriculum/module-5-vla.md |
| Capstone Spec | T008 | specs/curriculum/capstone.md |
| Prerequisite Map | T009 | specs/curriculum/prerequisite-map.md |
| Curriculum Lock | T011 | specs/curriculum/CURRICULUM-LOCK.md |

---

## Next Phase (After Curriculum Lock)

Once Phase 1 is complete and curriculum is locked:

1. **Phase 2**: Textbook Content Creation (book-content-writer agent)
2. **Phase 3**: RAG System Design & Backend (backend-rag agent)
3. **Phase 4**: Frontend Platform & AI Embedding (frontend-platform agent)
4. **Phase 5**: Authentication & Personalization (identity-reasoning-guardian agent)
5. **Phase 6**: Urdu Translation (frontend-platform agent)
6. **Phase 7**: Deployment & Release (devops-github agent)

---

## Notes

- All Phase 1 tasks are curriculum design, NOT implementation
- The [CUR] tag indicates curriculum-architect ownership
- [P] tasks can run in parallel if multiple agents are available
- Curriculum lock blocks all downstream phases
- Constitution Principle II (Curriculum Authority) governs this phase

---
id: 0007
title: Capstone Integrated Humanoid System Specification
stage: spec
date: 2024-12-24
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: curriculum-architect
command: Create Capstone specification
labels: ["curriculum", "capstone", "integration", "physical-ai"]
links:
  spec: /specs/curriculum/capstone.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/curriculum/capstone.md
tests:
  - N/A (specification document)
---

## Prompt

You are the curriculum-architect agent executing Phase 1 of the Physical AI & Humanoid Robotics curriculum design.

**Task**: Create Capstone: Integrated Humanoid System specification at `/specs/curriculum/capstone.md`

**Context**:
- Modules 1-5 exist (or are being created) in `/specs/curriculum/`
- Capstone was defined in course-overview.md with capability outcomes
- Use the same format and structure as existing module specs

**Skills to Apply**:
- chapter-structuring
- pedagogy-mapping

**Requirements for Capstone**:
Based on course-overview.md, the Capstone must:
- Integrate all learned capabilities into a coherent humanoid robot system
- Receive voice commands, plan tasks, navigate environments, identify objects, and perform manipulation
- All within simulation with explicit sim-to-real discussion

**Capabilities from course-overview.md**:
| ID | Capability | Verification |
|----|------------|--------------|
| CAP-C1 | Students can architect a complete robotic system integrating perception, planning, and action | Produce a system architecture diagram showing all components and data flows |
| CAP-C2 | Students can implement voice-commanded navigation (NL to path planning to execution) | Demonstrate robot navigating to spoken destination with obstacle avoidance |
| CAP-C3 | Students can implement voice-commanded object identification using VLM | Demonstrate robot identifying objects in scene based on spoken description |
| CAP-C4 | Students can implement voice-commanded manipulation using task planning | Demonstrate robot picking/placing objects based on spoken commands |
| CAP-C5 | Students can identify and document the sim-to-real gaps in their complete system | Produce a transfer analysis document identifying gaps and mitigation strategies |
| CAP-C6 | Students can evaluate their system against defined performance criteria | Run evaluation scenarios and report quantitative metrics (success rate, latency) |

**Integration Points from course-overview.md**:
- Module 1: Physical constraints awareness in all design decisions
- Module 2: ROS 2 as the integration backbone
- Module 3: Gazebo or Isaac Sim as execution environment
- Module 4: Isaac perception and VSLAM for robust sensing
- Module 5: VLA pipeline for natural language interface

**Capstone Deliverables**:
1. Working simulation demonstrating all five capabilities
2. System architecture documentation
3. Sim-to-real transfer analysis
4. Performance evaluation report

**Critical Constraints**:
1. Capstone is NOT where students learn new concepts - it's integration only
2. Requires tradeoff reasoning and design decisions, not just following steps
3. 3-week duration for integration and evaluation
4. All 5 prior modules must be utilized
5. Success criteria must be measurable
6. Physical grounding mandate applies to all design decisions

## Response snapshot

Created comprehensive capstone specification at `/specs/curriculum/capstone.md` with the following structure:

**Capstone Goal**: Integration and evaluation of complete Physical AI stack - NO new concepts introduced.

**Prerequisites**: ALL prior modules required with explicit capability mapping from M1-C1 through M5-C6.

**Capabilities Gained** (CAP-C1 through CAP-C6):
- System architecture with physical grounding
- Voice-commanded navigation
- Voice-commanded object identification
- Voice-commanded manipulation
- Sim-to-real transfer analysis
- Quantitative performance evaluation

**Time Allocation** (3 weeks):
- Week 15: System Architecture (design before implementation)
- Week 16: Implementation (incremental integration)
- Week 17: Evaluation and Transfer Analysis

**Project Structure**:
- Phase 1: Architecture document with timing budgets and physical constraints
- Phase 2: Working simulation with all five capability demonstrations
- Phase 3: Performance evaluation report and transfer analysis document

**Integration Requirements**: Detailed checklists for each prior module showing exactly how capabilities apply.

**Assessment Strategy**: Formative checkpoints at each phase; summative assessment with 4 graded deliverables (architecture 20%, demo 40%, evaluation 20%, transfer analysis 20%).

**Cognitive Load Validation**: PASS - Zero new concepts introduced; all complexity is integration, not learning.

**Physical Grounding Compliance**: Mandatory in all deliverables with explicit checklists.

**Success/Failure Conditions**: Clear criteria distinguishing integration mastery from component-only learning.

## Outcome

- Impact: Created complete capstone specification enabling structured 3-week integration project
- Tests: N/A (specification document)
- Files: 1 created (specs/curriculum/capstone.md, ~850 lines)
- Next prompts: Create Module 5 specification; Review all module specifications for consistency; Create chapter-level content outlines
- Reflection: The capstone design enforces the critical principle that NO new concepts are introduced - the capstone validates integration mastery of all prior learning. The phased structure (architecture > implementation > evaluation) prevents students from jumping into coding without design, which is a common failure mode.

## Evaluation notes (flywheel)

- Failure modes observed: None - followed existing module specification format closely
- Graders run and results (PASS/FAIL): Manual review - PASS (follows course-overview.md capstone definition)
- Prompt variant (if applicable): N/A
- Next experiment: Verify capstone prerequisites align exactly with Module 5 capabilities once Module 5 spec is created

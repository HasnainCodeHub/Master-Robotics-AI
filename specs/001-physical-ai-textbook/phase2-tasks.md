# Tasks: Physical AI & Humanoid Robotics - AI-Native Textbook

**Input**: Locked curriculum specifications from `/specs/curriculum/`
**Prerequisites**: Phase 1 COMPLETE, CURRICULUM-FREEZE.md signed
**Scope**: PHASE 2 - Textbook Content Creation

**Important**: This phase focuses on Docusaurus-based textbook content ONLY. NO backend, RAG, authentication, personalization, or deployment logic is allowed.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[CONTENT]**: Content authoring task (owned by book-content-writer)
- **[INFRA]**: Infrastructure task (owned by project-bootstrap or frontend-platform)
- **[AUDIT]**: Validation task (owned by curriculum-architect)
- Include exact file paths in descriptions

## Path Conventions

- Docusaurus project: `/docs/`
- Curriculum specs: `/specs/curriculum/` (READ-ONLY in this phase)
- Phase artifacts: `/specs/001-physical-ai-textbook/`

---

## TASK GROUP 2.0 — PHASE ENTRY VALIDATION

**Purpose**: Confirm Phase 1 prerequisites before proceeding

**Owner**: chief-orchestrator

---

### T200 — Phase 1 Completion Check

**Owner**: chief-orchestrator
**Dependencies**: Phase 1 completed

**Description**:
Confirm Phase 1 (Curriculum Intelligence Design) is fully completed and frozen.

**Validation Checklist**:
- [ ] `/specs/curriculum/CURRICULUM-FREEZE.md` exists with OFFICIAL status
- [ ] All 8 curriculum specs have status `LOCKED (Phase 1 Complete)`
- [ ] No `[NEEDS CLARIFICATION]` markers remain in any curriculum spec
- [ ] Prerequisite map validates no ordering violations

**Acceptance Criteria**:
- All curriculum specs exist under `/specs/curriculum/`
- Curriculum is explicitly marked LOCKED
- No open ambiguities remain
- chief-orchestrator signs off on Phase 2 authorization

**Artifacts Verified**:
| Artifact | Path | Required Status |
|----------|------|-----------------|
| Course Overview | `/specs/curriculum/course-overview.md` | LOCKED |
| Module 1 | `/specs/curriculum/module-1.md` | LOCKED |
| Module 2 | `/specs/curriculum/module-2.md` | LOCKED |
| Module 3 | `/specs/curriculum/module-3.md` | LOCKED |
| Module 4 | `/specs/curriculum/module-4.md` | LOCKED |
| Module 5 | `/specs/curriculum/module-5.md` | LOCKED |
| Capstone | `/specs/curriculum/capstone.md` | LOCKED |
| Prerequisite Map | `/specs/curriculum/prerequisite-map.md` | Validated |
| Freeze Declaration | `/specs/curriculum/CURRICULUM-FREEZE.md` | OFFICIAL |

---

## TASK GROUP 2.1 — DOCUSAURUS PLATFORM INITIALIZATION

**Purpose**: Create the Docusaurus documentation site for the textbook

**Owner**: project-bootstrap, frontend-platform

---

### T201 — Initialize Docusaurus Project

**Owner**: project-bootstrap
**Skills Used**: project-scaffolding
**Dependencies**: T200

**Description**:
Create a Docusaurus documentation site for the Physical AI & Humanoid Robotics textbook.

**Actions**:
1. Initialize Docusaurus project in `/docs/` directory
2. Configure `docusaurus.config.js` with:
   - Title: "Physical AI & Humanoid Robotics"
   - Tagline: "An AI-Native Textbook"
   - Base URL configuration for GitHub Pages
   - Search plugin (local search)
3. Set up basic directory structure for modules
4. Configure MDX support for interactive content
5. Add placeholder pages for navigation testing

**Deliverables**:
- Docusaurus project initialized at `/docs/`
- `package.json` with required dependencies
- `docusaurus.config.js` with base configuration
- `sidebars.js` with placeholder structure
- Placeholder `index.md` landing page

**Acceptance Criteria**:
- `npm install` completes without errors
- `npm run start` launches development server
- `npm run build` produces static site
- No real textbook content yet (placeholders only)
- Clean, minimal setup following Docusaurus best practices

**Directory Structure Created**:
```
/docs/
├── docusaurus.config.js
├── package.json
├── sidebars.js
├── src/
│   └── pages/
│       └── index.js
├── docs/
│   ├── intro.md (placeholder)
│   ├── module-1/
│   ├── module-2/
│   ├── module-3/
│   ├── module-4/
│   ├── module-5/
│   └── capstone/
└── static/
    └── img/
```

---

### T202 — Configure Global Navigation & Sidebar

**Owner**: frontend-platform
**Skills Used**: chapter-structuring
**Dependencies**: T201

**Description**:
Configure sidebar and navigation to reflect curriculum module order exactly as specified in locked curriculum specs.

**Actions**:
1. Read module structure from each curriculum spec:
   - `/specs/curriculum/module-1.md` - 4 chapters
   - `/specs/curriculum/module-2.md` - 6 chapters
   - `/specs/curriculum/module-3.md` - 6 chapters
   - `/specs/curriculum/module-4.md` - 6 chapters
   - `/specs/curriculum/module-5.md` - 6 chapters
   - `/specs/curriculum/capstone.md` - 3 phases
2. Create `sidebars.js` with exact chapter titles from specs
3. Configure navigation header with module links
4. Add footer with course information

**Deliverables**:
- Updated `sidebars.js` with complete navigation tree
- Placeholder `.md` files for all chapters (empty with title only)
- Navigation header configured in `docusaurus.config.js`

**Acceptance Criteria**:
- Module order matches curriculum specs exactly: M1 → M2 → M3 → M4 → M5 → Capstone
- Chapter order within each module matches spec exactly
- Naming matches curriculum language precisely
- No missing or extra sections
- Sidebar collapses/expands appropriately

**Sidebar Structure** (from curriculum specs):
```
Physical AI & Humanoid Robotics
├── Module 1: Physical AI Foundations (4 chapters)
│   ├── Ch 1: Embodied Intelligence
│   ├── Ch 2: Sensor Fundamentals
│   ├── Ch 3: Actuator Fundamentals
│   └── Ch 4: Physical Constraints
├── Module 2: ROS 2 Fundamentals (6 chapters)
│   ├── Ch 1: ROS 2 Architecture
│   ├── Ch 2: Topics & Pub/Sub
│   ├── Ch 3: Services
│   ├── Ch 4: Actions
│   ├── Ch 5: URDF & Robot Description
│   └── Ch 6: Integration Patterns
├── Module 3: Digital Twin & Simulation (6 chapters)
│   ├── Ch 1: Simulation Fundamentals
│   ├── Ch 2: Gazebo World Building
│   ├── Ch 3: Simulated Sensors
│   ├── Ch 4: Simulated Actuators
│   ├── Ch 5: Unity Integration
│   └── Ch 6: Reality Gap
├── Module 4: NVIDIA Isaac Ecosystem (6 chapters)
│   ├── Ch 1: Isaac Sim Foundations
│   ├── Ch 2: Scene Composition & Sensors
│   ├── Ch 3: Isaac ROS Integration
│   ├── Ch 4: Visual SLAM
│   ├── Ch 5: Domain Randomization
│   └── Ch 6: Sim-to-Real Transfer
├── Module 5: Vision-Language-Action (6 chapters)
│   ├── Ch 1: Speech Recognition
│   ├── Ch 2: LLM Task Planning
│   ├── Ch 3: Grounding & Action Mapping
│   ├── Ch 4: Vision-Language Models
│   ├── Ch 5: Safety Constraints
│   └── Ch 6: Failure Handling
└── Capstone: Integrated Humanoid (3 phases)
    ├── Phase 1: System Architecture
    ├── Phase 2: Implementation
    └── Phase 3: Evaluation & Transfer
```

---

## TASK GROUP 2.2 — MODULE CONTENT AUTHORING

**Purpose**: Write all textbook content strictly following locked curriculum specifications

**Owner**: book-content-writer

**Content Requirements (ALL chapters)**:
- Problem motivation before solution (Constitution Principle VI)
- Clear learning objectives matching curriculum spec
- Physical grounding elements (sensors, actuators, latency/noise/physics)
- Concept explanation with realistic examples
- Summary and key takeaways
- RAG-friendly structure with clear section headers

---

### T203 — Write Module 1 Content (Physical AI Foundations)

**Owner**: book-content-writer
**Skills Used**: technical-writing, diagram-explainer
**Dependencies**: T202

**Description**:
Write all 4 chapters of Module 1 strictly following `/specs/curriculum/module-1.md`.

**Source Specification**: `/specs/curriculum/module-1.md`

**Chapters to Write**:

| Chapter | Title | Key Concepts | Word Estimate |
|---------|-------|--------------|---------------|
| 1.1 | Embodied Intelligence | Embodied vs disembodied AI, perception-action loop, reality gap | 2000-3000 |
| 1.2 | Sensor Fundamentals | Proprioceptive/exteroceptive, noise models, failure modes | 3000-4000 |
| 1.3 | Actuator Fundamentals | Motor types, torque-speed curves, safety constraints | 2500-3500 |
| 1.4 | Physical Constraints | Latency budgets, uncertainty propagation, real-time requirements | 2500-3500 |

**Physical Grounding Requirements** (from module spec):
- Chapter 1: Core focus on latency as physical constraint, noise as measurement reality
- Chapter 2: Core focus on camera, LiDAR, IMU specifications and noise characteristics
- Chapter 3: Core focus on motor types, response characteristics, safety
- Chapter 4: Core focus on timing budgets and uncertainty propagation

**Deliverables**:
- `/docs/docs/module-1/chapter-1-embodied-intelligence.md`
- `/docs/docs/module-1/chapter-2-sensor-fundamentals.md`
- `/docs/docs/module-1/chapter-3-actuator-fundamentals.md`
- `/docs/docs/module-1/chapter-4-physical-constraints.md`
- `/docs/docs/module-1/index.md` (module introduction)

**Acceptance Criteria**:
- Each chapter has exactly 1.1-1.5x the learning objectives from spec
- No concepts introduced that aren't in curriculum spec
- No unstated prerequisites
- Physical AI emphasis throughout (no "software-only" framing)
- Sensor datasheet analysis exercises use realistic specifications
- Actuator specifications reference real-world motor parameters
- Concept count per chapter ≤ curriculum spec limit (5-6 concepts)

**Content Quality Checklist** (per chapter):
- [ ] Opens with problem motivation scenario
- [ ] Learning objectives match curriculum spec
- [ ] Physical grounding elements explicitly addressed
- [ ] Examples use realistic numerical values (noise in mm, latency in ms)
- [ ] Summary restates key capabilities gained
- [ ] Self-assessment questions verify capability, not just recall
- [ ] No ROS 2 content (belongs in Module 2)

---

### T204 — Write Module 2 Content (ROS 2 Fundamentals)

**Owner**: book-content-writer
**Skills Used**: technical-writing, diagram-explainer
**Dependencies**: T203

**Description**:
Write all 6 chapters of Module 2 strictly following `/specs/curriculum/module-2.md`.

**Source Specification**: `/specs/curriculum/module-2.md`

**Chapters to Write**:

| Chapter | Title | Key Concepts | Word Estimate |
|---------|-------|--------------|---------------|
| 2.1 | ROS 2 Architecture | DDS, QoS, workspaces, introspection tools | 2500-3500 |
| 2.2 | Topics & Publishers/Subscribers | Pub/sub pattern, message types, callbacks | 3000-4000 |
| 2.3 | Services | Request/response, timeout handling, sync vs async | 2000-3000 |
| 2.4 | Actions | Goal/feedback/result, cancellation, state machine | 3000-4000 |
| 2.5 | URDF & Robot Description | Links, joints, tf2, visualization | 2500-3500 |
| 2.6 | Integration Patterns | Multi-pattern nodes, lifecycle, launch files | 2500-3500 |

**Physical Grounding Requirements** (from module spec):
- All chapters reference M1 sensor/actuator concepts
- QoS settings justified by physical sensor characteristics
- Timeout values based on physical operation durations
- Timer rates matching physical sensor rates

**Deliverables**:
- `/docs/docs/module-2/chapter-1-ros2-architecture.md`
- `/docs/docs/module-2/chapter-2-topics-pubsub.md`
- `/docs/docs/module-2/chapter-3-services.md`
- `/docs/docs/module-2/chapter-4-actions.md`
- `/docs/docs/module-2/chapter-5-urdf-robot-description.md`
- `/docs/docs/module-2/chapter-6-integration-patterns.md`
- `/docs/docs/module-2/index.md` (module introduction)

**Acceptance Criteria**:
- Python only (rclpy) - no C++ examples
- Production patterns, not minimal tutorials
- QoS configurations explicitly reference M1 sensor/actuator specs
- Examples connect to humanoid robotics use cases
- No API-dump writing; capability-driven explanations
- Code examples are complete and runnable
- Error handling included (not just happy path)

**Code Quality Requirements**:
- All Python code uses type hints
- Docstrings explain physical grounding of design choices
- Code follows ROS 2 Python style guidelines
- Examples include realistic timing values (Hz, ms, s)

---

### T205 — Write Module 3 Content (Simulation & Digital Twin)

**Owner**: book-content-writer
**Skills Used**: technical-writing, diagram-explainer
**Dependencies**: T204

**Description**:
Write all 6 chapters of Module 3 strictly following `/specs/curriculum/module-3.md`.

**Source Specification**: `/specs/curriculum/module-3.md`

**Chapters to Write**:

| Chapter | Title | Key Concepts | Word Estimate |
|---------|-------|--------------|---------------|
| 3.1 | Simulation Fundamentals | Physics engines, RTF, simulation loop | 2500-3500 |
| 3.2 | Gazebo World Building | SDF, physics properties, lighting | 3000-4000 |
| 3.3 | Simulated Sensors | Camera, LiDAR, IMU plugins, noise models | 3500-4500 |
| 3.4 | Simulated Actuators | Joint controllers, PID, friction | 2500-3500 |
| 3.5 | Unity Integration | ROS-TCP-Connector, URDF import | 2500-3500 |
| 3.6 | Reality Gap | Gap taxonomy, validation methodology | 2500-3500 |

**Physical Grounding Requirements** (from module spec):
- Sensor noise parameters derived from M1 datasheet analysis
- Actuator dynamics reference M1 specifications
- Explicit comparison: what physics engines model vs simplify
- Reality gap callout boxes in sensor/actuator chapters

**Deliverables**:
- `/docs/docs/module-3/chapter-1-simulation-fundamentals.md`
- `/docs/docs/module-3/chapter-2-gazebo-world-building.md`
- `/docs/docs/module-3/chapter-3-simulated-sensors.md`
- `/docs/docs/module-3/chapter-4-simulated-actuators.md`
- `/docs/docs/module-3/chapter-5-unity-integration.md`
- `/docs/docs/module-3/chapter-6-reality-gap.md`
- `/docs/docs/module-3/index.md` (module introduction)

**Acceptance Criteria**:
- Gazebo Harmonic version used (not classic Gazebo)
- ROS 2 integration via ros_gz bridge
- Physics concepts clearly explained with simulations framed as learning tools
- Sensor models explicitly map to M1 Physical AI foundations
- Reality gap explicitly discussed in each relevant chapter
- Configuration files provided for all exercises

**Platform Requirements**:
- Gazebo: Primary platform for physics simulation
- Unity: Alternative for visualization-intensive scenarios
- Decision framework for when to use each platform

---

### T206 — Write Module 4 Content (NVIDIA Isaac Ecosystem)

**Owner**: book-content-writer
**Skills Used**: technical-writing, diagram-explainer
**Dependencies**: T205

**Description**:
Write all 6 chapters of Module 4 strictly following `/specs/curriculum/module-4.md`.

**Source Specification**: `/specs/curriculum/module-4.md`

**Chapters to Write**:

| Chapter | Title | Key Concepts | Word Estimate |
|---------|-------|--------------|---------------|
| 4.1 | Isaac Sim Foundations | Omniverse, USD, PhysX 5 | 3000-4000 |
| 4.2 | Scene Composition & Sensors | USD composition, RTX sensors | 3500-4500 |
| 4.3 | Isaac ROS Integration | Bridge, NITROS, perception packages | 2500-3500 |
| 4.4 | Visual SLAM | VSLAM concepts, Isaac ROS VSLAM, accuracy metrics | 3500-4500 |
| 4.5 | Domain Randomization | Isaac Replicator, visual/physics/sensor randomization | 3000-4000 |
| 4.6 | Sim-to-Real Transfer | Transfer strategies, system identification, validation | 3000-4000 |

**Physical Grounding Requirements** (from module spec):
- Hardware prerequisites clearly stated (RTX 2070+ minimum)
- Performance comparisons between Gazebo and Isaac
- VSLAM accuracy metrics with ground truth comparison
- Transfer analysis references M3 reality gap framework

**Deliverables**:
- `/docs/docs/module-4/chapter-1-isaac-sim-foundations.md`
- `/docs/docs/module-4/chapter-2-scene-composition-sensors.md`
- `/docs/docs/module-4/chapter-3-isaac-ros-integration.md`
- `/docs/docs/module-4/chapter-4-visual-slam.md`
- `/docs/docs/module-4/chapter-5-domain-randomization.md`
- `/docs/docs/module-4/chapter-6-sim-to-real-transfer.md`
- `/docs/docs/module-4/index.md` (module introduction)

**Acceptance Criteria**:
- Isaac Sim 2023.1.1+ and Isaac ROS Humble packages used
- Capstone preparation explicit in each chapter
- Clear connection to prior modules (M2 ROS 2, M3 simulation)
- No hardware required for exercises (cloud alternatives provided)
- All Python scripting (no C++)
- Domain randomization bounds are physically plausible

**Isaac-Specific Requirements**:
- RTX rendering explained for synthetic data generation
- NITROS zero-copy messaging explained
- VSLAM accuracy evaluation methodology included

---

### T207 — Write Module 5 Content (Vision-Language-Action)

**Owner**: book-content-writer
**Skills Used**: technical-writing, diagram-explainer
**Dependencies**: T206

**Description**:
Write all 6 chapters of Module 5 strictly following `/specs/curriculum/module-5.md`.

**Source Specification**: `/specs/curriculum/module-5.md`

**Chapters to Write**:

| Chapter | Title | Key Concepts | Word Estimate |
|---------|-------|--------------|---------------|
| 5.1 | Speech Recognition | Whisper deployment, audio pipeline, noise robustness | 2500-3500 |
| 5.2 | LLM Task Planning | Prompt engineering, structured output, LLM limitations | 3500-4500 |
| 5.3 | Grounding & Action Mapping | Symbol grounding, reference resolution, action mapping | 3500-4500 |
| 5.4 | Vision-Language Models | VLM deployment, scene understanding, object identification | 3000-4000 |
| 5.5 | Safety Constraints | Workspace limits, velocity limits, collision checking | 3000-4000 |
| 5.6 | Failure Handling | Failure taxonomy, recovery strategies, human-in-the-loop | 3000-4000 |

**Critical Framing Requirements** (from module spec):
- VLA as **system orchestration**, NOT "AI magic"
- LLMs as "semantic parsers with limitations"
- Safety as **non-negotiable override** (Constitution Principle III parallel)
- Failure cases shown as prominently as success cases

**Physical Grounding Requirements**:
- Audio latency in VLA timing budget
- LLM inference time affects robot responsiveness
- VLM hallucination risks for safety-critical perception
- Actuator limits must override AI suggestions

**Deliverables**:
- `/docs/docs/module-5/chapter-1-speech-recognition.md`
- `/docs/docs/module-5/chapter-2-llm-task-planning.md`
- `/docs/docs/module-5/chapter-3-grounding-action-mapping.md`
- `/docs/docs/module-5/chapter-4-vision-language-models.md`
- `/docs/docs/module-5/chapter-5-safety-constraints.md`
- `/docs/docs/module-5/chapter-6-failure-handling.md`
- `/docs/docs/module-5/index.md` (module introduction)

**Acceptance Criteria**:
- VLA framed as system orchestration throughout
- No "LLM magic" explanations
- Safety chapter treats safety as absolute override
- Explicit latency measurements for every AI component
- LLM prompts are complete and tested
- Safety filters log all decisions with reasoning
- ML prerequisites appendix referenced for students without ML background

**VLA-Specific Checklist** (per chapter):
- [ ] AI component framed as tool, not intelligence
- [ ] Explicit latency budget contribution stated
- [ ] Failure modes for this stage enumerated
- [ ] Connection to adjacent pipeline stages clear
- [ ] Physical constraint integration explicit
- [ ] Safety considerations addressed

---

### T208 — Write Capstone Content (Integrated Humanoid System)

**Owner**: book-content-writer
**Skills Used**: technical-writing, diagram-explainer
**Dependencies**: T207

**Description**:
Write Capstone content strictly following `/specs/curriculum/capstone.md`.

**Source Specification**: `/specs/curriculum/capstone.md`

**Critical Constraint**: The capstone introduces **NO new concepts**. All content references prior modules.

**Sections to Write**:

| Section | Title | Content Focus | Word Estimate |
|---------|-------|---------------|---------------|
| Intro | Capstone Overview | Goals, prerequisites recap, deliverables | 1500-2000 |
| Phase 1 | System Architecture | Architecture template, design requirements | 2500-3500 |
| Phase 2 | Implementation | Integration guide, debugging strategies | 3000-4000 |
| Phase 3 | Evaluation & Transfer | Evaluation scenarios, transfer analysis template | 2500-3500 |

**Integration Requirements** (from capstone spec):
- Module 1: Physical constraints in all design decisions
- Module 2: ROS 2 as integration backbone
- Module 3: Gazebo or Isaac Sim as execution environment
- Module 4: VSLAM and perception for robust sensing
- Module 5: VLA pipeline for natural language interface

**Deliverables**:
- `/docs/docs/capstone/index.md` (capstone overview)
- `/docs/docs/capstone/phase-1-system-architecture.md`
- `/docs/docs/capstone/phase-2-implementation.md`
- `/docs/docs/capstone/phase-3-evaluation-transfer.md`
- `/docs/docs/capstone/templates/` (architecture template, evaluation rubric, transfer analysis template)

**Acceptance Criteria**:
- NO new concepts introduced (all reference prior modules)
- All 29 capabilities from M1-M5 are utilized
- Requires tradeoff reasoning, not just step-following
- Integration focus: connecting known concepts
- Physical grounding in all deliverable requirements
- Failure analysis embraced, not hidden

**Capstone Checklists Provided**:
- [ ] Architecture design checklist
- [ ] Integration testing checklist
- [ ] Evaluation scenario checklist
- [ ] Transfer analysis checklist

---

## TASK GROUP 2.3 — RAG-READINESS & CONTENT QUALITY

**Purpose**: Ensure content is structured for RAG retrieval and curriculum-aligned

**Owner**: book-content-writer, curriculum-architect

---

### T209 — Enforce RAG-Friendly Content Structure

**Owner**: book-content-writer
**Skills Used**: content-chunking
**Dependencies**: T203, T204, T205, T206, T207, T208

**Description**:
Refactor all written content to ensure clean semantic chunking for RAG retrieval.

**RAG Structure Requirements** (from Constitution Principle VI):
- Semantically complete sections
- Consistent terminology throughout
- Clear headings hierarchy (H1 → H2 → H3)
- One primary concept per section
- No excessively long paragraphs (max 150 words)

**Actions**:
1. Audit all chapter files for section structure
2. Split any section with multiple primary concepts
3. Ensure consistent terminology across modules
4. Add explicit section IDs for precise retrieval
5. Verify heading hierarchy consistency

**Deliverables**:
- All chapter files updated with RAG-friendly structure
- Section ID convention documented
- Terminology glossary created at `/docs/docs/glossary.md`

**Acceptance Criteria**:
- Clear section headers at appropriate granularity
- One primary concept per section
- No section exceeds 500 words without subsections
- Consistent terminology verified across modules
- Glossary covers all technical terms

**RAG-Friendly Patterns**:
```markdown
## Section Title {#section-id}

[1-2 sentence introduction of concept]

### Subsection A

[Focused explanation of one aspect]

### Subsection B

[Focused explanation of another aspect]

### Key Takeaways

[Bulleted summary for retrieval]
```

---

### T210 — Curriculum Drift Audit

**Owner**: curriculum-architect
**Skills Used**: pedagogy-mapping
**Dependencies**: T209

**Description**:
Audit all written content against locked curriculum specifications to detect any drift.

**Audit Checklist** (per chapter):
1. Learning objectives match curriculum spec exactly
2. Concept count within spec limits (4-6 per chapter)
3. Physical grounding elements addressed (sensors, actuators, latency/noise/physics)
4. Prerequisites from prior chapters/modules correctly assumed
5. No concepts introduced that aren't in curriculum spec
6. Autonomy level appropriate for module position

**Actions**:
1. Create audit matrix: Curriculum Spec vs Written Content
2. Flag any deviations (additions, omissions, reorderings)
3. Document drift findings in audit report
4. Recommend corrections for any drift found

**Deliverables**:
- `/specs/001-physical-ai-textbook/phase2-curriculum-audit.md`
- Deviation report (if any drift found)
- Correction recommendations

**Acceptance Criteria**:
- No new concepts introduced beyond curriculum spec
- No reordered learning progression
- All curriculum objectives satisfied by written content
- All physical grounding requirements met
- Audit matrix complete for all 31 chapters

**Drift Categories**:
| Category | Severity | Action |
|----------|----------|--------|
| Additional concept | MEDIUM | Remove unless curriculum-architect approves |
| Missing objective | HIGH | Content must be added |
| Reordered progression | HIGH | Restructure to match spec |
| Physical grounding gap | HIGH | Add required grounding element |
| Terminology inconsistency | LOW | Standardize to glossary term |

---

### T211 — Create Appendices and Supplementary Content

**Owner**: book-content-writer
**Skills Used**: technical-writing
**Dependencies**: T209

**Description**:
Create supplementary content that supports the main curriculum without adding new concepts.

**Appendices Required** (from curriculum specs):
1. **ML Concepts for Roboticists** - Background for Module 5 (neural networks, embeddings, transformers)
2. **Hardware Requirements Guide** - GPU requirements, cloud alternatives, self-check scripts
3. **ROS 2 Installation Guide** - Ubuntu 22.04 setup, workspace configuration
4. **Gazebo & Isaac Installation Guide** - Platform-specific setup instructions
5. **Glossary** - All technical terms with consistent definitions

**Deliverables**:
- `/docs/docs/appendix/ml-concepts-roboticists.md`
- `/docs/docs/appendix/hardware-requirements.md`
- `/docs/docs/appendix/ros2-installation.md`
- `/docs/docs/appendix/simulation-installation.md`
- `/docs/docs/glossary.md`

**Acceptance Criteria**:
- Appendices support main content without introducing new curriculum concepts
- Hardware requirements match Module 4 and Module 5 specs exactly
- Installation guides tested and verified working
- Glossary terms consistent with content usage

---

## TASK GROUP 2.4 — PHASE VALIDATION & LOCK

**Purpose**: Validate Phase 2 completion and lock textbook content

**Owner**: chief-orchestrator

---

### T212 — Build Verification

**Owner**: frontend-platform
**Dependencies**: T209, T210, T211

**Description**:
Verify that Docusaurus project builds cleanly with all content.

**Actions**:
1. Run `npm run build` and verify no errors
2. Run `npm run serve` and verify local serving works
3. Check all internal links resolve correctly
4. Verify sidebar navigation matches curriculum spec
5. Test search functionality

**Deliverables**:
- Build log with no errors
- Link validation report
- Screenshot of complete navigation

**Acceptance Criteria**:
- `npm run build` completes with 0 errors
- `npm run serve` serves complete site
- All 31 chapters accessible via navigation
- All internal links valid
- Search returns relevant results

---

### T213 — Phase 2 Validation & Lock

**Owner**: chief-orchestrator
**Dependencies**: T210, T212

**Description**:
Validate that all Phase 2 objectives are met and lock textbook content for downstream phases.

**Validation Checklist**:
- [ ] Docusaurus builds cleanly (T212 verified)
- [ ] All 5 modules + capstone content exists (31 chapters)
- [ ] Curriculum drift audit passed (T210)
- [ ] RAG-friendly structure verified (T209)
- [ ] All appendices created (T211)
- [ ] No backend or AI logic present in `/docs/`
- [ ] All content is curriculum-aligned

**Phase 2 Completion Criteria**:
| Criterion | Verification Method | Status |
|-----------|---------------------|--------|
| Docusaurus project functional | `npm run build` succeeds | |
| All modules written | File count verification | |
| Curriculum alignment | Audit report | |
| RAG structure | Section structure audit | |
| No backend code | Directory inspection | |
| Physical grounding | Checklist per chapter | |

**Deliverables**:
- `/specs/001-physical-ai-textbook/PHASE2-COMPLETE.md` (freeze declaration)
- Phase 2 completion checklist signed
- Authorization for Phase 3 (Backend RAG) to proceed

**Acceptance Criteria**:
- All Phase 2 tasks completed
- Curriculum drift audit shows no major deviations
- chief-orchestrator signs off on Phase 2 completion
- Textbook content LOCKED for Phase 3

---

## DEPENDENCIES & EXECUTION ORDER

### Task Dependency Graph

```
T200 (Phase 1 Check)
  │
  ▼
T201 (Initialize Docusaurus)
  │
  ▼
T202 (Configure Navigation)
  │
  ├────────────────────────────────────────────────────────┐
  ▼                                                        │
T203 (Module 1) ──► T204 (Module 2) ──► T205 (Module 3)   │
                                              │            │
                                              ▼            │
                                        T206 (Module 4)    │
                                              │            │
                                              ▼            │
                                        T207 (Module 5)    │
                                              │            │
                                              ▼            │
                                        T208 (Capstone)    │
                                              │            │
  ┌───────────────────────────────────────────┘            │
  │                                                        │
  ▼                                                        │
T209 (RAG Structure) ◄─────────────────────────────────────┘
  │
  ├─────────────┐
  ▼             ▼
T210 (Audit)  T211 (Appendices)
  │             │
  └──────┬──────┘
         ▼
T212 (Build Verification)
         │
         ▼
T213 (Phase 2 Lock)
```

### Task Dependency Matrix

| Task | Depends On | Blocks |
|------|------------|--------|
| T200 | Phase 1 | T201 |
| T201 | T200 | T202 |
| T202 | T201 | T203 |
| T203 | T202 | T204, T209 |
| T204 | T203 | T205, T209 |
| T205 | T204 | T206, T209 |
| T206 | T205 | T207, T209 |
| T207 | T206 | T208, T209 |
| T208 | T207 | T209 |
| T209 | T203-T208 | T210, T211 |
| T210 | T209 | T213 |
| T211 | T209 | T212 |
| T212 | T210, T211 | T213 |
| T213 | T212 | Phase 3 |

### Parallel Opportunities

**Limited parallelism in Phase 2** due to sequential module dependencies:
- Module content must be written sequentially (M1 → M2 → M3 → M4 → M5 → Cap)
- T210 and T211 can run in parallel after T209
- No parallelism in content authoring (curriculum dependencies)

---

## ARTIFACTS TO CREATE

### Docusaurus Structure

| Artifact | Task | Path |
|----------|------|------|
| Docusaurus Config | T201 | `/docs/docusaurus.config.js` |
| Sidebars | T202 | `/docs/sidebars.js` |
| Module 1 Content | T203 | `/docs/docs/module-1/*.md` (5 files) |
| Module 2 Content | T204 | `/docs/docs/module-2/*.md` (7 files) |
| Module 3 Content | T205 | `/docs/docs/module-3/*.md` (7 files) |
| Module 4 Content | T206 | `/docs/docs/module-4/*.md` (7 files) |
| Module 5 Content | T207 | `/docs/docs/module-5/*.md` (7 files) |
| Capstone Content | T208 | `/docs/docs/capstone/*.md` (4 files) |
| Appendices | T211 | `/docs/docs/appendix/*.md` (4 files) |
| Glossary | T209 | `/docs/docs/glossary.md` |

### Validation Artifacts

| Artifact | Task | Path |
|----------|------|------|
| Curriculum Audit | T210 | `/specs/001-physical-ai-textbook/phase2-curriculum-audit.md` |
| Phase 2 Freeze | T213 | `/specs/001-physical-ai-textbook/PHASE2-COMPLETE.md` |

---

## CONTENT STATISTICS

| Module | Chapters | Est. Words | Est. Pages |
|--------|----------|------------|------------|
| Module 1 | 4 | 10,000-14,000 | 20-28 |
| Module 2 | 6 | 15,500-21,500 | 31-43 |
| Module 3 | 6 | 17,000-23,500 | 34-47 |
| Module 4 | 6 | 18,500-25,500 | 37-51 |
| Module 5 | 6 | 18,500-25,000 | 37-50 |
| Capstone | 4 | 9,500-13,000 | 19-26 |
| Appendices | 5 | 5,000-8,000 | 10-16 |
| **Total** | **37** | **94,000-130,500** | **188-261** |

---

## PHASE 2 COMPLETION CONDITIONS

Phase 2 is complete when:

- [x] T200: Phase 1 verified complete
- [ ] T201: Docusaurus project initialized
- [ ] T202: Navigation configured to match curriculum
- [ ] T203: Module 1 content written
- [ ] T204: Module 2 content written
- [ ] T205: Module 3 content written
- [ ] T206: Module 4 content written
- [ ] T207: Module 5 content written
- [ ] T208: Capstone content written
- [ ] T209: RAG-friendly structure verified
- [ ] T210: Curriculum drift audit passed
- [ ] T211: Appendices created
- [ ] T212: Build verification passed
- [ ] T213: Phase 2 freeze declared

**Explicit Exclusions** (NOT in Phase 2):
- Backend API implementation
- RAG system implementation
- User authentication
- Personalization logic
- Urdu translation
- Deployment to GitHub Pages
- Any AI/LLM integration code

---

## NEXT PHASE (After Phase 2 Lock)

Once Phase 2 is complete and textbook content is locked:

1. **Phase 3**: Backend RAG Implementation (backend-rag agent)
   - FastAPI backend
   - Vector database setup (Qdrant)
   - Content chunking for RAG
   - Retrieval guardrails

2. **Phase 4**: Frontend Platform & AI Embedding (frontend-platform agent)
   - Embedded chatbot UI
   - Personalization controls
   - Translation toggle

3. **Phase 5**: Authentication & Personalization (identity-reasoning-guardian agent)
   - Better Auth integration
   - User profiling

4. **Phase 6**: Deployment (devops-github agent)
   - GitHub Pages deployment
   - Railway deployment

---

## NOTES

- All Phase 2 tasks are content creation, NOT implementation
- The [CONTENT] tag indicates book-content-writer ownership
- Constitution Principle II (Curriculum Authority) governs content alignment
- Constitution Principle VI (Content Structure) governs RAG-readiness
- Curriculum specs are READ-ONLY in this phase
- Any curriculum changes require formal amendment process

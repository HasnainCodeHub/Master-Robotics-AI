# Implementation Plan: Physical AI & Humanoid Robotics - AI-Native Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2024-12-24 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

## Summary

Build a complete AI-native educational system consisting of:
1. **Docusaurus Textbook** - 5 modules + capstone covering Physical AI & Humanoid Robotics
2. **FastAPI RAG Backend** - Grounded Q&A using Qdrant vectors + Neon Postgres
3. **Embedded Chatbot** - Strict retrieval-only answers with explicit refusal
4. **Bonus Features** - Better Auth personalization + Urdu translation

Technical approach: Static site with API backend, vector-based retrieval, and strict grounding guardrails.

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.x, Node.js 20.x LTS
- Backend: Python 3.11+

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18.x, MDX
- Backend: FastAPI, OpenAI SDK, Qdrant Client, SQLAlchemy, Better Auth

**Storage**:
- Qdrant Cloud (Free Tier) - Vector embeddings for content chunks
- Neon Serverless Postgres - User profiles, chat history, metadata

**Testing**:
- Frontend: Jest, Playwright (E2E)
- Backend: pytest, httpx (async testing)

**Target Platform**:
- Web browsers (Chrome, Firefox, Safari, Edge)
- GitHub Pages (static frontend)
- Railway (Python backend)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Page load: <3 seconds
- Chatbot response: <5 seconds (including retrieval)
- 100 concurrent users without degradation

**Constraints**:
- RAG must refuse if retrieval insufficient (NON-NEGOTIABLE)
- No external knowledge in chatbot responses
- Free tier services only (Qdrant Cloud, Neon, Railway)

**Scale/Scope**:
- 5 modules + 1 capstone = ~20-30 chapters
- ~100 content chunks for RAG
- Target: 100 concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-Driven Development | ✅ PASS | Spec created before plan |
| II. Curriculum Authority | ✅ PASS | curriculum-architect owns content structure |
| III. RAG Safety (NON-NEGOTIABLE) | ✅ PASS | Refusal logic in backend-rag spec |
| IV. Physical Grounding | ✅ PASS | Content will reference real hardware |
| V. Agent Mandate Boundaries | ✅ PASS | Agents defined with explicit skills |
| VI. Content Structure | ✅ PASS | Template enforces objectives/examples/takeaways |
| VII. Test-First Discipline | ✅ PASS | Tests in task phases |
| VIII. Simplicity & Minimal Change | ✅ PASS | Using mandated stack only |
| IX. No Hallucination | ✅ PASS | RAG grounding enforced |
| X. Deployment Reproducibility | ✅ PASS | GitHub Pages + Railway with env vars |

**Constitution Check Result**: ALL GATES PASS

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (API specs)
│   ├── rag-api.yaml     # RAG endpoints OpenAPI
│   └── auth-api.yaml    # Auth endpoints OpenAPI
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
book/
├── docs/
│   ├── module-1-physical-ai/
│   │   ├── 01-embodied-intelligence.md
│   │   ├── 02-sensors-actuators.md
│   │   └── 03-physical-constraints.md
│   ├── module-2-ros2/
│   ├── module-3-simulation/
│   ├── module-4-nvidia-isaac/
│   ├── module-5-vla/
│   └── capstone/
├── src/
│   ├── components/
│   │   ├── Chatbot/
│   │   ├── TranslationToggle/
│   │   └── PersonalizationBadge/
│   ├── pages/
│   └── services/
│       └── api.ts
├── static/
├── docusaurus.config.ts
└── package.json

# Backend (FastAPI)
backend/
├── app/
│   ├── main.py
│   ├── api/
│   │   ├── routes/
│   │   │   ├── chat.py
│   │   │   ├── auth.py
│   │   │   └── content.py
│   │   └── deps.py
│   ├── core/
│   │   ├── config.py
│   │   └── security.py
│   ├── models/
│   │   ├── user.py
│   │   ├── chat.py
│   │   └── content.py
│   ├── services/
│   │   ├── rag.py
│   │   ├── embedding.py
│   │   └── translation.py
│   └── db/
│       ├── qdrant.py
│       └── postgres.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
└── Dockerfile

# Shared
├── .github/
│   └── workflows/
│       ├── deploy-book.yml
│       └── deploy-backend.yml
├── .env.example
└── README.md
```

**Structure Decision**: Web application with separate frontend (Docusaurus static site) and backend (FastAPI API). This aligns with the mandated tech stack and deployment targets (GitHub Pages for book, Railway for backend).

## Phase Execution Plan

### Phase 0: Governance & Foundations
- Confirm constitution saved at `/specs/constitution.md`
- Freeze agent definitions and skill boundaries
- Validate all work references specs

### Phase 1: Curriculum Intelligence Design
- Define course-level learning goals
- Define module-level objectives
- Define chapter-level outcomes
- Validate prerequisite flow

**Artifacts**: `/specs/curriculum/` directory with module specs

### Phase 2: Textbook Content Creation
- Initialize Docusaurus project
- Write chapters from curriculum specs
- Enforce concept → example → physical grounding
- Prepare content for RAG chunking

**Artifacts**: `/book/docs/` with all module content

### Phase 3: RAG System Design & Backend
- Define semantic chunking rules
- Define retrieval scope and refusal conditions
- Define FastAPI API boundaries
- Configure Qdrant + Neon connections

**Artifacts**: `/specs/rag/` specs, `/backend/` implementation

### Phase 4: Frontend Platform & AI Embedding
- Define chapter reading UI patterns
- Embed chatbot component
- Define selected-text query UX
- Define refusal messaging

**Artifacts**: `/book/src/components/`

### Phase 5: Authentication & Personalization (Bonus)
- Implement Better Auth signup/signin
- Define user profile schema
- Define personalization rules

**Artifacts**: `/specs/auth/`, auth components

### Phase 6: Urdu Translation (Bonus)
- Define translation scope
- Define technical term preservation
- Implement chapter-level toggle

**Artifacts**: Translation component, `/specs/frontend/translation-rules.md`

### Phase 7: Deployment & Release
- Configure GitHub Pages workflow
- Configure Railway deployment
- Set up environment variables
- Validate CI checkpoints

**Artifacts**: `.github/workflows/`, deployment configs

## Complexity Tracking

> No violations requiring justification. All complexity is mandated by constitution.

| Item | Justification |
|------|---------------|
| Two deployment targets | Constitution mandates GitHub Pages + Railway |
| Two databases | Constitution mandates Qdrant (vectors) + Neon (relational) |
| Multiple languages | TypeScript frontend + Python backend per constitution |

## Integration Points

### Frontend ↔ Backend

```
Docusaurus (GitHub Pages)
    │
    ├── /api/chat (POST) ──────→ FastAPI (Railway)
    │       └── question, selected_text?     │
    │                                        ├── Qdrant Cloud
    │                                        │   └── vector search
    │                                        │
    │ ←── grounded_answer, sources ──────────┤
    │                                        │
    ├── /api/auth/* ───────────→             ├── Neon Postgres
    │       └── signup, signin               │   └── users, sessions
    │                                        │
    └── /api/translate (POST) ─→             └── Translation service
            └── chapter_id, target_lang
```

### Data Flow: RAG Query

1. User selects text (optional) and types question
2. Frontend sends `POST /api/chat` with question + selected_text
3. Backend embeds question using OpenAI
4. Backend queries Qdrant for similar content chunks
5. If chunks found with confidence > threshold:
   - Generate answer using only retrieved chunks
   - Return answer + source references
6. If chunks insufficient:
   - Return explicit refusal message
7. Frontend displays answer or refusal

## Next Steps

1. **Run `/sp.tasks`** to generate task breakdown
2. **Execute Phase 0** - Finalize governance artifacts
3. **Execute Phase 1** - Curriculum design (curriculum-architect agent)
4. **Continue sequentially** through phases

## Artifacts to Generate

The following artifacts will be created during plan execution:

| Artifact | Phase | Agent |
|----------|-------|-------|
| research.md | 0 | chief-orchestrator |
| data-model.md | 1 | backend-rag |
| contracts/rag-api.yaml | 1 | backend-rag |
| contracts/auth-api.yaml | 1 | identity-reasoning-guardian |
| quickstart.md | 1 | project-bootstrap |
| /specs/curriculum/*.md | 1 | curriculum-architect |
| /specs/rag/*.md | 3 | backend-rag |
| /specs/auth/*.md | 5 | identity-reasoning-guardian |
| /specs/frontend/*.md | 4,6 | frontend-platform |

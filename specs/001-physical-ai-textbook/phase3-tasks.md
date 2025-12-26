# Tasks: Phase 3 — Backend & RAG Implementation

**Feature Branch**: `001-physical-ai-textbook`
**Phase**: 3 — Backend & RAG
**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: Phase 2 Complete (textbook content locked), plan.md, spec.md, data-model.md, contracts/rag-api.yaml, research.md

**Scope**: Design and implement a SAFE, GROUNDED Retrieval-Augmented Generation system served by a FastAPI backend, prepared for Railway deployment.

**Explicit Exclusions**:
- Frontend UI work (Phase 4)
- Authentication logic (Phase 5)
- Translation feature (Phase 6)

**Tests**: Optional unless explicitly requested.

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US2 = RAG Chatbot)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Backend Project Initialization)

**Purpose**: Initialize FastAPI backend project structure and dependencies

- [ ] T301 Verify Phase 2 completion (PHASE2-COMPLETE.md exists and is locked)
- [ ] T302 Create backend project directory structure per plan.md in backend/
- [ ] T303 [P] Create requirements.txt with FastAPI, Qdrant, SQLAlchemy, OpenAI dependencies in backend/requirements.txt
- [ ] T304 [P] Create pyproject.toml with project metadata in backend/pyproject.toml
- [ ] T305 [P] Create .env.example with required environment variables in backend/.env.example
- [ ] T306 Create main FastAPI application entry point in backend/app/main.py
- [ ] T307 [P] Create core config module with settings in backend/app/core/config.py
- [ ] T308 [P] Create logging configuration in backend/app/core/logging.py

**Checkpoint**: Backend skeleton is runnable with `uvicorn app.main:app`

---

## Phase 2: Foundational (Database & External Service Connections)

**Purpose**: Core infrastructure that MUST be complete before RAG implementation

**CRITICAL**: No RAG work can begin until database connections are verified

- [ ] T309 Create Qdrant client connection module in backend/app/db/qdrant.py
- [ ] T310 Create Neon Postgres connection with SQLAlchemy async in backend/app/db/postgres.py
- [ ] T311 [P] Create Qdrant collection schema for ContentChunk (1536 dimensions) in backend/app/db/qdrant.py
- [ ] T312 [P] Create SQLAlchemy models for ChatSession and ChatMessage in backend/app/models/chat.py
- [ ] T313 Create database migration script using Alembic in backend/alembic/
- [ ] T314 [P] Create health check endpoint verifying both database connections in backend/app/api/routes/health.py
- [ ] T315 Configure CORS middleware for GitHub Pages origin in backend/app/main.py
- [ ] T316 Create base exception handlers and error response schemas in backend/app/api/deps.py

**Checkpoint**: Health endpoint returns healthy with both qdrant_connected and postgres_connected = true

---

## Phase 3: User Story 2 — RAG Chatbot (Priority: P1) MVP

**Goal**: Implement grounded Q&A using ONLY retrieved textbook content with explicit refusal when retrieval is insufficient

**Independent Test**:
- Ask in-scope question (e.g., "What is a ROS 2 topic?") → Get grounded answer with sources
- Ask out-of-scope question (e.g., "How do I train a custom LLM?") → Get explicit refusal

### Content Ingestion Pipeline

- [ ] T317 [US2] Create content chunking service with section-based splitting in backend/app/services/chunking.py
- [ ] T318 [US2] Create embedding service using OpenAI text-embedding-3-small in backend/app/services/embedding.py
- [ ] T319 [US2] Create content ingestion CLI script to process docs/docs/ content in backend/scripts/ingest_content.py
- [ ] T320 [US2] Implement chunk overlap logic (50 tokens) and heading prefix in backend/app/services/chunking.py

### Retrieval Implementation

- [ ] T321 [US2] Create RAG service with vector search in backend/app/services/rag.py
- [ ] T322 [US2] Implement similarity threshold filtering (score > 0.7) in backend/app/services/rag.py
- [ ] T323 [US2] Create retrieval result schema with SourceReference in backend/app/schemas/chat.py

### Refusal Logic (NON-NEGOTIABLE)

- [ ] T324 [US2] Implement RefusalResponse schema with reason enum in backend/app/schemas/chat.py
- [ ] T325 [US2] Implement refusal logic for empty retrieval in backend/app/services/rag.py
- [ ] T326 [US2] Implement refusal logic for low-confidence results in backend/app/services/rag.py
- [ ] T327 [US2] Implement refusal logic for out-of-scope detection in backend/app/services/rag.py

### Answer Generation

- [ ] T328 [US2] Create grounded answer generator using OpenAI with context-only prompt in backend/app/services/generation.py
- [ ] T329 [US2] Implement selected_text mode limiting context to user-highlighted text in backend/app/services/rag.py
- [ ] T330 [US2] Add system prompt enforcing no external knowledge in backend/app/services/generation.py

### Chat API Endpoints

- [ ] T331 [US2] Create ChatRequest and ChatResponse Pydantic schemas per rag-api.yaml in backend/app/schemas/chat.py
- [ ] T332 [US2] Implement POST /api/chat endpoint in backend/app/api/routes/chat.py
- [ ] T333 [US2] Create ChatSession service for session management in backend/app/services/session.py
- [ ] T334 [US2] Store chat messages in Postgres with sources and was_refusal flag in backend/app/services/session.py

### Content Debug Endpoint

- [ ] T335 [US2] Implement GET /api/content/chunks endpoint for admin/debug in backend/app/api/routes/content.py

**Checkpoint**: POST /api/chat returns grounded answers for in-scope questions and explicit refusals for out-of-scope questions

---

## Phase 4: Railway Deployment Preparation

**Purpose**: Prepare backend for Railway deployment

- [ ] T336 [P] Create Dockerfile for FastAPI application in backend/Dockerfile
- [ ] T337 [P] Create railway.toml configuration in backend/railway.toml
- [ ] T338 [P] Create Procfile for Railway deployment in backend/Procfile
- [ ] T339 Document environment variables needed for Railway deployment in backend/README.md
- [ ] T340 Create GitHub Actions workflow for backend deployment in .github/workflows/deploy-backend.yml

**Checkpoint**: Backend deploys successfully to Railway with all environment variables configured

---

## Phase 5: Validation & Phase Lock

**Purpose**: Validate RAG safety and lock Phase 3

- [ ] T341 Manual test: In-scope question returns grounded answer with sources
- [ ] T342 Manual test: Out-of-scope question returns explicit refusal
- [ ] T343 Manual test: Empty retrieval returns refusal (not hallucination)
- [ ] T344 Manual test: Selected text mode uses ONLY selected text as context
- [ ] T345 Manual test: Health endpoint confirms both databases connected
- [ ] T346 Create PHASE3-COMPLETE.md documenting all implemented components in specs/001-physical-ai-textbook/PHASE3-COMPLETE.md

**Checkpoint**: Phase 3 is validated, documented, and locked

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately after Phase 2 lock verification
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all RAG work
- **User Story 2 (Phase 3)**: Depends on Foundational completion
  - Content Ingestion → Retrieval → Refusal Logic → Answer Generation → API Endpoints
- **Deployment (Phase 4)**: Depends on User Story 2 completion
- **Validation (Phase 5)**: Depends on Deployment completion

### Task Dependencies within User Story 2

```
T317 (chunking) → T319 (ingestion CLI) → T320 (overlap logic)
                       ↓
T318 (embedding) ──────┴──────→ T321 (RAG service)
                                     ↓
                               T322 (threshold) → T323 (schemas)
                                     ↓
                         T324-T327 (refusal logic)
                                     ↓
                         T328-T330 (answer generation)
                                     ↓
                         T331-T334 (API endpoints)
```

### Parallel Opportunities

Tasks marked [P] within each phase can run in parallel:
- Phase 1: T303, T304, T305, T307, T308
- Phase 2: T311, T312, T314
- Phase 4: T336, T337, T338

---

## Parallel Example: Phase 2 Foundation

```bash
# Launch all parallel foundation tasks together:
Task: "Create Qdrant collection schema (T311)"
Task: "Create SQLAlchemy models (T312)"
Task: "Create health check endpoint (T314)"
```

---

## Implementation Strategy

### MVP First (Minimum Viable RAG)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all RAG)
3. Complete T317-T323: Basic retrieval working
4. Complete T324-T327: Refusal logic (NON-NEGOTIABLE)
5. Complete T328-T332: API endpoint working
6. **STOP and VALIDATE**: Test grounding and refusal behavior
7. Complete remaining tasks

### Incremental Delivery

1. Setup + Foundation → Backend skeleton running
2. Add Ingestion + Retrieval → Content searchable
3. Add Refusal Logic → Safety enforced
4. Add Generation → Grounded answers
5. Add API → Full endpoint working
6. Deploy → Railway live

---

## RAG Safety Checklist (NON-NEGOTIABLE)

Before marking Phase 3 complete, verify ALL of the following:

- [ ] Chatbot NEVER answers without retrieved context
- [ ] Refusal message is explicit and user-friendly
- [ ] Refusal reason is logged (insufficient_retrieval, out_of_scope, selected_text_insufficient)
- [ ] Sources are always included with grounded answers
- [ ] was_refusal flag is correctly set in all responses
- [ ] Selected text mode uses ONLY user-selected text
- [ ] System prompt explicitly prohibits external knowledge
- [ ] Confidence threshold (0.7) is enforced

---

## Notes

- [P] tasks = different files, no dependencies
- [US2] = User Story 2 (RAG Chatbot) from spec.md
- Each checkpoint should be independently testable
- Commit after each task or logical group
- Refusal is ALWAYS preferred over speculation
- NO authentication logic in this phase (Phase 5)
- NO frontend components in this phase (Phase 4)

---

## Task Summary

| Phase | Tasks | Parallelizable |
|-------|-------|----------------|
| Setup | T301-T308 (8) | 5 |
| Foundational | T309-T316 (8) | 3 |
| User Story 2 | T317-T335 (19) | 0 (sequential dependencies) |
| Deployment | T336-T340 (5) | 3 |
| Validation | T341-T346 (6) | 0 (sequential) |
| **Total** | **46 tasks** | **11 parallelizable** |

**MVP Scope**: T301-T335 (35 tasks) for working RAG endpoint
**Full Scope**: All 46 tasks for deployed, validated Phase 3

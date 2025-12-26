# Tasks: Phase 4 — Frontend Platform & Embedded Chatbot

**Feature Branch**: `001-physical-ai-textbook`
**Phase**: 4 — Frontend Platform
**Input**: Design documents from `/specs/001-physical-ai-textbook/` and Phase 3 backend
**Prerequisites**: Phase 3 Complete (backend locked), plan.md, spec.md, contracts/rag-api.yaml

**Scope**: Embed the RAG chatbot into the Docusaurus-based textbook UI and expose AI interaction safely, transparently, and curriculum-aligned.

**Explicit Exclusions**:
- Authentication logic (Phase 5)
- Translation feature (Phase 6)
- Backend modifications (Phase 3 locked)

**Tests**: Optional unless explicitly requested.

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Maps to User Story 2 (RAG Chatbot) and User Story 3 (Navigation)
- Include exact file paths in descriptions

---

## Phase 1: Entry Validation

**Purpose**: Verify Phase 3 is complete and backend is ready for integration

- [ ] T400 Verify Phase 3 completion (PHASE3-COMPLETE.md exists and is locked)
- [ ] T401 Verify FastAPI backend runs locally with `uvicorn app.main:app`
- [ ] T402 Test POST /api/chat endpoint returns grounded answer or refusal
- [ ] T403 Test GET /health endpoint returns healthy status

**Checkpoint**: Backend integration verified and ready

---

## Phase 2: Frontend Architecture Specs

**Purpose**: Define UI specifications before implementation

- [ ] T404 [P] Create chatbot UI specification in specs/frontend/chatbot-ui.md
- [ ] T405 [P] Create selected-text interaction specification in specs/frontend/selected-text-ui.md
- [ ] T406 [P] Create source transparency specification in specs/frontend/source-display.md

**Checkpoint**: Frontend specifications complete and locked

---

## Phase 3: User Story 2 — Chatbot UI Implementation (Priority: P1)

**Goal**: Embed functional RAG chatbot in Docusaurus textbook

**Independent Test**:
- Ask in-scope question via UI → See grounded answer with sources
- Ask out-of-scope question → See clear refusal message

### Chatbot Component Structure

- [ ] T407 [US2] Create Chatbot component directory in docs/src/components/Chatbot/
- [ ] T408 [P] [US2] Create ChatMessage component in docs/src/components/Chatbot/ChatMessage.tsx
- [ ] T409 [P] [US2] Create ChatInput component in docs/src/components/Chatbot/ChatInput.tsx
- [ ] T410 [P] [US2] Create SourceReference component in docs/src/components/Chatbot/SourceReference.tsx
- [ ] T411 [US2] Create main Chatbot container in docs/src/components/Chatbot/index.tsx
- [ ] T412 [US2] Create Chatbot CSS/styles in docs/src/components/Chatbot/styles.module.css

### API Integration

- [ ] T413 [US2] Create API service module in docs/src/services/api.ts
- [ ] T414 [US2] Implement sendChatMessage function calling POST /api/chat
- [ ] T415 [US2] Implement error handling for network failures and API errors
- [ ] T416 [US2] Add environment configuration for API base URL in docs/docusaurus.config.ts

### State Management

- [ ] T417 [US2] Create chat state management (messages, loading, session) in docs/src/hooks/useChat.ts
- [ ] T418 [US2] Implement message history state with sources tracking
- [ ] T419 [US2] Handle session_id for conversation continuity

### Chatbot Integration

- [ ] T420 [US2] Create ChatbotWrapper for global availability in docs/src/theme/Root.tsx
- [ ] T421 [US2] Add floating chatbot toggle button in docs/src/components/Chatbot/ChatbotToggle.tsx
- [ ] T422 [US2] Implement open/close animation and positioning

**Checkpoint**: Chatbot UI functional with backend integration

---

## Phase 4: Selected-Text Mode Implementation

**Goal**: Enable "answer from selected text only" feature

**Independent Test**:
- Select text → Ask question → Answer uses only selected text
- Select insufficient text → See appropriate refusal

### Text Selection Capture

- [ ] T423 [US2] Create text selection hook in docs/src/hooks/useTextSelection.ts
- [ ] T424 [US2] Implement selection detection within chapter content
- [ ] T425 [US2] Create visual indicator for selected text scope

### Selected-Text Query

- [ ] T426 [US2] Add selected_text parameter to sendChatMessage API call
- [ ] T427 [US2] Display selected text preview in chat input area
- [ ] T428 [US2] Add "Clear selection" button to reset scope
- [ ] T429 [US2] Show visual indicator when in selected-text mode

**Checkpoint**: Selected-text mode fully functional

---

## Phase 5: RAG Transparency & UX Safety

**Goal**: Make AI boundaries visible and refusal messages clear

**Independent Test**:
- Answer shows clickable source references
- Refusal is clearly styled (not as error)

### Source Display

- [ ] T430 [US2] Implement collapsible source references section
- [ ] T431 [US2] Link source references to chapter sections (if possible)
- [ ] T432 [US2] Show confidence scores optionally (admin/debug mode)

### Refusal UX

- [ ] T433 [US2] Style refusal messages differently from answers (not as error)
- [ ] T434 [US2] Display refusal_reason in user-friendly format
- [ ] T435 [US2] Add helpful suggestions for out-of-scope questions

### Loading States

- [ ] T436 [P] [US2] Implement loading indicator during API call
- [ ] T437 [P] [US2] Add typing animation for assistant responses

**Checkpoint**: RAG transparency complete

---

## Phase 6: Polish & Accessibility

**Purpose**: Final refinements and accessibility

- [ ] T438 [P] Keyboard navigation support (Tab, Enter, Escape)
- [ ] T439 [P] ARIA labels for screen readers
- [ ] T440 [P] Mobile responsive layout
- [ ] T441 [P] Dark mode support (if Docusaurus theme supports)
- [ ] T442 Test chatbot across all modules and chapters

**Checkpoint**: Chatbot polished and accessible

---

## Phase 7: Validation & Lock

**Purpose**: Validate frontend behavior against RAG safety expectations

- [ ] T443 Manual test: In-scope question shows answer with sources
- [ ] T444 Manual test: Out-of-scope question shows clear refusal
- [ ] T445 Manual test: Selected-text mode limits context correctly
- [ ] T446 Manual test: Refusal is styled appropriately (not as error)
- [ ] T447 Manual test: Source references are visible and accurate
- [ ] T448 Create PHASE4-COMPLETE.md documenting all implemented components in specs/001-physical-ai-textbook/PHASE4-COMPLETE.md

**Checkpoint**: Phase 4 validated, documented, and locked

---

## Dependencies & Execution Order

### Phase Dependencies

- **Entry Validation (Phase 1)**: Depends on Phase 3 completion
- **Specs (Phase 2)**: Depends on Entry Validation
- **Chatbot UI (Phase 3)**: Depends on Specs completion
- **Selected-Text (Phase 4)**: Depends on Chatbot UI completion
- **Transparency (Phase 5)**: Depends on Chatbot UI completion (can parallel with Phase 4)
- **Polish (Phase 6)**: Depends on Phases 4 and 5 completion
- **Validation (Phase 7)**: Depends on Phase 6 completion

### Task Dependencies within Chatbot UI (Phase 3)

```
T407 (directory) → T408-T410 (components in parallel)
                         ↓
                   T411 (main container)
                         ↓
                   T412 (styles)
                         ↓
T413-T415 (API) → T416 (config)
                         ↓
T417-T419 (state) → T420-T422 (integration)
```

### Parallel Opportunities

Tasks marked [P] can run in parallel:
- Phase 2: T404, T405, T406
- Phase 3: T408, T409, T410 (after T407)
- Phase 5: T436, T437
- Phase 6: T438, T439, T440, T441

---

## Implementation Strategy

### MVP First (Basic Chat)

1. Complete Entry Validation (Phase 1)
2. Complete Specs (Phase 2)
3. Complete T407-T422 for basic chatbot
4. **STOP and VALIDATE**: Test basic Q&A works
5. Continue with selected-text and polish

### Component Architecture

```
docs/src/
├── components/
│   └── Chatbot/
│       ├── index.tsx           # Main container
│       ├── ChatMessage.tsx     # Single message
│       ├── ChatInput.tsx       # Input field + send
│       ├── SourceReference.tsx # Source display
│       ├── ChatbotToggle.tsx   # Open/close button
│       └── styles.module.css   # Component styles
├── hooks/
│   ├── useChat.ts              # Chat state
│   └── useTextSelection.ts     # Text selection
├── services/
│   └── api.ts                  # Backend API calls
└── theme/
    └── Root.tsx                # Global chatbot wrapper
```

---

## Frontend Safety Checklist

Before marking Phase 4 complete, verify:

- [ ] No LLM calls from frontend (only backend API)
- [ ] selected_text sent exactly as user selected
- [ ] Refusal messages are user-friendly (not error styled)
- [ ] Source references are visible and accurate
- [ ] Loading states prevent double-submission
- [ ] Error messages don't expose internal details
- [ ] API base URL is configurable via environment

---

## Notes

- [P] tasks = different files, no dependencies
- [US2] = User Story 2 (RAG Chatbot) from spec.md
- All backend calls must go through /api/chat
- NEVER call OpenAI or any LLM directly from frontend
- Refusal is success (200 response), not error

---

## Task Summary

| Phase | Tasks | Parallelizable |
|-------|-------|----------------|
| Entry Validation | T400-T403 (4) | 0 |
| Specs | T404-T406 (3) | 3 |
| Chatbot UI | T407-T422 (16) | 3 |
| Selected-Text | T423-T429 (7) | 0 |
| Transparency | T430-T437 (8) | 2 |
| Polish | T438-T442 (5) | 4 |
| Validation | T443-T448 (6) | 0 |
| **Total** | **49 tasks** | **12 parallelizable** |

**MVP Scope**: T400-T422 (23 tasks) for basic working chatbot
**Full Scope**: All 49 tasks for polished, accessible Phase 4

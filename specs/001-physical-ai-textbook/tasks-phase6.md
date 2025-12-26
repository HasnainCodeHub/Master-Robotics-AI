# Tasks: Physical AI & Humanoid Robotics - AI-Native Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md, data-model.md, Phase 5 LOCKED
**Scope**: PHASE 6 — Urdu Translation Toggle

**User Story**: US5 - Read Content in Urdu (Priority: P3 - Bonus)

**Objective**: Enable on-demand Urdu translation of textbook chapters
via a user-controlled toggle, without affecting curriculum meaning
or RAG behavior.

**Critical Constraints**:
- Translate presentation ONLY (not stored canonical content)
- Preserve technical accuracy and terminology
- NOT alter retrieval, embeddings, or RAG logic
- Translation is optional and reversible by the user
- Technical terms (ROS 2, Gazebo, Isaac, VLA, etc.) remain in English

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US5]**: Maps to User Story 5 - Read Content in Urdu
- Include exact file paths in descriptions

## Path Conventions

- Frontend: `docs/src/` (Docusaurus)
- Backend: `backend/app/` (FastAPI)
- Specs: `specs/localization/`

---

## Phase 6.0: Entry Validation

**Purpose**: Confirm Phase 5 is complete and locked before proceeding

**Owner**: chief-orchestrator

- [ ] T600 Verify Phase 5 completion and lock status in specs/auth/PHASE-5-LOCK.md
  - Confirm authentication works correctly (signup, signin, signout, me, profile)
  - Confirm personalization is presentation-only
  - Confirm Phase 5 marked LOCKED

**Acceptance Criteria (T600)**:
- Phase 5 lock file exists and shows LOCKED status
- All Phase 5 tasks (T500-T509) marked complete
- Safety audit passed (RAG outputs identical for all users)

**Checkpoint**: Phase 5 verified complete - Phase 6 can proceed

**Dependencies**: Phase 5 LOCKED

---

## Phase 6.1: Translation Specification

**Purpose**: Define translation policy and UX behavior

**Owner**: localization-manager (specs), frontend-platform (UX)

### T601 - Translation Policy Specification

- [ ] T601 [US5] Create translation policy specification in specs/localization/translation-policy.md

**Description**: Define rules governing Urdu translation behavior.

**Must Include**:
- On-demand translation only (not pre-translated)
- No persistence of translated content (translate at runtime)
- Technical term preservation list (ROS 2, Gazebo, Isaac Sim, VLA, NVIDIA, etc.)
- Fallback to English on ambiguity or translation failure
- Translation scope: UI text and chapter content only
- Explicit prohibition of meaning changes

**Acceptance Criteria (T601)**:
- [ ] Policy explicitly forbids meaning changes
- [ ] Technical terms preservation list documented
- [ ] Translation scope clearly defined as UI-only
- [ ] Fallback behavior specified
- [ ] No RAG involvement required

**Dependencies**: T600

---

### T602 - Translation UX Specification

- [ ] T602 [US5] Create translation UX specification in specs/localization/translation-ui.md

**Description**: Define how users interact with the translation feature.

**Must Include**:
- Toggle placement (per-chapter or global)
- Visual language indicator (EN / UR badge)
- Loading state during translation
- Error state if translation fails
- Clear state persistence rules (session vs permanent)
- Accessibility considerations (RTL layout for Urdu)

**Acceptance Criteria (T602)**:
- [ ] User always knows current language state
- [ ] Toggle is reversible instantly
- [ ] Loading states clearly indicated
- [ ] RTL layout requirements documented
- [ ] Mobile responsive behavior specified

**Dependencies**: T601

---

## Phase 6.2: Translation Implementation

**Purpose**: Implement backend endpoint and frontend toggle

**Owner**: backend-rag (API), frontend-platform (UI)

### T603 - Translation API Endpoint

- [ ] T603 [US5] Implement POST /api/translate endpoint in backend/app/api/routes/translate.py

**Description**: Create a standalone translation endpoint that does NOT interact with RAG.

**Implementation Requirements**:
- Create `backend/app/api/routes/translate.py` with POST /api/translate
- Create `backend/app/schemas/translate.py` with request/response schemas
- Create `backend/app/services/translation.py` for translation logic
- Add route to `backend/app/api/routes/__init__.py`

**Request Schema**:
```json
{
  "text": "string (required, max 10000 chars)",
  "target_language": "ur" (enum: ur),
  "preserve_terms": ["ROS 2", "Gazebo", ...] (optional)
}
```

**Response Schema**:
```json
{
  "original_text": "string",
  "translated_text": "string",
  "target_language": "ur",
  "preserved_terms": ["ROS 2", ...],
  "translation_provider": "string"
}
```

**Acceptance Criteria (T603)**:
- [ ] Accepts text + target language
- [ ] Returns translated text only
- [ ] Does NOT access vector store or RAG logic
- [ ] Does NOT import from services/rag.py, services/generation.py, or db/qdrant.py
- [ ] Preserves technical terms (no translation)
- [ ] Returns error on empty or oversized input
- [ ] Rate limiting consideration documented

**Dependencies**: T601

---

### T604 - Frontend Translation Toggle

- [ ] T604 [US5] Implement chapter-level translation toggle in docs/src/components/Translation/

**Description**: Create frontend components for translation control.

**Files to Create**:
- `docs/src/components/Translation/TranslationContext.tsx` - State management
- `docs/src/components/Translation/TranslationToggle.tsx` - Toggle button component
- `docs/src/components/Translation/LanguageIndicator.tsx` - EN/UR badge
- `docs/src/components/Translation/api.ts` - API client for /api/translate
- `docs/src/components/Translation/types.ts` - TypeScript definitions
- `docs/src/components/Translation/styles.module.css` - Component styles
- `docs/src/components/Translation/index.ts` - Exports

**Implementation Requirements**:
- Toggle sends visible chapter text to backend
- Replaces displayed content with translation
- Original English content preserved in React state
- Toggle back restores original English instantly
- Loading spinner during translation
- Error toast on translation failure
- RTL text direction for Urdu content

**Acceptance Criteria (T604)**:
- [ ] Toggle sends only visible text to backend
- [ ] Original English content preserved in memory
- [ ] Instant reversal when toggling back to English
- [ ] Loading state shown during translation
- [ ] Error handling with user-friendly message
- [ ] RTL text direction applied for Urdu
- [ ] Works on mobile (responsive)

**Dependencies**: T602, T603

---

### T604.1 - Integrate Translation Provider into Root

- [ ] T604.1 [US5] Add TranslationProvider to docs/src/theme/Root.tsx

**Description**: Integrate translation context into the app wrapper.

**Acceptance Criteria (T604.1)**:
- [ ] TranslationProvider added after PersonalizationProvider
- [ ] Translation state accessible to all components

**Dependencies**: T604

---

## Phase 6.3: Safety & Accuracy Validation

**Purpose**: Ensure translation does not affect curriculum or RAG

**Owner**: curriculum-architect (accuracy), chief-orchestrator (safety)

### T605 - Technical Terminology Validation

- [ ] T605 [US5] Validate translated content for technical accuracy

**Description**: Create a validation checklist and test translated content.

**Validation Checklist**:
- [ ] ROS 2 terminology correct (node, topic, service, action, launch)
- [ ] NVIDIA Isaac terms preserved (Isaac Sim, Isaac ROS, Omniverse)
- [ ] AI/ML terms correct (VLA, transformer, embedding, inference)
- [ ] Hardware terms correct (Jetson, GPU, CUDA)
- [ ] Simulator terms preserved (Gazebo, URDF, SDF)

**Test Cases**:
1. Translate a paragraph containing "ROS 2 node" - verify "ROS 2" preserved
2. Translate code comments - verify code unchanged
3. Translate chapter intro - verify meaning preserved
4. Translate error message - verify actionable guidance preserved

**Acceptance Criteria (T605)**:
- [ ] Key terms remain correct or transliterated appropriately
- [ ] No semantic drift in translated content
- [ ] Code blocks NOT translated
- [ ] URLs and paths NOT translated

**Dependencies**: T604

---

### T606 - Translation Safety Audit

- [ ] T606 [US5] Audit translation implementation for RAG isolation

**Description**: Verify translation feature does NOT affect RAG behavior.

**Audit Checklist**:
```bash
# Verify NO imports from RAG services in translation module
grep -r "from app.services.rag\|from app.services.generation\|from app.db.qdrant" backend/app/services/translation.py
# Expected: NO MATCHES

grep -r "from app.services.rag\|from app.services.generation\|from app.db.qdrant" backend/app/api/routes/translate.py
# Expected: NO MATCHES

# Verify translation does not modify chat endpoint
diff backend/app/api/routes/chat.py (before Phase 6)
# Expected: NO CHANGES
```

**Safety Verification**:
- [ ] Translation service has NO imports from RAG/generation/qdrant
- [ ] Chat endpoint unchanged from Phase 5
- [ ] Same question in English vs Urdu UI produces identical RAG answer
- [ ] Translation happens ONLY in presentation layer (frontend)
- [ ] Translated text is NOT sent to RAG/LLM

**Acceptance Criteria (T606)**:
- [ ] Translation is UI-only (verified via code audit)
- [ ] RAG outputs identical regardless of UI language
- [ ] No curriculum meaning altered

**Dependencies**: T605

---

## Phase 6.4: Phase Validation & Lock

**Purpose**: Final validation and phase lock

**Owner**: chief-orchestrator

### T607 - Phase 6 Validation & Lock

- [ ] T607 [US5] Validate and lock Phase 6 deliverables

**Description**: Final validation of all Phase 6 tasks and creation of lock file.

**Validation Checklist**:
- [ ] T600: Phase 5 verified complete
- [ ] T601: Translation policy spec complete
- [ ] T602: Translation UX spec complete
- [ ] T603: Translation API endpoint working
- [ ] T604: Frontend toggle working
- [ ] T604.1: TranslationProvider integrated
- [ ] T605: Technical terminology validated
- [ ] T606: Safety audit passed

**Lock File**: Create `specs/localization/PHASE-6-LOCK.md`

**Lock File Contents**:
- Task completion status for T600-T607
- Safety audit results
- Files modified/created list
- Declaration of LOCKED status

**Acceptance Criteria (T607)**:
- [ ] All Phase 6 tasks completed
- [ ] Safety audit passed (T606)
- [ ] Lock file created
- [ ] Phase 6 declared LOCKED

**Dependencies**: T600, T601, T602, T603, T604, T604.1, T605, T606

---

## Dependency Graph

```
T600 (Phase 5 Check)
  │
  ├──> T601 (Translation Policy)
  │      │
  │      ├──> T602 (Translation UX)
  │      │      │
  │      │      └──> T604 (Frontend Toggle)
  │      │             │
  │      │             ├──> T604.1 (Provider Integration)
  │      │             │
  │      │             └──> T605 (Terminology Validation)
  │      │                    │
  │      │                    └──> T606 (Safety Audit)
  │      │                           │
  │      │                           └──> T607 (Lock)
  │      │
  │      └──> T603 (Translation API) ──> T604
  │
  └──────────────────────────────────────> T607
```

## Parallel Execution Opportunities

| Group | Tasks | Reason |
|-------|-------|--------|
| Specs | T601, then T602 | Sequential (UX depends on policy) |
| Implementation | T603 ∥ T602 | API and UX spec can be parallel |
| Validation | T605, then T606 | Sequential (audit after validation) |

---

## Technical Notes

### Translation Provider Options

1. **OpenAI GPT-4** - High quality, may need prompt engineering for terminology
2. **Google Cloud Translation** - Good for Urdu, needs term glossary
3. **Azure Translator** - Custom terminology support
4. **Self-hosted** - Full control, higher latency

**Recommendation**: Use OpenAI with system prompt that preserves technical terms.

### RTL Considerations

- Urdu is written right-to-left (RTL)
- CSS `direction: rtl` and `text-align: right` required
- Mixed content (Urdu + English terms) needs `unicode-bidi: embed`
- Test with actual Urdu users if possible

### Performance Considerations

- Cache translations in session storage (not permanent)
- Translate visible content only (lazy translation)
- Show skeleton loader during translation
- Consider chunking large chapters

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Translation quality issues | Medium | High | Review with Urdu speaker, preserve technical terms |
| RTL layout breaks | Medium | Medium | Test thoroughly, use established RTL CSS patterns |
| API rate limits | Low | Medium | Cache translations, implement retry logic |
| Meaning drift | Medium | High | Explicit term preservation list, validation checklist |

---

## Success Metrics

1. **Functionality**: Toggle works on all chapter pages
2. **Accuracy**: Technical terms preserved (100% of preserved list)
3. **Performance**: Translation completes in < 3 seconds
4. **Safety**: RAG outputs identical in EN/UR UI (verified)
5. **UX**: Users can switch languages instantly

---

## Execution Order Summary

1. **T600** - Verify Phase 5 LOCKED
2. **T601** - Create translation policy spec
3. **T602** + **T603** (parallel) - UX spec + API endpoint
4. **T604** - Frontend toggle implementation
5. **T604.1** - Provider integration
6. **T605** - Terminology validation
7. **T606** - Safety audit
8. **T607** - Phase lock

**Estimated Scope**: 8 tasks, medium complexity

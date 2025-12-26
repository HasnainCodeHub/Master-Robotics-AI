# Tasks: Physical AI & Humanoid Robotics - AI-Native Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md, data-model.md, Phase 4 LOCKED
**Scope**: PHASE 5 — Authentication & Personalization

**User Story**: US4 - Sign Up and Personalize Experience (Priority: P3 - Bonus)

**Objective**: Add user authentication and safe, non-intrusive personalization
to the textbook platform using Better Auth.

**Critical Constraints**:
- Use ONLY Better Auth for authentication
- NOT modify RAG retrieval or reasoning
- NOT change curriculum meaning
- Personalize presentation ONLY (depth, hints, examples)

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US4]**: Maps to User Story 4 - Sign Up and Personalize Experience
- Include exact file paths in descriptions

## Path Conventions

- Frontend: `docs/src/` (Docusaurus)
- Backend: `backend/app/` (FastAPI)
- Specs: `specs/auth/`

---

## Phase 5.0: Entry Validation

**Purpose**: Confirm Phase 4 is complete and locked before proceeding

**Owner**: chief-orchestrator

- [ ] T500 Verify Phase 4 completion and lock status in specs/frontend/PHASE-4-LOCK.md
  - Confirm embedded chatbot works end-to-end
  - Confirm selected-text mode enforced
  - Confirm Phase 4 marked LOCKED

**Acceptance Criteria (T500)**:
- Phase 4 lock file exists and shows LOCKED status
- All Phase 4 tasks marked complete

**Checkpoint**: Phase 4 verified complete - Phase 5 can proceed

---

## Phase 5.1: Authentication Specification

**Purpose**: Define authentication flow and user profile schema

**Owner**: identity-personalization

- [ ] T501 [US4] Create authentication flow specification in specs/auth/signup-flow.md
  - Signup steps using Better Auth
  - Signin steps using Better Auth
  - Session handling (JWT tokens, refresh)
  - Logout behavior
  - Password requirements

**Acceptance Criteria (T501)**:
- Flow uses Better Auth exclusively
- No custom auth logic invented
- Session management clearly defined

---

- [ ] T502 [US4] Create user profile schema specification in specs/auth/user-profile-schema.md
  - Software background field (beginner / intermediate / advanced)
  - Robotics background field (none / basic / hands-on)
  - Hardware access field (simulation-only / Jetson / robot)
  - Field validation rules
  - Privacy considerations

**Acceptance Criteria (T502)**:
- Fields are minimal and relevant
- No sensitive personal data collected
- Aligns with data-model.md UserProfile entity

---

## Phase 5.2: Personalization Rules

**Purpose**: Define how user profile influences content presentation

**Owner**: identity-personalization

- [ ] T503 [US4] Create personalization rules specification in specs/auth/personalization-rules.md
  - Explanation depth adjustments by level
  - Optional hints toggle rules
  - Optional examples by experience level
  - Terminology clarification rules
  - RAG isolation guarantee

**Allowed Adjustments**:
- Explanation depth (beginner: more detail, advanced: concise)
- Optional hints (show/hide based on preference)
- Optional examples (additional for beginners, advanced patterns for experts)
- Terminology clarification (tooltips for beginners)

**Explicitly Forbidden**:
- Removing content
- Changing definitions
- Altering learning objectives
- Influencing RAG retrieval or refusal logic

**Acceptance Criteria (T503)**:
- Rules are presentation-only
- Curriculum meaning is preserved
- RAG behavior unchanged by personalization

**Checkpoint**: All Phase 5 specifications complete

---

## Phase 5.3: Authentication Implementation - Backend

**Purpose**: Implement Better Auth integration in FastAPI backend

**Owner**: identity-personalization

- [ ] T504 [P] [US4] Implement Better Auth configuration in backend/app/core/auth.py
  - Better Auth client setup
  - JWT token configuration
  - Session middleware integration
  - Environment variable configuration

- [ ] T505 [P] [US4] Create User model in backend/app/models/user.py
  - User entity per data-model.md
  - UserProfile entity per data-model.md
  - SQLAlchemy model definitions
  - Relationship mappings

- [ ] T506 [US4] Implement auth routes in backend/app/api/routes/auth.py
  - POST /api/auth/signup (register with profile)
  - POST /api/auth/signin (login)
  - POST /api/auth/signout (logout)
  - GET /api/auth/me (current user)

- [ ] T507 [US4] Implement user profile storage in backend/app/services/user.py
  - Create user service
  - Profile creation on signup
  - Profile retrieval on login
  - Profile update endpoint

- [ ] T508 [US4] Add database migrations for user tables
  - Run: `alembic revision --autogenerate -m "add user tables"`
  - Users table
  - User profiles table
  - Indexes per data-model.md

**Acceptance Criteria (T504-T508)**:
- Signup and signin work
- Sessions persist correctly
- Profile data stored in Neon Postgres
- Backend remains stateless where possible

**Checkpoint**: Backend authentication complete

---

## Phase 5.4: Authentication Implementation - Frontend

**Purpose**: Create signup and signin UI components

**Owner**: frontend-platform

- [ ] T509 [P] [US4] Create auth types in docs/src/components/Auth/types.ts
  - User interface
  - UserProfile interface
  - AuthState interface
  - SignupRequest/SigninRequest types

- [ ] T510 [P] [US4] Create auth styles in docs/src/components/Auth/styles.module.css
  - Form styling
  - Input styling
  - Button styling
  - Error states

- [ ] T511 [US4] Create auth context provider in docs/src/components/Auth/AuthContext.tsx
  - Auth state management
  - Login/logout actions
  - Token storage (localStorage)
  - Auto-refresh logic

- [ ] T512 [US4] Create auth API client in docs/src/components/Auth/api.ts
  - signup() function
  - signin() function
  - signout() function
  - getMe() function

- [ ] T513 [US4] Create signup form component in docs/src/components/Auth/SignupForm.tsx
  - Email/password inputs
  - Software background selector (beginner/intermediate/advanced)
  - Robotics background selector (beginner/intermediate/advanced)
  - Hardware access selector
  - Form validation
  - Error display

- [ ] T514 [US4] Create signin form component in docs/src/components/Auth/SigninForm.tsx
  - Email/password inputs
  - Remember me checkbox
  - Error display
  - Link to signup

- [ ] T515 [US4] Create auth modal component in docs/src/components/Auth/AuthModal.tsx
  - Modal wrapper
  - Tab switching (signin/signup)
  - Close button
  - Backdrop

- [ ] T516 [US4] Create user menu component in docs/src/components/Auth/UserMenu.tsx
  - User avatar/name display
  - Dropdown menu
  - Profile link
  - Signout button

- [ ] T517 [US4] Integrate auth into navbar in docs/src/theme/Navbar/index.tsx
  - Sign In button (when logged out)
  - User menu (when logged in)
  - Swizzle Docusaurus navbar if needed

**Acceptance Criteria (T509-T517)**:
- Clean, minimal UI
- Profile questions included at signup
- Clear success and error states
- Auth state persists across page navigation

**Checkpoint**: Frontend authentication complete

---

## Phase 5.5: Personalization Implementation

**Purpose**: Implement presentation-level personalization based on user profile

**Owner**: frontend-platform + identity-personalization

- [ ] T518 [US4] Create personalization context in docs/src/components/Personalization/PersonalizationContext.tsx
  - User profile state
  - Personalization preferences
  - Toggle on/off functionality

- [ ] T519 [P] [US4] Create personalization styles in docs/src/components/Personalization/styles.module.css
  - Beginner hint boxes
  - Advanced deep-dive sections
  - Collapsible sections
  - Tooltip styling

- [ ] T520 [US4] Create BeginnerHint component in docs/src/components/Personalization/BeginnerHint.tsx
  - Shown only for beginner software level
  - Collapsible by default for others
  - Clear visual distinction

- [ ] T521 [US4] Create AdvancedDeepDive component in docs/src/components/Personalization/AdvancedDeepDive.tsx
  - Shown only for advanced users
  - Collapsible for intermediate
  - Hidden for beginners

- [ ] T522 [US4] Create TermTooltip component in docs/src/components/Personalization/TermTooltip.tsx
  - Hover tooltip for technical terms
  - Shown for beginners
  - Optional for others

- [ ] T523 [US4] Create PersonalizationToggle component in docs/src/components/Personalization/PersonalizationToggle.tsx
  - Toggle switch in UI
  - Enable/disable all personalization
  - Persist preference

- [ ] T524 [US4] Integrate personalization into Root theme in docs/src/theme/Root.tsx
  - Add PersonalizationProvider
  - Wrap app with context
  - Load user profile on auth

**Acceptance Criteria (T518-T524)**:
- Content meaning unchanged
- Personalization can be toggled on/off
- Different experiences for different levels
- Core content always visible

**Checkpoint**: Personalization implementation complete

---

## Phase 5.6: Safety Validation

**Purpose**: Ensure personalization does NOT affect AI reasoning or retrieval

**Owner**: chief-orchestrator

- [ ] T525 [US4] Verify RAG isolation in backend/app/services/rag.py
  - RAG service does NOT access user profile
  - Retrieved chunks identical for all users
  - Refusal logic unchanged

- [ ] T526 [US4] Verify chatbot answers are not personalized in docs/src/components/Chatbot/api.ts
  - Chat requests do NOT include user level
  - Answers are identical regardless of user
  - Only UI presentation differs

- [ ] T527 [US4] Document personalization boundaries in specs/auth/personalization-audit.md
  - What is personalized (presentation)
  - What is NOT personalized (RAG, content)
  - Test cases proving isolation

**Acceptance Criteria (T525-T527)**:
- RAG answers identical for all users
- Only presentation differs
- No backend behavior changes based on user level
- Audit documentation complete

---

## Phase 5.7: Phase Validation & Lock

**Purpose**: Validate and lock Phase 5 deliverables

**Owner**: chief-orchestrator

- [ ] T528 Validate Phase 5 completion
  - Signup and signin functional
  - Profile stored and retrieved
  - Personalization working per rules
  - Safety audit passed

- [ ] T529 Create Phase 5 lock declaration in specs/auth/PHASE-5-LOCK.md
  - List all artifacts created
  - Confirm all tasks complete
  - Document safety validation results
  - Declare Phase 5 LOCKED

**Acceptance Criteria (T528-T529)**:
- All T500-T527 tasks complete
- Auth flow works end-to-end
- Personalization respects boundaries
- Phase 5 officially locked

**Checkpoint**: PHASE 5 COMPLETE - Authentication & Personalization locked

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 5.0 (Entry Validation)
    │
    ▼
Phase 5.1 (Auth Specification)
    │
    ▼
Phase 5.2 (Personalization Rules)
    │
    ├── Phase 5.3 (Backend Implementation)
    │
    └── Phase 5.4 (Frontend Implementation) ←── Can run in parallel
    │
    ▼
Phase 5.5 (Personalization Implementation)
    │
    ▼
Phase 5.6 (Safety Validation)
    │
    ▼
Phase 5.7 (Lock)
```

### Task Dependencies

| Task | Depends On | Can Parallel With |
|------|------------|-------------------|
| T500 | None | - |
| T501 | T500 | - |
| T502 | T501 | - |
| T503 | T502 | - |
| T504-T505 | T503 | Each other (both [P]) |
| T506 | T504, T505 | - |
| T507 | T506 | - |
| T508 | T507 | - |
| T509-T510 | T503 | Each other, T504-T505 |
| T511 | T509 | - |
| T512 | T511 | - |
| T513-T514 | T512 | Each other |
| T515 | T513, T514 | - |
| T516 | T515 | - |
| T517 | T516 | - |
| T518 | T517, T508 | - |
| T519 | T518 | - |
| T520-T522 | T519 | Each other (all [P]) |
| T523 | T520-T522 | - |
| T524 | T523 | - |
| T525-T527 | T524 | Each other |
| T528 | T525-T527 | - |
| T529 | T528 | - |

### Parallel Opportunities

**Backend and Frontend can run in parallel after specs complete**:
```
Phase 5.3 (Backend T504-T508) ──┐
                               ├── Both can run simultaneously
Phase 5.4 (Frontend T509-T517) ┘
```

**Within Backend Implementation**:
```
T504 (Auth config) ──┐
                     ├── Both [P]
T505 (User model) ───┘
```

**Within Frontend Implementation**:
```
T509 (Types) ───┐
                ├── Both [P]
T510 (Styles) ──┘
```

**Personalization Components**:
```
T520 (BeginnerHint) ──┐
T521 (AdvancedDeepDive) ──┼── All [P]
T522 (TermTooltip) ───┘
```

---

## Implementation Strategy

### Sequential Execution (Single Agent)

1. Complete T500 (Entry Validation)
2. Complete T501-T503 (Specifications)
3. Complete T504-T508 (Backend Auth)
4. Complete T509-T517 (Frontend Auth)
5. Complete T518-T524 (Personalization)
6. Complete T525-T527 (Safety Validation)
7. Complete T528-T529 (Lock)

### Parallel Execution (Multiple Agents)

1. Complete T500 (Sequential - entry gate)
2. Complete T501-T503 (Sequential - specs first)
3. Launch T504-T508 and T509-T517 in parallel (Backend + Frontend)
4. Wait for both to complete
5. Complete T518-T524 (Personalization - needs both)
6. Complete T525-T529 (Validation and Lock)

---

## Phase 5 Completion Conditions

Phase 5 is complete when:

- [ ] Authentication flow works (signup → signin → signout)
- [ ] User profile is collected during signup
- [ ] Profile is stored in Neon Postgres
- [ ] Personalization adjusts presentation based on profile
- [ ] Personalization can be toggled on/off
- [ ] RAG answers are IDENTICAL for all users
- [ ] Only presentation differs between users
- [ ] All safety validations pass
- [ ] Phase 5 lock declaration is signed

---

## Artifacts to Create

| Artifact | Task | Path |
|----------|------|------|
| Auth Flow Spec | T501 | specs/auth/signup-flow.md |
| User Profile Schema | T502 | specs/auth/user-profile-schema.md |
| Personalization Rules | T503 | specs/auth/personalization-rules.md |
| Auth Config | T504 | backend/app/core/auth.py |
| User Model | T505 | backend/app/models/user.py |
| Auth Routes | T506 | backend/app/api/routes/auth.py |
| User Service | T507 | backend/app/services/user.py |
| Auth Types | T509 | docs/src/components/Auth/types.ts |
| Auth Styles | T510 | docs/src/components/Auth/styles.module.css |
| Auth Context | T511 | docs/src/components/Auth/AuthContext.tsx |
| Auth API | T512 | docs/src/components/Auth/api.ts |
| Signup Form | T513 | docs/src/components/Auth/SignupForm.tsx |
| Signin Form | T514 | docs/src/components/Auth/SigninForm.tsx |
| Auth Modal | T515 | docs/src/components/Auth/AuthModal.tsx |
| User Menu | T516 | docs/src/components/Auth/UserMenu.tsx |
| Personalization Context | T518 | docs/src/components/Personalization/PersonalizationContext.tsx |
| BeginnerHint | T520 | docs/src/components/Personalization/BeginnerHint.tsx |
| AdvancedDeepDive | T521 | docs/src/components/Personalization/AdvancedDeepDive.tsx |
| TermTooltip | T522 | docs/src/components/Personalization/TermTooltip.tsx |
| PersonalizationToggle | T523 | docs/src/components/Personalization/PersonalizationToggle.tsx |
| Personalization Audit | T527 | specs/auth/personalization-audit.md |
| Phase 5 Lock | T529 | specs/auth/PHASE-5-LOCK.md |

---

## Next Phase (After Phase 5 Lock)

Once Phase 5 is complete and locked:

1. **Phase 6**: Urdu Translation (frontend-platform agent)
2. **Phase 7**: Deployment & Release (devops-github agent)

---

## Notes

- All Phase 5 tasks focus on authentication and personalization
- [P] tasks can run in parallel if multiple agents are available
- Personalization MUST NOT affect RAG behavior
- Better Auth is the ONLY auth solution (no custom implementations)
- Constitution Principle IX (No Hallucination) still applies - personalization is presentation only

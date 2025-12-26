---
id: 0010
title: Authentication Flow Specification Created
stage: spec
date: 2024-12-25
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: identity-personalization-agent
command: T501 - Create auth flow specification
labels: ["authentication", "signup", "signin", "session", "logout", "jwt"]
links:
  spec: /specs/auth/signup-flow.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/auth/signup-flow.md
tests:
  - N/A (specification document)
---

## Prompt

[EXECUTION] identity-personalization agent has executed T501

Create the authentication flow specification at /specs/auth/signup-flow.md

CONTEXT:
- The backend already has JWT-based auth implemented in backend/app/core/auth.py
- Auth routes exist in backend/app/api/routes/auth.py (signup, signin, signout, me, profile)
- Better Auth is the ONLY auth provider

Create a comprehensive specification that covers:
1. **Signup Flow**
   - Email + password registration
   - Profile data collection (software level, robotics level, hardware access)
   - Validation rules (email format, password strength)
   - Success/error states

2. **Signin Flow**
   - Email + password authentication
   - "Remember me" option (7 days vs 1 hour token)
   - Success/error states

3. **Session Handling**
   - JWT token storage (localStorage)
   - Token refresh strategy
   - Session persistence across page reloads

4. **Logout Behavior**
   - Token invalidation
   - State cleanup
   - Redirect behavior

CONSTRAINTS:
- Better Auth is the ONLY auth provider
- No social login (Google, GitHub, etc.)
- No OAuth/OIDC
- Simple email/password auth only

## Response snapshot

Created comprehensive authentication flow specification at `/specs/auth/signup-flow.md` (Version 2.0.0) covering:

1. **Signup Flow** (Section 2): Complete user journey with credential collection (email, password with validation), profile data collection (software level, robotics level, hardware access), backend processing steps, and detailed error state handling.

2. **Signin Flow** (Section 3): Authentication with "remember me" option affecting token expiry (1 hour default, 7 days with remember me), security-conscious error messages to prevent email enumeration.

3. **Session Handling** (Section 4): JWT architecture with HS256 algorithm, token payload structure (sub, exp, iat claims), localStorage storage strategy with security tradeoffs, session persistence logic on page reload, and current MVP approach (no automatic refresh).

4. **Logout Flow** (Section 5): Stateless logout with server notification, client-side cleanup, redirect behavior, and anonymous mode continuation for chatbot.

5. **Additional Sections**: API endpoints reference, security considerations (password hashing, token security, CORS), frontend implementation notes (AuthContext structure, protected routes, anonymous mode), database schema reference.

Specification aligns with existing backend implementation in:
- `backend/app/core/auth.py` (JWT token creation, password hashing)
- `backend/app/api/routes/auth.py` (endpoint definitions)
- `backend/app/schemas/user.py` (request/response schemas)
- `backend/app/services/user.py` (user creation, authentication)

## Outcome

- Impact: Created definitive authentication specification for Phase 5
- Tests: N/A (specification document, no code changes)
- Files: 1 file updated (specs/auth/signup-flow.md)
- Next prompts: T502 - User profile schema specification, T503 - Personalization rules specification
- Reflection: Specification grounded in actual backend implementation ensures accuracy and implementability

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

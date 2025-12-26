---
id: 0010
title: Personalization Rules Specification
stage: spec
date: 2024-12-25
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: identity-personalization-agent
command: T503 - Create personalization rules specification
labels: ["personalization", "presentation-layer", "rag-isolation", "phase5"]
links:
  spec: /specs/auth/personalization-rules.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/auth/personalization-rules.md
tests:
  - N/A (specification document)
---

## Prompt

[EXECUTION] identity-personalization agent has executed T503

Create the personalization rules specification at /specs/auth/personalization-rules.md

CONTEXT:
- This is an educational textbook platform with RAG-based Q&A
- Phase 3 backend has strict RAG safety guarantees
- RAG outputs must be IDENTICAL for all users
- Personalization is PRESENTATION-ONLY

Create a specification that documents:

1. **Allowed Presentation Adjustments**
   - Beginner mode: Show additional explanations, expand acronyms, add context
   - Advanced mode: Concise views, technical terminology, skip basics
   - Hardware-aware hints: Show relevant hardware examples based on user's access

2. **Presentation Personalization Examples**
   - Same RAG answer shown differently:
     - Beginner sees: "ROS 2 (Robot Operating System 2) is a middleware framework that..."
     - Advanced sees: "ROS 2 is a middleware framework that..."
   - Same code example with different comments:
     - Beginner: Extensive inline comments
     - Advanced: Minimal comments

3. **EXPLICIT FORBIDDEN ACTIONS** (NON-NEGOTIABLE)
   - NEVER influence retrieval query
   - NEVER filter or re-rank retrieved chunks based on user profile
   - NEVER modify LLM prompt based on user profile
   - NEVER change the semantic meaning of answers
   - NEVER add facts not in the retrieved content
   - NEVER hide information based on user level

4. **Implementation Rules**
   - Personalization happens AFTER RAG generates the answer
   - Personalization is a pure presentation layer
   - User can toggle personalization on/off
   - Same answer content, different presentation

5. **Audit Requirements**
   - RAG pipeline has NO knowledge of user profile
   - Profile data never sent to embedding or LLM services
   - Personalization code separated from RAG code

## Response snapshot

Created comprehensive personalization rules specification at `/specs/auth/personalization-rules.md` with:

1. **Core Architecture Diagram**: Clear separation between RAG pipeline (user-blind) and presentation layer (profile-aware)

2. **Section 1 - Allowed Adjustments**:
   - Explanation depth based on software/robotics level
   - Optional hints with level-dependent default states
   - Code comment visibility via CSS classes
   - Terminology tooltips with level-aware behavior
   - Hardware-aware highlighting (visual badges only)

3. **Section 2 - Presentation Examples**:
   - Same RAG answer with beginner acronym expansion
   - Same code with different comment visibility
   - Same text with optional tooltips for beginners

4. **Section 3 - Forbidden Actions (NON-NEGOTIABLE)**:
   - RAG query manipulation forbidden
   - Chunk filtering by profile forbidden
   - LLM prompt modification forbidden
   - Semantic meaning changes forbidden
   - Adding external facts forbidden
   - Information hiding forbidden

5. **Section 4 - Implementation Rules**:
   - 6-step personalization timing (profile used only at step 5)
   - Pure presentation layer requirements
   - User toggle specification
   - Code separation between RAG and personalization modules

6. **Section 5 - Audit Requirements**:
   - RAG pipeline isolation verification
   - Profile data isolation from embedding/LLM services
   - Answer identity verification test case
   - Audit logging requirements

## Outcome

- Impact: Comprehensive personalization rules locked, ensuring RAG outputs remain identical for all users
- Tests: Specification includes test cases for answer identity verification
- Files: specs/auth/personalization-rules.md (updated from v1.0 to v1.1)
- Next prompts: Implement presentation layer components, integrate with frontend
- Reflection: Strong emphasis on NON-NEGOTIABLE boundaries aligns with constitution Principle III

## Evaluation notes (flywheel)

- Failure modes observed: None - existing file structure followed
- Graders run and results (PASS/FAIL): N/A (specification phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Verify audit requirements can be enforced at code review time

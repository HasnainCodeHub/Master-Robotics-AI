---
id: "005"
title: Phase 4 Frontend Chatbot Tasks Generated
stage: tasks
date: 2024-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: hackathon-team
command: /sp.tasks
labels: ["phase4", "frontend", "chatbot", "docusaurus", "task-generation"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/phase4-tasks.md
tests:
  - null
---

## Prompt

/sp.tasks

Scope: PHASE 4 — Frontend Platform & Embedded Chatbot
Objective: Embed the RAG chatbot into the Docusaurus-based textbook UI and expose AI interaction safely, transparently, and curriculum-aligned.

This phase MUST:
- Use ONLY Docusaurus + JavaScript/TypeScript
- Integrate with the FastAPI backend from Phase 3
- Avoid authentication logic (Phase 5)
- Avoid translation logic (Phase 6)
- Make RAG boundaries visible to the user

## Response snapshot

Generated comprehensive Phase 4 tasks.md with 49 tasks organized across 7 phases:

1. **Entry Validation (4 tasks)**: Verify Phase 3 backend is ready
2. **Frontend Specs (3 tasks)**: chatbot-ui.md, selected-text-ui.md, source-display.md
3. **Chatbot UI Implementation (16 tasks)**: Components, API integration, state management
4. **Selected-Text Mode (7 tasks)**: Text selection capture and scoped queries
5. **RAG Transparency (8 tasks)**: Source display, refusal UX, loading states
6. **Polish & Accessibility (5 tasks)**: Keyboard nav, ARIA, mobile, dark mode
7. **Validation & Lock (6 tasks)**: Manual testing and PHASE4-COMPLETE.md

Key deliverables:
- Chatbot component hierarchy in docs/src/components/Chatbot/
- API service calling POST /api/chat
- useChat and useTextSelection hooks
- ChatbotWrapper for global availability
- Source references display with chapter links
- Refusal messages styled as info (not error)
- Selected-text mode with visual scope indicator

Frontend Safety:
- NO LLM calls from frontend (backend API only)
- selected_text sent exactly as user selected
- Refusal is 200 response, not error

## Outcome

- Impact: Created complete Phase 4 task breakdown for Frontend Platform & Embedded Chatbot
- Tests: No tests generated (not explicitly requested)
- Files: 1 file created (specs/001-physical-ai-textbook/phase4-tasks.md)
- Next prompts: Execute Phase 4 tasks starting with T400, or run `/sp.implement`
- Reflection: Task organization follows component architecture with clear UI/API separation

## Evaluation notes (flywheel)

- Failure modes observed: None - all design documents were available
- Graders run and results (PASS/FAIL): N/A - task generation workflow
- Prompt variant (if applicable): Standard /sp.tasks with phase scope
- Next experiment: Consider adding visual design mockups or Figma references

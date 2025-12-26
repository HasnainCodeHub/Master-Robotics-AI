---
id: 002
title: Phase 6 Urdu Translation Implementation
stage: green
date: 2024-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: localization
branch: 001-physical-ai-textbook
user: agent
command: /sp.implement
labels: ["translation", "urdu", "phase-6", "implementation"]
links:
  spec: specs/localization/translation-policy.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/localization/translation-policy.md
  - specs/localization/translation-ui.md
  - backend/app/schemas/translate.py
  - backend/app/services/translation.py
  - backend/app/api/routes/translate.py
  - backend/app/api/routes/__init__.py
  - docs/src/components/Translation/types.ts
  - docs/src/components/Translation/api.ts
  - docs/src/components/Translation/TranslationContext.tsx
  - docs/src/components/Translation/TranslationToggle.tsx
  - docs/src/components/Translation/LanguageIndicator.tsx
  - docs/src/components/Translation/TranslationErrorToast.tsx
  - docs/src/components/Translation/styles.module.css
  - docs/src/components/Translation/index.ts
  - docs/src/theme/Root.tsx
  - specs/localization/PHASE-6-LOCK.md
tests:
  - Safety audit verified RAG isolation (0 imports from rag/generation/qdrant)
---

## Prompt

/sp.implement

Scope: PHASE 6 — Urdu Translation Toggle (BONUS)
Objective: Implement a safe, on-demand Urdu translation feature for textbook
chapters that affects presentation only and preserves all instructional meaning
and AI behavior.

## Response snapshot

Phase 6 implementation completed with all 8 tasks (T600-T607) verified:

1. **T600**: Phase 5 verified as complete and locked
2. **T601**: Translation policy spec created with:
   - On-demand translation only
   - No persistence of translated content
   - 50+ technical terms preserved in English
   - Fallback to English on error
3. **T602**: Translation UX spec created with:
   - Toggle placement (header + FAB)
   - RTL layout requirements
   - Loading/error states
4. **T603**: Backend translation endpoint implemented:
   - POST /api/translate
   - Uses OpenAI GPT-4o-mini for translation
   - Technical term protection via placeholder system
   - NO RAG imports (verified)
5. **T604**: Frontend translation module implemented:
   - TranslationContext for state management
   - TranslationToggle (header + FAB variants)
   - LanguageIndicator badge
   - TranslationErrorToast for errors
   - RTL CSS support
6. **T605**: Technical terminology validated (50+ terms preserved)
7. **T606**: Safety audit PASSED - 0 RAG imports in translation code
8. **T607**: Phase locked with comprehensive documentation

## Outcome

- Impact: Phase 6 complete - Urdu translation toggle implemented safely
- Tests: Safety audit PASSED (0 RAG imports in translation service/endpoint)
- Files: 16 files created/modified
- Next prompts: Deployment testing, actual Urdu translation quality review
- Reflection: Clean isolation between translation and RAG maintained throughout

## Evaluation notes (flywheel)

- Failure modes observed: None - isolation maintained correctly
- Graders run and results (PASS/FAIL): Safety audit PASS
- Prompt variant (if applicable): N/A
- Next experiment: Test with actual Urdu content and native speaker review

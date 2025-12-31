---
id: 0005
title: Refactor Sign-Out Modal to Use React Portal
stage: refactor
date: 2026-01-01
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: developer
command: direct-prompt
labels: ["modal", "portal", "auth", "ui", "accessibility"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/src/components/Auth/ConfirmDialog.tsx
tests:
  - Manual verification of modal centering
---

## Prompt

TITLE:
Refactor Sign-Out Confirmation Modal to Professional Centered Dialog

CONTEXT:
The current sign-out confirmation UI appears visually misplaced and overlays the hero section in an unprofessional manner.
This breaks visual hierarchy and user trust.
We need a polished, accessible, centered confirmation modal consistent with modern SaaS UX patterns.

OBJECTIVE:
Replace the existing sign-out toggle/overlay with a proper centered modal dialog that:
- Appears in the exact center of the viewport
- Uses backdrop dimming
- Clearly communicates action intent
- Prevents background interaction
- Matches the project's dark, gradient-based design system

REQUIREMENTS:
1. MODAL POSITIONING - Fixed, centered horizontally and vertically
2. BACKDROP & FOCUS - Semi-transparent dark backdrop with blur, scroll lock, focus trap, ESC key
3. VISUAL DESIGN - Rounded container, soft shadow, clear title and actions
4. BUTTON HIERARCHY - Primary destructive action, secondary cancel
5. INTERACTION FLOW - Open on sign-out click, confirm executes logout, cancel/backdrop closes
6. RESPONSIVENESS - Desktop, tablet, mobile support
7. CODE QUALITY - Clean, maintainable

## Response snapshot

Analyzed existing ConfirmDialog implementation and identified the root cause: the modal was being rendered inside `userMenuContainer` which has `position: relative`, creating a stacking context that caused z-index and positioning issues.

Solution implemented:
1. Added React Portal support via `createPortal` from `react-dom`
2. Added SSR-compatible portal container initialization using `useState` and `useEffect`
3. Modal now renders directly into `document.body`, bypassing any parent stacking contexts
4. All existing features preserved: focus trapping, ESC key handling, body scroll locking, backdrop click, keyboard accessibility

The existing CSS was already well-designed with proper centering, backdrop blur, responsive breakpoints, and professional styling.

## Outcome

- Impact: Fixed modal positioning issue by using React Portal pattern
- Tests: Manual verification required for modal centering across viewport sizes
- Files: 1 file modified (ConfirmDialog.tsx)
- Next prompts: Test on production build, verify SSR compatibility
- Reflection: React Portals are essential for modal components to avoid stacking context issues from parent containers

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward portal implementation
- Graders run and results (PASS/FAIL): N/A - manual verification
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider extracting portal logic into reusable hook

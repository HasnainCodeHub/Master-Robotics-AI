---
id: 006
title: User Profile Schema Specification
stage: spec
date: 2024-12-25
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: identity-personalization-agent
command: Create user profile schema specification
labels: ["auth", "personalization", "user-profile", "schema", "phase5"]
links:
  spec: specs/auth/user-profile-schema.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/auth/user-profile-schema.md
tests:
  - N/A (specification document)
---

## Prompt

Create the user profile schema specification at /specs/auth/user-profile-schema.md

CONTEXT:
- The backend already has models in backend/app/models/user.py with:
  - User: id, email, password_hash, timestamps
  - UserProfile: user_id, software_level, robotics_level, hardware_access, personalization_enabled
  - ExperienceLevel enum: beginner, intermediate, advanced
  - HardwareAccess enum: simulation_only, jetson_device, physical_robot

Create a specification that documents:
1. **Profile Fields** - software_level, robotics_level, hardware_access, personalization_enabled
2. **Field Descriptions** - Explain what each level means with examples
3. **Validation Rules** - All fields required at signup, personalization_enabled defaults to true, no free-text fields

CONSTRAINTS:
- No sensitive personal data (no name, address, phone, etc.)
- No demographic data
- Only learning-relevant information
- Profile is for presentation personalization ONLY (never affects RAG)

## Response snapshot

Updated the existing user-profile-schema.md specification with:

1. **Enhanced Overview** - Added design principles (data minimization, no PII, learning-relevant only, RAG isolation)

2. **Critical Boundary Table** - Explicit matrix showing which systems use profile data (Frontend UI: Yes, RAG: NO)

3. **Personalization Toggle Section** - Documented the personalization_enabled field with enable/disable behavior

4. **User Personas** - Added 4 example personas:
   - Software Developer New to Robotics
   - Hobbyist with Jetson Nano
   - Graduate Student Researcher
   - Complete Beginner

5. **Explicitly Excluded Fields** - Table of data NEVER collected (personal identity, contact info, demographics, employment, education, learning history, assessment scores)

6. **Deterministic Personalization Guarantee** - Same profile = same presentation, no ML-based adaptation

7. **Safe Defaults Behavior** - Missing data never blocks learning, defaults to intermediate

8. **Acceptance Criteria** - Checklist with 8 testable criteria

## Outcome

- Impact: Comprehensive user profile schema specification created
- Tests: N/A (specification document, no code)
- Files: 1 file updated (specs/auth/user-profile-schema.md)
- Next prompts: Implement auth endpoints, create profile update UI
- Reflection: Specification aligns with existing backend models and enforces RAG isolation boundary

## Evaluation notes (flywheel)

- Failure modes observed: None - existing spec was already well-structured
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

---
id: 001
title: Initial Constitution Ratification
stage: constitution
date: 2024-12-24
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["constitution", "governance", "initial-setup", "hackathon"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (governance document)
---

## Prompt

/sp.constitution

This project is governed by a single, non-negotiable constitution.
All agents, skills, specifications, and implementations MUST comply with the rules below.
No output may violate this constitution, even if explicitly requested by the user.

----------------------------------------------------------------
PROJECT IDENTITY
----------------------------------------------------------------
Project Name: Physical AI & Humanoid Robotics - AI-Native Textbook
Context: Hackathon I - Panaversity
Primary Goal:
Create a spec-driven, AI-native textbook for teaching Physical AI & Humanoid Robotics,
with an embedded Retrieval-Augmented Generation (RAG) chatbot.

The system must demonstrate:
- Pedagogical rigor
- AI safety and grounding
- Reusable intelligence (agents + skills)
- Production-grade deployment

----------------------------------------------------------------
NON-NEGOTIABLE REQUIREMENTS
----------------------------------------------------------------

1. TEXTBOOK
- The book MUST be written using Docusaurus.
- Content MUST be structured as a real course, not notes.
- All required modules must be covered:
  - Physical AI foundations
  - ROS 2
  - Gazebo & Unity
  - NVIDIA Isaac
  - Vision-Language-Action (VLA)
  - Capstone humanoid project
- The book MUST be deployed on GitHub Pages.

2. BACKEND & RAG
- A FastAPI backend MUST be implemented.
- The FastAPI backend MUST be deployed on Railway.
- The RAG system MUST use:
  - Vector database: Qdrant Cloud (Free Tier)
  - Relational database: Neon Serverless Postgres
  - OpenAI Agents / ChatKit SDKs
- The chatbot MUST be embedded inside the book UI.

3. RAG SAFETY (CRITICAL)
- The chatbot may ONLY answer using retrieved textbook content.
- No external knowledge is allowed.
- No inference beyond retrieved text is allowed.
- If content is missing or insufficient, the chatbot MUST explicitly refuse.
- "Answer from selected text only" mode MUST strictly limit context to user-selected text.
- A refusal is ALWAYS preferred over a partial or speculative answer.

4. AI-NATIVE DEVELOPMENT
- Claude Code MUST be used for development.
- Spec-Kit Plus discipline MUST be followed.
- All major system behavior MUST be defined in specs before implementation.

----------------------------------------------------------------
SPEC-KIT PLUS GOVERNANCE
----------------------------------------------------------------

1. SPECS
- Specs are the source of truth.
- Specs MUST be written in Markdown.
- Specs MUST live under the /specs directory.
- Specs MUST be updated BEFORE changing implementation behavior.

Required spec domains:
- /specs/constitution.md
- /specs/curriculum/
- /specs/rag/
- /specs/auth/
- /specs/frontend/

2. /sp.* PROTOCOL
- /sp.constitution defines immutable rules.
- /sp.clarify is used to remove ambiguity.
- /sp.plan is used to define sequencing and architecture.
- /sp.tasks is used to define atomic, verifiable work.
- Implementation MUST NOT occur during clarify, plan, or tasks phases.

----------------------------------------------------------------
AGENTS & AUTHORITY MODEL
----------------------------------------------------------------

Agents are reasoning roles, not executors.

The following agents are authoritative and MUST NOT exceed their mandate:

- curriculum-architect
- book-content-writer
- backend-rag
- frontend-platform
- identity-personalization
- project-bootstrap
- devops-github
- chief-orchestrator

----------------------------------------------------------------
SKILLS GOVERNANCE
----------------------------------------------------------------

- Skills are reusable execution patterns.
- Skills may ONLY be used by agents that explicitly declare them.
- Agents MUST NOT use skills outside their declared list.
- Skills MUST NOT make curriculum, content, or architectural decisions.

----------------------------------------------------------------
AUTHENTICATION & PERSONALIZATION (BONUS)
----------------------------------------------------------------

- Better Auth for signup/signin
- User profiling for software and hardware background
- Personalization may adjust depth and examples but MUST NOT alter content

----------------------------------------------------------------
URDU TRANSLATION (BONUS)
----------------------------------------------------------------

- Translation button per chapter
- Prose only, code unchanged
- Consistent technical terminology

----------------------------------------------------------------
DEPLOYMENT RULES
----------------------------------------------------------------

- Book: GitHub Pages
- Backend: Railway (FastAPI)
- Environment variables for secrets
- Reproducible deployments

----------------------------------------------------------------
ANTI-PATTERNS (STRICTLY FORBIDDEN)
----------------------------------------------------------------

- Hallucinated content
- Silent scope creep
- AI answers without retrieved evidence
- Mixing agent responsibilities

## Response snapshot

Created constitution v1.0.0 with 10 core principles:
1. Spec-Driven Development
2. Curriculum Authority
3. RAG Safety (NON-NEGOTIABLE)
4. Physical Grounding
5. Agent Mandate Boundaries
6. Content Structure
7. Test-First Discipline
8. Simplicity & Minimal Change
9. No Hallucination
10. Deployment Reproducibility

Plus sections for: Non-Negotiable Requirements, Agents & Authority Model (8 agents), Skills Governance, Authentication & Personalization, Urdu Translation, Deployment Rules, Anti-Patterns, Spec-Kit Plus Protocol, and Governance.

## Outcome

- Impact: MAJOR - Initial constitution ratified for project governance
- Tests: N/A (governance document, not code)
- Files: 1 file created (.specify/memory/constitution.md)
- Next prompts: /sp.plan for curriculum architecture, agent definitions refinement
- Reflection: Constitution successfully captures all hackathon requirements with clear boundaries

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual review PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate against template requirements

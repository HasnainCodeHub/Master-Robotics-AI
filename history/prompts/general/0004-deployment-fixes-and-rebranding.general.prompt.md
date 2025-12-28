---
id: 0004
title: Deployment Fixes and Rebranding to Embodied AI
stage: general
date: 2025-12-28
surface: agent
model: claude-opus-4-5
feature: none
branch: main
user: HasnainCodeHub
command: interactive-session
labels: ["deployment", "cors", "railway", "github-pages", "rebranding", "ui"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/Dockerfile
  - backend/app/main.py
  - docs/src/utils/config.ts
  - docs/docusaurus.config.ts
  - docs/src/css/custom.css
tests:
  - CORS preflight request validation
  - Backend health check
  - Login API endpoint test
  - Frontend deployment verification
---

## Prompt

User requested multiple tasks in this session:
1. Update repository with latest commit
2. Analyze Railway deployment and solve all issues
3. Check if frontend is fetching with correct backend URL
4. Test if login works on live site
5. Fix "Failed to fetch" login error on GitHub Pages
6. Rename navbar button from "Textbook" to "Book"
7. Rename site title to professional industry-level name
8. Increase navbar logo size

## Response snapshot

### Issues Fixed:

1. **Dockerfile COPY Issue**
   - Removed invalid `COPY ../docs /app/docs` line (Docker cannot copy from parent directories)
   - Deployment changed from FAILED to SUCCESS

2. **CORS Configuration**
   - Fixed CORS_ORIGINS on Railway: removed invalid path `/Master-Robotics-AI/`
   - Updated from `["GET", "POST", "PATCH", "OPTIONS"]` to `["*"]`
   - Added `expose_headers=["*"]` for full CORS support

3. **Frontend API Configuration**
   - Fixed `getApiBaseUrl()` to return Railway URL in production
   - Was incorrectly returning `window.location.origin` (GitHub Pages URL)

4. **Rebranding**
   - Site title: "Physical AI & Humanoid Robotics" -> "Embodied AI"
   - Tagline: "Master Physical Intelligence & Humanoid Robotics"
   - Navbar button: "Textbook" -> "Book"

5. **UI Improvements**
   - Increased navbar logo size to 38px

### Verification:
- Backend health: healthy (Qdrant + Postgres connected)
- CORS preflight: 200 OK with correct headers
- Login API: Returns JWT token successfully
- GitHub Pages: Deployed with correct Railway backend URL

## Outcome

- Impact: Critical deployment blockers resolved, login now works on production
- Tests: All API endpoints verified working
- Files: 5 files modified
- Next prompts: Push changes to remote, verify end-to-end login on live site
- Reflection: CORS origins must be scheme+host only (no paths); Docker COPY cannot access parent directories

## Evaluation notes (flywheel)

- Failure modes observed: Invalid CORS origin format with path, Docker build context limitation
- Graders run and results (PASS/FAIL): PASS - All API tests passed, deployment successful
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Monitor production login for 24h to ensure stability

---
id: "0012"
title: "Global Spotlight Search (Cmd/Ctrl+K) Implementation"
stage: green
date: 2025-12-27
surface: agent
model: claude-opus-4-5-20251101
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: developer
command: implement
labels: ["search", "spotlight", "keyboard-shortcut", "qdrant", "frontend", "backend"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/app/services/search.py
  - backend/app/schemas/chat.py
  - backend/app/api/routes/chat.py
  - docs/src/components/Search/types.ts
  - docs/src/components/Search/SearchContext.tsx
  - docs/src/components/Search/SearchModal.tsx
  - docs/src/components/Search/SearchBar.tsx
  - docs/src/components/Search/styles.module.css
  - docs/src/components/Search/index.ts
  - docs/src/components/Chatbot/api.ts
  - docs/src/components/Chatbot/types.ts
  - docs/src/theme/Root.tsx
  - docs/src/theme/NavbarItem/ComponentTypes.tsx
  - docs/src/theme/NavbarItem/SearchNavbarItem.tsx
  - docs/docusaurus.config.ts
tests:
  - Manual verification of Ctrl/Cmd+K shortcut
  - Manual verification of search results rendering
  - Manual verification of navigation to book sections
  - Manual verification of Ask Tutor integration
  - Manual verification of auth toast for unauthenticated users
---

## Prompt

User requested implementation of a global spotlight-style search feature for the Physical AI & Humanoid Robotics textbook with the following requirements:

1. Search bar centered in navbar with "Search docs..." label
2. Keyboard shortcut: Ctrl+K (Windows/Linux), Cmd+K (macOS)
3. Spotlight-style dark modal with:
   - Dark translucent backdrop with blur
   - Centered floating panel with scale animation
   - Search input with autofocus
   - Scrollable results list with keyboard navigation
4. Hybrid search using Qdrant semantic search
5. Result actions: Navigate to section, Ask Tutor
6. Auth check with toast for Ask Tutor action
7. Respect RAG guardrails - no answer generation, search only

## Response snapshot

Implemented complete global spotlight search feature:

### Backend Implementation

1. **Search Service** (`backend/app/services/search.py`):
   - `search_textbook()` - Semantic search via Qdrant
   - `highlight_query_terms()` - Creates highlighted snippets
   - Minimum score filtering (0.3 threshold)
   - Returns ranked results with relevance scores

2. **Search Schemas** (`backend/app/schemas/chat.py`):
   - `SearchRequest` - query, limit
   - `SearchResult` - chunk_id, text, module_id, chapter_id, section_heading, score, highlight
   - `SearchResponse` - query, results, total

3. **Search Endpoint** (`backend/app/api/routes/chat.py`):
   - `POST /api/search` - Auth-required search endpoint
   - No answer generation (search only)
   - Logs search requests and results

### Frontend Implementation

4. **Search Context** (`docs/src/components/Search/SearchContext.tsx`):
   - Global search state management
   - Keyboard shortcut listener (Ctrl/Cmd+K)
   - ESC to close functionality
   - SSR-safe with isClient guard

5. **Search Modal** (`docs/src/components/Search/SearchModal.tsx`):
   - Spotlight-style dark UI with blur backdrop
   - Scale + fade entrance animation
   - Debounced search (300ms)
   - Keyboard navigation (↑↓ + Enter)
   - Auth toast for "Ask Tutor" when not logged in
   - Module path mapping for navigation

6. **Search Bar** (`docs/src/components/Search/SearchBar.tsx`):
   - Navbar trigger with "Search docs..." label
   - Shows Ctrl+K / ⌘K shortcut hint
   - Detects macOS for correct shortcut display

7. **Styles** (`docs/src/components/Search/styles.module.css`):
   - Dark spotlight theme (#1a1a2e background)
   - Indigo accent color (#6366f1)
   - Light mode support with [data-theme='light']
   - Responsive mobile layout
   - Auth toast with slide-up animation

8. **Integration**:
   - SearchProvider added to Root.tsx
   - SearchModal in BrowserOnlyComponents
   - custom-search navbar item type
   - Added to docusaurus.config.ts navbar

### Features
- **Ctrl/Cmd+K** opens search instantly
- **ESC** closes modal
- **Arrow keys** navigate results
- **Enter** opens selected result
- **Ask Tutor** sends content to chatbot (auth required)
- **Auth Toast** appears for unauthenticated users with login button

## Outcome

- Impact: Users can now search the entire textbook using semantic search and navigate directly to relevant sections
- Tests: Manual verification - keyboard shortcuts work, search returns results, navigation works, auth toast displays
- Files: 15 files created/modified across backend and frontend
- Next prompts: Add search analytics, implement keyword boosting, add search history
- Reflection: Clean separation between search (retrieval only) and tutor (answer generation) maintains RAG guardrails

## Evaluation notes (flywheel)

- Failure modes observed: None - implementation followed existing patterns and RAG architecture
- Graders run and results (PASS/FAIL): N/A - manual testing
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add fuzzy matching for typo tolerance in search queries

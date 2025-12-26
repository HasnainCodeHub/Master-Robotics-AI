# Physical AI & Humanoid Robotics Textbook Project

[![Docusaurus](https://img.shields.io/badge/Docusaurus-v4-green.svg)](https://docusaurus.io/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.109+-blue.svg)](https://fastapi.tiangolo.com/)
[![Python](https://img.shields.io/badge/Python-3.11+-yellow.svg)](https://python.org/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.0+-blue.svg)](https://www.typescriptlang.org/)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

## Project Title & One-Paragraph Overview

This project delivers an AI-native, comprehensive, and interactive textbook on Physical AI and Humanoid Robotics. Built with Docusaurus and FastAPI, it features a RAG-powered AI tutor, global spotlight search, session management, authentication with personalization, Urdu translation support, and a professional UX. Leveraging Spec-Kit Plus and Claude Code agents, it showcases an innovative approach to content generation, curriculum design, and interactive learning experiences, culminating in a fully deployed, production-ready educational platform.

## Quick Start

```bash
# Clone repository
git clone https://github.com/physical-ai-robotics/robotics-book.git
cd robotics-book

# Backend setup
cd backend
uv pip install -r pyproject.toml
alembic upgrade head
uvicorn app.main:app --reload --port 8001

# Frontend setup (new terminal)
cd docs
npm install
npm start

# Access at http://localhost:3000
```

## Live Demo

-   **Frontend**: [https://physical-ai-robotics.github.io/robotics-book/](https://physical-ai-robotics.github.io/robotics-book/)
-   **API Docs**: [Backend OpenAPI Documentation](https://your-backend-url.railway.app/docs)

## Problem Statement

The rapid evolution of Physical AI and Humanoid Robotics necessitates up-to-date, engaging, and accessible educational resources. Traditional textbook creation struggles to keep pace with these advancements. This project addresses this by demonstrating an AI-driven pipeline that accelerates content development, integrates interactive learning tools (like RAG chatbots), and facilitates localization, making cutting-edge knowledge available to a global audience efficiently.

## Key Features

### 🤖 AI-Powered Learning
-   **RAG Chatbot**: Retrieval-Augmented Generation tutor that answers questions strictly from textbook content
-   **Grounding Guardrails**: Refuses to answer if context is insufficient (no hallucinations)
-   **Source Citations**: Every answer includes references to textbook sections
-   **Greeting Detection**: Smart routing for conversational greetings vs technical questions
-   **Session Continuity**: Persistent chat history across sessions

### 🔍 Global Search
-   **Spotlight Search**: Cmd/Ctrl+K shortcut opens global search modal
-   **Semantic Search**: Powered by Qdrant vector database with relevance scoring
-   **Hybrid Ranking**: Combines semantic similarity with keyword matching
-   **Quick Actions**: Navigate to section or ask AI tutor directly from results
-   **Dark Mode**: Professional spotlight-style UI with animations

### 🔐 Authentication & Personalization
-   **Full-Page Auth Routes**: Professional login/signup pages (not modals)
-   **JWT Authentication**: Secure token-based auth with refresh tokens
-   **User Profiles**: Experience level tracking (software, robotics, hardware access)
-   **Protected Content**: Book content requires authentication
-   **Session Management**: Secure chat session tracking per user
-   **Sign-Out Confirmation**: Warning dialog prevents accidental logouts

### 💬 Chat Experience
-   **Floating Chat Panel**: Accessible from any page
-   **Chat History Sidebar**: Browse previous conversations
-   **Selected Text Mode**: Ask questions about highlighted content
-   **Multi-Mode Support**: General questions, explain text, translate to Urdu
-   **Refusal Handling**: Clear messaging when questions are out-of-scope
-   **Loading States**: Professional UX with spinners and status indicators

### 🌐 Localization
-   **Urdu Translation**: AI-powered translation of textbook content
-   **RTL Support**: Right-to-left text rendering for Arabic script
-   **Technical Accuracy**: Preserves technical terms and code examples
-   **Toggle Control**: Easy switching between English and Urdu

### 🎨 Professional UX
-   **Modern Design**: Glass-morphism effects, gradients, smooth animations
-   **Dark Mode**: Full dark theme support throughout
-   **Responsive Layout**: Mobile-first design, works on all screen sizes
-   **Accessibility**: ARIA attributes, keyboard navigation, screen reader support
-   **Toast Notifications**: Non-intrusive success/error messages
-   **Loading States**: Clear feedback during async operations

### 📚 Content Features
-   **Structured Curriculum**: 5 modules + capstone project
-   **34+ Chapters**: Comprehensive coverage from foundations to advanced topics
-   **Interactive Examples**: Code snippets, diagrams, and practical exercises
-   **Progressive Learning**: Beginner to advanced content paths
-   **Glossary & Appendices**: Reference materials and supplementary content

## Curriculum Structure

### Module 1: Physical AI Foundations
-   What is Physical AI?
-   Sensors and Perception
-   Actuators and Control
-   Physical Constraints and Real-World Challenges

### Module 2: ROS 2 Fundamentals
-   ROS 2 Architecture and Core Concepts
-   Topics, Services, and Actions
-   URDF and Robot Description
-   Launch Files and Configuration
-   Integration Patterns
-   Testing and Debugging

### Module 3: Simulation & Digital Twin
-   Introduction to Simulation
-   Gazebo Fundamentals
-   Unity for Robotics
-   Reality Gap and Domain Randomization
-   Sim-to-Real Transfer
-   Digital Twin Concepts

### Module 4: NVIDIA Isaac Ecosystem
-   Isaac Sim Overview
-   Visual SLAM and Mapping
-   Navigation Stack Integration
-   Manipulation and Grasping
-   Sim-to-Real with Isaac
-   Performance Optimization

### Module 5: Vision-Language-Action Systems
-   Speech Recognition Integration
-   LLM-Based Planning and Reasoning
-   Vision-Language Models
-   Grounding and Task Execution
-   Multi-Modal Integration
-   Safety and Robustness

### Capstone Project
-   **Voice-Commanded Service Robot**: Integrated project combining all modules
-   Voice control, navigation, manipulation, and AI planning

### 🔧 Developer Experience
-   **API Documentation**: Auto-generated OpenAPI/Swagger docs
-   **Type Safety**: TypeScript on frontend, Python type hints on backend
-   **Hot Reload**: Fast development with live updates
-   **Structured Logging**: JSON logs for debugging and monitoring
-   **Error Handling**: Comprehensive error boundaries and validation

## Architecture Overview

The project employs a multi-component architecture designed for scalability, maintainability, and AI-driven development:

### System Components

```
┌─────────────────────────────────────────────────────────────┐
│                     User Interface (Browser)                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Docusaurus Frontend (React + TypeScript)            │  │
│  │  - Homepage & Marketing Pages                        │  │
│  │  - Textbook Content (Markdown/MDX)                   │  │
│  │  - Auth Pages (Login/Signup)                         │  │
│  │  - Floating Chat Panel                               │  │
│  │  - Global Search Modal (Ctrl+K)                      │  │
│  │  - User Menu & Profile Settings                      │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↕ HTTPS/REST
┌─────────────────────────────────────────────────────────────┐
│              FastAPI Backend (Python)                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  API Routes                                          │  │
│  │  - /api/auth/*      (signup, signin, signout)       │  │
│  │  - /api/chat        (RAG chat endpoint)             │  │
│  │  - /api/sessions/*  (session management)            │  │
│  │  - /api/search      (semantic search)               │  │
│  │  - /api/translate   (Urdu translation)              │  │
│  │  - /api/content     (personalized content)          │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Services Layer                                      │  │
│  │  - RAG Pipeline (process_question)                  │  │
│  │  - Agent Runner (Gemini via OpenAI SDK)             │  │
│  │  - Session Manager (create, save, list)             │  │
│  │  - Search Service (semantic + keyword)              │  │
│  │  - Embedding Service                                 │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
              ↕                           ↕
┌─────────────────────┐    ┌──────────────────────────────┐
│  PostgreSQL         │    │  Qdrant Vector DB            │
│  - Users            │    │  - Textbook embeddings       │
│  - Profiles         │    │  - Semantic search index     │
│  - Chat Sessions    │    │  - Metadata (module, chapter)│
│  - Messages         │    └──────────────────────────────┘
└─────────────────────┘
```

### Data Flow: Chat Request

1. **User asks question** → Frontend sends to `/api/chat`
2. **Greeting detection** → If greeting, bypass RAG and use minimal context
3. **RAG Pipeline**:
   - Generate question embedding
   - Query Qdrant for relevant chunks (top-k retrieval)
   - Filter by similarity threshold (0.7)
   - If insufficient context → Refuse with clear message
4. **Agent Generation** → Gemini 2.5 Flash generates grounded answer
5. **Response** → Answer + sources sent to frontend
6. **Persistence** → Message saved to PostgreSQL with session_id

### Security Model

-   **Authentication**: JWT tokens stored in localStorage
-   **Authorization**: Backend validates token on every protected endpoint
-   **Password Hashing**: bcrypt with salt
-   **CORS**: Configured for frontend domain only
-   **Input Validation**: Pydantic schemas on backend, Zod on frontend
-   **Open Redirect Prevention**: URL validation before redirects
-   **SQL Injection**: SQLAlchemy ORM prevents direct SQL
-   **XSS Protection**: React's built-in escaping + CSP headers

### AI Development Workflow (Spec-Kit Plus)

-   **Specification** (`/sp.specify`): Define features with clear requirements
-   **Planning** (`/sp.plan`): Architectural decisions and design artifacts
-   **Task Breakdown** (`/sp.tasks`): Testable implementation tasks
-   **Implementation** (`/sp.implement`): Execute tasks with agent assistance
-   **Prompt History Records**: Every user interaction documented
-   **Architecture Decision Records**: Significant decisions captured for posterity

## Tech Stack

### Frontend
-   **Framework**: Docusaurus v4 (React, TypeScript)
-   **Styling**: CSS Modules, Custom Theming
-   **State Management**: React Context API
-   **Content**: Markdown, MDX
-   **UI Components**: Custom React components with accessibility support
-   **Routing**: Docusaurus Router (React Router under the hood)
-   **Build Tool**: Webpack (via Docusaurus)
-   **Deployment**: GitHub Pages

### Backend
-   **Framework**: FastAPI (Python 3.11+)
-   **ASGI Server**: Uvicorn
-   **Database**: PostgreSQL with SQLAlchemy ORM
-   **Migrations**: Alembic
-   **Vector Database**: Qdrant (semantic search)
-   **Authentication**: JWT tokens with bcrypt password hashing
-   **AI Integration**:
    - OpenAI Agents SDK with Gemini 2.5 Flash
    - Custom RAG pipeline with retrieval guardrails
    - Semantic embeddings for search
-   **API Documentation**: Auto-generated OpenAPI/Swagger
-   **Logging**: Structured JSON logging
-   **Deployment**: Railway

### AI Development
-   **Workflow**: Spec-Kit Plus (SDD-RI methodology)
-   **Agents**: Custom Claude Code agents for orchestration, content writing, architecture, etc.
-   **Skills**: Reusable domain-specific skills (25+ custom skills)
-   **Prompt Engineering**: Systematic prompt history records (PHRs)
-   **Version Control**: Git with feature branches

### DevOps & Tools
-   **Version Control**: Git, GitHub
-   **CI/CD**: GitHub Actions (frontend), Railway auto-deploy (backend)
-   **Package Management**:
    - Frontend: npm/yarn
    - Backend: uv (Python package manager)
-   **Code Quality**: TypeScript strict mode, Python type hints
-   **Testing**: Manual verification, integration testing

## AI-Native Workflow Explanation

Our development process is fundamentally AI-native, relying on a sophisticated orchestration of Spec-Kit Plus and custom Claude Code agents and skills.

-   **Spec-Kit Plus Commands**: We utilized commands like `/sp.specify` for detailed feature specifications, `/sp.plan` for architectural decisions, `/sp.tasks` for breaking down implementation into testable units, and `/sp.implement` for executing development tasks.
-   **Claude Code Agents & Skills**: Dedicated agents (e.g., `chief-orchestrator`, `curriculum-architect`, `book-content-writer`, `backend-rag`, `frontend-platform`, `localization`) were developed to handle specific aspects of the project. These agents leveraged specialized skills (e.g., `chapter-structuring`, `technical-writing`, `pedagogy-mapping`, `content-chunking`, `retrieval-guardrails`, `deployment-validation`) to automate complex tasks, enforce project conventions, and ensure high-quality outputs throughout the development lifecycle. This approach enabled rapid iteration and consistent adherence to design principles.

## API Endpoints Reference

### Authentication
-   `POST /api/auth/signup` - Create new user account with profile
-   `POST /api/auth/signin` - Authenticate and receive JWT token
-   `POST /api/auth/signout` - Sign out (invalidate session)
-   `GET /api/auth/me` - Get current user profile
-   `PUT /api/auth/profile` - Update user profile settings

### Chat & RAG
-   `POST /api/chat` - Send question to RAG tutor
    - Supports greeting detection
    - Returns grounded answer or refusal
    - Includes source citations
    - Session continuity tracking

### Search
-   `POST /api/search` - Semantic search across textbook
    - Returns ranked results with scores
    - No answer generation (search only)
    - Requires authentication

### Session Management
-   `GET /api/sessions` - List user's chat sessions
-   `GET /api/sessions/last` - Get most recent session
-   `GET /api/sessions/{id}` - Get specific session with messages
-   `POST /api/sessions` - Create new session explicitly
-   `DELETE /api/sessions/{id}` - Delete session and messages

### Content & Translation
-   `POST /api/translate` - Translate content to Urdu
-   `GET /api/content` - Get personalized content based on profile

### Health & Monitoring
-   `GET /api/health` - Health check endpoint
-   `GET /docs` - OpenAPI/Swagger documentation

## Phase Breakdown (Phases 1–6)

The project was developed iteratively across six distinct phases:

1.  **Phase 1: Curriculum Intelligence**: Established the pedagogical framework and detailed curriculum structure for the textbook, defining modules, topics, and learning objectives.

2.  **Phase 2: Textbook Content**: Generated the core textual content for the Physical AI and Humanoid Robotics textbook, covering fundamental concepts, advanced topics, and practical applications.

3.  **Phase 3: RAG Chatbot Backend**: Implemented a Retrieval Augmented Generation (RAG) system with:
    - FastAPI backend with authentication
    - Qdrant vector database integration
    - Gemini 2.5 Flash via OpenAI Agents SDK
    - Strict grounding guardrails and refusal logic
    - PostgreSQL for user and session persistence

4.  **Phase 4: Embedded Chat UI**: Developed and integrated a user-friendly chat interface with:
    - Floating chat panel accessible from any page
    - Chat history sidebar with session management
    - Selected text mode for contextual questions
    - Multi-mode support (general, explain, translate)
    - Professional UX with loading states and error handling

5.  **Phase 5: Authentication & Personalization**: Added comprehensive auth system with:
    - Full-page login/signup routes (not modals)
    - User profiles with experience levels
    - Protected content guards
    - Session management and continuity
    - Sign-out confirmation dialogs
    - Toast notifications for feedback
    - Redirect handling with security validation

6.  **Phase 6: Advanced Features**:
    - **Global Spotlight Search**: Cmd/Ctrl+K semantic search with quick actions
    - **Urdu Translation**: AI-powered translation with RTL support
    - **Greeting Detection**: Smart routing for conversational vs technical questions
    - **Professional UX**: Glass-morphism, animations, dark mode, responsive design

## Reusable Intelligence Section

### Agents

-   **chief-orchestrator**: Manages overall project workflow, task delegation, and ensures phase completion.
-   **curriculum-architect**: Designs and structures the textbook curriculum, defining learning objectives and content flow.
-   **book-content-writer**: Generates and refines textbook chapters and sections based on the curriculum.
-   **backend-rag**: Implements the Retrieval Augmented Generation (RAG) system and its API.
-   **frontend-platform**: Develops and integrates the Docusaurus frontend and chat UI.
-   **identity-personalization**: Handles user authentication and personalization features.
-   **devops-github**: Manages deployment workflows and CI/CD pipelines.

### Skills

#### Content & Pedagogy Skills
-   **chapter-structuring**: Organizes textbook content into logical chapters, sections, and sub-sections
-   **technical-writing**: Ensures clarity, accuracy, and adherence to technical writing standards
-   **pedagogy-mapping**: Aligns content with learning objectives and effective teaching methodologies
-   **content-chunking**: Prepares content for optimal RAG retrieval by breaking it into manageable chunks

#### RAG & AI Skills
-   **retrieval-guardrails**: Implements rules to ensure RAG responses are grounded and safe
-   **content-personalization**: Adapts content depth and examples based on user profile
-   **diagram-explainer**: Explains system architectures and workflows using diagrams

#### Frontend & UX Skills
-   **ui-architecture**: Structural rules for composing frontend layouts (pages, modals, overlays)
-   **runtime-safety**: Prevents frontend runtime crashes and undefined behavior
-   **auth-ui-patterns**: Standardizes authentication flows and UI behavior
-   **state-management**: Manages shared frontend state predictably
-   **platform-integration**: Safely connects frontend to backend APIs

#### Development & Operations Skills
-   **deployment-validation**: Verifies successful deployment of frontend and backend services
-   **git-automation**: Automates Git workflows and repository hygiene
-   **project-scaffolding**: Initializes AI-native monorepo with standard structure
-   **spec-driven-design**: Designs systems using explicit specifications
-   **hardware-requirements-validation**: Validates and explains hardware requirements
-   **translation-urdu**: Translates technical educational content into Urdu
-   **user-profiling**: Collects and normalizes user background information

### Why this matters

This modular approach to AI agents and skills creates a highly reusable and adaptable development framework. It significantly reduces manual effort, improves consistency, and enables rapid development of complex, AI-driven applications. This "intelligence as code" paradigm is a blueprint for future AI-native software engineering.

## How to Run Locally

### Prerequisites
-   **Node.js** 18+ and npm/yarn
-   **Python** 3.11+
-   **PostgreSQL** 14+ (or Docker)
-   **Qdrant** vector database (or Docker)
-   **Google API Key** (for Gemini)

### Backend Setup

```bash
# 1. Navigate to backend directory
cd backend

# 2. Install Python dependencies using uv
uv pip install -r pyproject.toml

# 3. Create .env file with required variables
cat > .env << EOF
DATABASE_URL=postgresql+asyncpg://user:password@localhost:5432/robotics_book
QDRANT_URL=http://localhost:6333
GOOGLE_API_KEY=your_gemini_api_key_here
JWT_SECRET_KEY=your_secret_key_here
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=30
RETRIEVAL_THRESHOLD=0.7
RETRIEVAL_LIMIT=5
EOF

# 4. Run database migrations
alembic upgrade head

# 5. (Optional) Seed vector database with textbook content
python scripts/seed_qdrant.py

# 6. Start the FastAPI server
uvicorn app.main:app --reload --host 0.0.0.0 --port 8001

# Backend will be available at: http://localhost:8001
# API docs at: http://localhost:8001/docs
```

### Frontend Setup

```bash
# 1. Navigate to docs directory
cd docs

# 2. Install Node.js dependencies
npm install

# 3. Create .env file (if needed for custom API URL)
cat > .env << EOF
API_BASE_URL=http://localhost:8001
EOF

# 4. Start Docusaurus development server
npm start

# Frontend will be available at: http://localhost:3000
```

### Docker Setup (Alternative)

```bash
# Start PostgreSQL and Qdrant using Docker Compose
docker-compose up -d postgres qdrant

# Then follow backend and frontend setup steps above
```

### Access the Application

1. **Homepage**: `http://localhost:3000/`
2. **Textbook**: `http://localhost:3000/textbook/intro`
3. **Login**: `http://localhost:3000/login`
4. **API Docs**: `http://localhost:8001/docs`

### Test Accounts

Create a test account via signup page or use:
```bash
# Create test user via API
curl -X POST http://localhost:8001/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test1234",
    "profile": {
      "software_level": "intermediate",
      "robotics_level": "beginner",
      "hardware_access": "simulation_only"
    }
  }'
```

*(Detailed instructions can be found in `backend/README.md` and `docs/README.md`)*

## Deployment Notes

The project leverages continuous deployment:
-   **Frontend**: Automatically deployed to GitHub Pages upon pushes to the `main` branch from the `docs/` directory.
-   **Backend**: Automatically deployed to Railway upon pushes to the `main` branch from the `backend/` directory, configured via `railway.toml`.

## Project Statistics

### Codebase Metrics
-   **Total Files**: 150+ files across frontend, backend, and specs
-   **Lines of Code**:
    - Frontend: ~8,000 lines (TypeScript, TSX, CSS)
    - Backend: ~4,000 lines (Python)
    - Specs & Docs: ~3,000 lines (Markdown)
-   **Components**: 40+ React components
-   **API Endpoints**: 15+ REST endpoints
-   **Skills**: 20+ custom Claude Code skills
-   **Agents**: 8+ specialized AI agents
-   **Prompt History Records**: 50+ documented interactions

### Features Delivered
-   ✅ Interactive textbook with 5 modules
-   ✅ RAG-powered AI tutor with grounding guardrails
-   ✅ Global spotlight search (Ctrl/Cmd+K)
-   ✅ Full authentication system with JWT
-   ✅ User profiles and personalization
-   ✅ Chat session management and history
-   ✅ Urdu translation support
-   ✅ Professional UX with dark mode
-   ✅ Full-page auth routes (not modals)
-   ✅ Sign-out confirmation dialogs
-   ✅ Toast notifications
-   ✅ Protected content guards
-   ✅ Semantic search with Qdrant
-   ✅ OpenAPI documentation

### Development Timeline
-   **Phase 1-2**: Curriculum design & content generation
-   **Phase 3**: RAG backend implementation
-   **Phase 4**: Chat UI and embedding
-   **Phase 5**: Authentication & personalization
-   **Phase 6**: Advanced features & polish
-   **Total**: Comprehensive platform built with AI-native workflow

## Hackathon Alignment Section

This project directly aligns with the hackathon's core themes by demonstrating:

### AI-Native Development
-   **25+ Custom Skills**: Reusable domain-specific AI capabilities
-   **8+ Specialized Agents**: Orchestrated workflow automation
-   **Spec-Driven Development**: Systematic feature implementation with `/sp.*` commands
-   **Prompt History Records**: Every interaction documented for reproducibility
-   **Architecture Decision Records**: Design rationale captured systematically

### Technical Excellence
-   **Full-Stack Implementation**: Production-ready frontend and backend
-   **Modern Tech Stack**: Docusaurus v4, FastAPI, PostgreSQL, Qdrant, Gemini
-   **Type Safety**: TypeScript strict mode + Python type hints
-   **Security**: JWT auth, password hashing, CORS, input validation, XSS prevention
-   **Scalability**: Async architecture, connection pooling, efficient queries

### Innovation & Complexity
-   **RAG Guardrails**: Novel approach to preventing hallucinations in educational content
-   **Greeting Detection**: Smart routing for conversational vs technical queries
-   **Session Continuity**: Stateful chat across page navigation
-   **Multi-Modal Search**: Semantic + keyword hybrid ranking
-   **Professional UX**: Full-page auth routes, confirmation dialogs, toast notifications

### Completeness
-   **Deployed**: Frontend on GitHub Pages, Backend on Railway
-   **Documented**: Comprehensive specs, ADRs, PHRs, API docs
-   **Accessible**: ARIA attributes, keyboard navigation, screen reader support
-   **Localized**: Urdu translation with RTL support
-   **Tested**: Manual verification across all features
-   **Polished**: Professional design, animations, error handling

## Contributing

We welcome contributions! Please see our contributing guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Follow Spec-Kit Plus workflow (`/sp.specify`, `/sp.plan`, `/sp.tasks`)
4. Commit changes with descriptive messages
5. Push to your branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

### Development Standards
-   Follow existing code style and conventions
-   Add type annotations (TypeScript/Python)
-   Write clear commit messages
-   Document architectural decisions (ADRs)
-   Test features manually before submitting

## License

This project is open-source under the **MIT License**.

```
MIT License

Copyright (c) 2025 Physical AI & Humanoid Robotics Textbook Project

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Acknowledgments

-   **Spec-Kit Plus**: For the AI-native development workflow and methodology
-   **Claude Code**: For agents and skills framework
-   **Anthropic**: For Claude API and Sonnet/Opus models
-   **Google**: For Gemini 2.5 Flash API
-   **OpenAI Agents SDK**: For agent orchestration framework
-   **Docusaurus Team**: For the excellent documentation framework
-   **FastAPI Community**: For the modern Python web framework
-   **Qdrant**: For vector database and semantic search capabilities

## Contact & Support

-   **Issues**: [GitHub Issues](https://github.com/physical-ai-robotics/robotics-book/issues)
-   **Discussions**: [GitHub Discussions](https://github.com/physical-ai-robotics/robotics-book/discussions)
-   **Documentation**: See `/docs` directory for detailed guides

---

**Built with AI-Native Development** | **Powered by Spec-Kit Plus & Claude Code** | **2025**
# Quickstart: Physical AI & Humanoid Robotics - AI-Native Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2024-12-24

This guide helps you get the development environment running locally.

## Prerequisites

- **Node.js**: 20.x LTS
- **Python**: 3.11+
- **Git**: Latest
- **pnpm** (recommended) or npm
- **Accounts**:
  - Qdrant Cloud (free tier): https://cloud.qdrant.io
  - Neon Postgres (free tier): https://neon.tech
  - OpenAI API key: https://platform.openai.com

## Quick Start (5 minutes)

### 1. Clone and Setup

```bash
# Clone repository
git clone <repo-url>
cd robotics-book

# Switch to feature branch
git checkout 001-physical-ai-textbook
```

### 2. Frontend (Docusaurus)

```bash
# Navigate to book directory
cd book

# Install dependencies
pnpm install  # or npm install

# Start development server
pnpm start  # or npm start

# Opens http://localhost:3000
```

### 3. Backend (FastAPI)

```bash
# Open new terminal, navigate to backend
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env
# Edit .env with your credentials (see below)

# Start development server
uvicorn app.main:app --reload --port 8001

# API available at http://localhost:8001
# Docs at http://localhost:8001/docs
```

## Environment Variables

Create `.env` in `/backend/`:

```env
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...

# Neon Postgres
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# Better Auth (Bonus)
AUTH_SECRET=generate-random-32-char-string

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://your-github-pages-url

# Environment
ENV=development
```

## Project Structure

```
robotics-book/
├── book/                    # Docusaurus frontend
│   ├── docs/               # Textbook content (MDX)
│   ├── src/components/     # React components (Chatbot, etc.)
│   └── docusaurus.config.ts
├── backend/                 # FastAPI backend
│   ├── app/
│   │   ├── api/routes/     # API endpoints
│   │   ├── services/       # Business logic (RAG, etc.)
│   │   └── db/             # Database connections
│   └── tests/
├── specs/                   # Specifications
│   └── 001-physical-ai-textbook/
│       ├── spec.md
│       ├── plan.md
│       ├── data-model.md
│       └── contracts/
└── .github/workflows/       # CI/CD
```

## Development Workflow

### Writing Content

1. Create/edit MDX files in `book/docs/module-X/`
2. Follow chapter template:
   ```mdx
   ---
   sidebar_position: 1
   ---

   # Chapter Title

   ## Learning Objectives
   - Objective 1
   - Objective 2

   ## Section 1
   Content with examples...

   ## Key Takeaways
   - Takeaway 1
   ```
3. Preview at http://localhost:3000

### Testing RAG

1. Ensure backend is running
2. Chunk content: `python scripts/chunk_content.py`
3. Upload to Qdrant: `python scripts/upload_embeddings.py`
4. Test chatbot in frontend or via API:
   ```bash
   curl -X POST http://localhost:8001/api/chat \
     -H "Content-Type: application/json" \
     -d '{"question": "What is a ROS 2 topic?"}'
   ```

### Running Tests

```bash
# Frontend tests
cd book && pnpm test

# Backend tests
cd backend && pytest

# E2E tests
cd book && pnpm test:e2e
```

## Common Issues

### CORS Errors
- Ensure `ALLOWED_ORIGINS` in `.env` includes your frontend URL
- Check that backend CORS middleware is configured

### Qdrant Connection Failed
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check Qdrant Cloud dashboard for cluster status

### OpenAI Rate Limits
- Use `text-embedding-3-small` for embeddings (cheaper, faster)
- Implement caching for repeated queries

### Database Connection Issues
- Neon requires `?sslmode=require` in connection string
- Check Neon dashboard for connection limits

## Deployment

### GitHub Pages (Frontend)

```bash
# Build static site
cd book && pnpm build

# Deploy via GitHub Actions (automatic on push to main)
# Or manually:
pnpm deploy
```

### Railway (Backend)

1. Connect Railway to GitHub repo
2. Set environment variables in Railway dashboard
3. Deploy automatically on push

## Useful Commands

```bash
# Format code
pnpm format        # Frontend
black .            # Backend

# Lint
pnpm lint          # Frontend
ruff .             # Backend

# Type check
pnpm typecheck     # Frontend
mypy app           # Backend

# Generate OpenAPI client
openapi-generator generate -i specs/contracts/rag-api.yaml -g typescript-fetch -o book/src/api
```

## Next Steps

1. Read `specs/001-physical-ai-textbook/spec.md` for full requirements
2. Review `plan.md` for implementation phases
3. Check `data-model.md` for database schema
4. Explore `contracts/*.yaml` for API specifications
5. Run `/sp.tasks` to see task breakdown

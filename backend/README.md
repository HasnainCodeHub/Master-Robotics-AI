# Physical AI Textbook RAG Backend

FastAPI backend providing Retrieval-Augmented Generation (RAG) for the Physical AI & Humanoid Robotics textbook.

## RAG Safety Rules (NON-NEGOTIABLE)

- All answers MUST be grounded in retrieved textbook content
- No external knowledge is permitted
- Refusal is ALWAYS preferred over speculation

## Quick Start

### Local Development

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Copy environment variables
cp .env.example .env
# Edit .env with your credentials

# Run the server
uvicorn app.main:app --reload
```

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/chat` | POST | Send question to RAG chatbot |
| `/api/content/chunks` | GET | Debug endpoint for content chunks |
| `/health` | GET | Health check for deployments |

### Content Ingestion

Before the chatbot can answer questions, you need to ingest the textbook content:

```bash
# Dry run (see what will be processed)
python scripts/ingest_content.py --dry-run

# Full ingestion
python scripts/ingest_content.py

# Ingest specific module
python scripts/ingest_content.py --module module-2
```

## Environment Variables

### Required

| Variable | Description |
|----------|-------------|
| `QDRANT_URL` | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Qdrant Cloud API key |
| `DATABASE_URL` | Neon Postgres connection string |
| `GOOGLE_API_KEY` | Google Gemini API key (FREE tier) for embeddings, generation, and translation |

### Optional

| Variable | Default | Description |
|----------|---------|-------------|
| `ENV` | `development` | Environment (development/production) |
| `LOG_LEVEL` | `INFO` | Logging level |
| `CORS_ORIGINS` | `http://localhost:3000` | Allowed CORS origins (comma-separated) |
| `EMBEDDING_MODEL` | `text-embedding-004` | Gemini embedding model (768 dimensions) |
| `EMBEDDING_DIMENSION` | `768` | Embedding vector dimension |
| `RETRIEVAL_THRESHOLD` | `0.7` | Minimum similarity score for retrieval |
| `RETRIEVAL_LIMIT` | `5` | Maximum chunks to retrieve |

## Railway Deployment

### 1. Create Railway Project

```bash
# Install Railway CLI
npm i -g @railway/cli

# Login
railway login

# Create new project
railway init
```

### 2. Set Environment Variables

In the Railway dashboard, add all required environment variables from `.env.example`.

For `CORS_ORIGINS`, include your GitHub Pages URL:
```
https://yourusername.github.io,http://localhost:3000
```

### 3. Deploy

```bash
# Link to project
railway link

# Deploy
railway up
```

Or connect your GitHub repository for automatic deployments.

### 4. Verify Deployment

```bash
# Check health
curl https://your-app.railway.app/health
```

## Project Structure

```
backend/
├── app/
│   ├── api/
│   │   ├── routes/
│   │   │   ├── chat.py      # POST /api/chat
│   │   │   ├── content.py   # GET /api/content/chunks
│   │   │   └── health.py    # GET /health
│   │   └── deps.py          # Dependencies, error handlers
│   ├── core/
│   │   ├── config.py        # Settings from environment
│   │   └── logging.py       # Structured logging
│   ├── db/
│   │   ├── qdrant.py        # Vector database operations
│   │   └── postgres.py      # Relational database operations
│   ├── models/
│   │   └── chat.py          # SQLAlchemy models
│   ├── schemas/
│   │   ├── chat.py          # Pydantic schemas for chat
│   │   ├── content.py       # Pydantic schemas for content
│   │   └── health.py        # Health check schema
│   ├── services/
│   │   ├── chunking.py      # Content chunking logic
│   │   ├── embedding.py     # Gemini embeddings (text-embedding-004)
│   │   ├── generation.py    # Gemini generation via OpenAI Agents SDK
│   │   ├── translation.py   # Gemini translation (English → Urdu)
│   │   ├── rag.py           # RAG pipeline with refusal
│   │   └── session.py       # Chat session management
│   └── main.py              # FastAPI application
├── scripts/
│   └── ingest_content.py    # Content ingestion CLI
├── alembic/                 # Database migrations
├── Dockerfile
├── railway.toml
├── Procfile
├── requirements.txt
└── .env.example
```

## Testing

```bash
# Test chat endpoint
curl -X POST http://localhost:8001/api/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is a ROS 2 topic?"}'

# Test with selected text
curl -X POST http://localhost:8001/api/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "Explain this", "selected_text": "A topic is a named bus..."}'

# Test health
curl http://localhost:8001/health
```

## Refusal Behavior

The chatbot will refuse to answer in these cases:

| Scenario | Refusal Reason |
|----------|----------------|
| No matching content found | `empty_retrieval` |
| All matches below 0.7 threshold | `insufficient_context` |
| Question about PID, PWM, PCB, etc. | `out_of_scope` |
| Selected text too short | `selected_text_insufficient` |

## License

Part of the Physical AI & Humanoid Robotics textbook project.

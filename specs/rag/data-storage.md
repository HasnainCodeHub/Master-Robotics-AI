# Data Storage Responsibilities

**Phase**: 3 — Backend & RAG
**Date**: 2024-12-24
**Status**: LOCKED
**Reference**: specs/001-physical-ai-textbook/data-model.md

---

## Overview

This document defines the clear separation of data storage responsibilities between Qdrant Cloud (vector storage) and Neon Postgres (relational storage).

---

## Storage Allocation Summary

| Data Type | Storage | Reason |
|-----------|---------|--------|
| Content chunks + embeddings | Qdrant | Vector similarity search |
| Chunk metadata | Qdrant payload | Co-located with vectors |
| Chat sessions | Postgres | Relational queries, user linking |
| Chat messages | Postgres | Audit trail, history |
| User accounts | Postgres | Phase 5 (not in scope) |
| User profiles | Postgres | Phase 5 (not in scope) |

---

## Qdrant Cloud

### Collection: `textbook_chunks`

**Purpose**: Store content chunks with embeddings for semantic search.

**Schema**:
```python
from qdrant_client.models import VectorParams, Distance

collection_config = {
    "collection_name": "textbook_chunks",
    "vectors_config": VectorParams(
        size=1536,  # text-embedding-3-small dimension
        distance=Distance.COSINE
    )
}
```

**Point Structure**:
```python
{
    "id": "uuid",  # Chunk unique ID
    "vector": [float] * 1536,  # Embedding
    "payload": {
        "text": "Full chunk text with heading prefix",
        "module_id": "module-2-ros2",
        "chapter_id": "chapter-2-topics-pubsub",
        "section_heading": "## What is a ROS 2 Topic?",
        "chunk_index": 0
    }
}
```

**Indexes**:
- Primary: HNSW on vector field (automatic)
- Payload indexes: `module_id`, `chapter_id` (for filtering)

**Connection**:
```python
from qdrant_client import QdrantClient

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
```

---

## Neon Postgres

### Database: `textbook_rag`

**Purpose**: Store chat sessions and messages for history and audit.

### Table: `chat_sessions`

```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,  -- Phase 5
    started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    ended_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_session_user ON chat_sessions(user_id);
CREATE INDEX idx_session_time ON chat_sessions(started_at DESC);
```

**Notes**:
- `user_id` is nullable for anonymous sessions (Phase 3)
- `ended_at` is null for active sessions

### Table: `chat_messages`

```sql
CREATE TYPE message_role AS ENUM ('user', 'assistant');

CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role message_role NOT NULL,
    content TEXT NOT NULL,
    sources JSONB,  -- Array of source references
    was_refusal BOOLEAN NOT NULL DEFAULT FALSE,
    refusal_reason VARCHAR(50),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_message_session ON chat_messages(session_id);
CREATE INDEX idx_message_time ON chat_messages(created_at DESC);
```

**Sources JSONB Schema**:
```json
{
  "chunks": [
    {
      "chunk_id": "uuid",
      "module_id": "string",
      "chapter_id": "string",
      "section_heading": "string",
      "score": 0.89
    }
  ],
  "used_selected_text": false
}
```

**Connection**:
```python
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession

engine = create_async_engine(
    os.getenv("DATABASE_URL").replace("postgres://", "postgresql+asyncpg://"),
    echo=False
)
```

---

## Phase 3 Scope (No User Tables)

The following tables are **NOT** created in Phase 3:

| Table | Reason | Phase |
|-------|--------|-------|
| users | Authentication feature | Phase 5 |
| user_profiles | Personalization feature | Phase 5 |

For Phase 3, `chat_sessions.user_id` is always `NULL` (anonymous).

---

## Environment Variables

```bash
# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=your-api-key

# Neon Postgres
DATABASE_URL=postgres://user:password@host/database?sslmode=require

# OpenAI (for embeddings)
OPENAI_API_KEY=sk-...
```

---

## Data Flow

### Content Ingestion (One-time)

```
Markdown files (docs/docs/)
        │
        ▼
    Chunking Service
        │
        ▼
    Embedding Service (OpenAI)
        │
        ▼
    Qdrant Cloud (textbook_chunks)
```

### Chat Query

```
User Question
        │
        ▼
    Embedding (OpenAI)
        │
        ▼
    Vector Search (Qdrant)
        │
        ▼
    Refusal Check
        │
        ├── REFUSE → Return refusal
        │
        └── PASS → LLM Generation (OpenAI)
                        │
                        ▼
                Save to Postgres (chat_messages)
                        │
                        ▼
                Return response
```

---

## Backup & Recovery

### Qdrant
- Qdrant Cloud handles backups automatically
- Point-in-time recovery available on paid tiers
- Free tier: Re-ingest from markdown if needed

### Postgres
- Neon provides automatic daily backups
- Point-in-time recovery available
- Manual backup: `pg_dump`

---

## Free Tier Limits

### Qdrant Cloud Free
| Resource | Limit | Our Usage |
|----------|-------|-----------|
| Vectors | 1M | ~300 chunks |
| Storage | 1 GB | < 50 MB |
| Collections | 5 | 1 |

### Neon Postgres Free
| Resource | Limit | Our Usage |
|----------|-------|-----------|
| Storage | 0.5 GB | < 100 MB |
| Compute | 191.9 hours/month | Light queries |
| Branches | 10 | 1 main |

Both well within limits for hackathon scope.

---

## Migration Commands

### Alembic Setup

```bash
cd backend
alembic init alembic
```

**alembic.ini**:
```ini
sqlalchemy.url = %(DATABASE_URL)s
```

### Initial Migration

```bash
alembic revision --autogenerate -m "Create chat tables"
alembic upgrade head
```

---

## Validation Checklist

Before deployment, verify:

- [ ] Qdrant collection created with correct dimensions (1536)
- [ ] Postgres tables created with correct schema
- [ ] Indexes created for query performance
- [ ] Environment variables set for both databases
- [ ] Connection health checks pass
- [ ] JSONB sources schema validated

---

## Sign-off

This data storage specification is **LOCKED** for Phase 3.

Qdrant = vectors, Postgres = relational. No overlap.

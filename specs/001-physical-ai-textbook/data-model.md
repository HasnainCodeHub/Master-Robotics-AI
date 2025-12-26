# Data Model: Physical AI & Humanoid Robotics - AI-Native Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2024-12-24
**Phase**: 1 - Design & Contracts

## Overview

This document defines the data entities, their relationships, and storage allocation across the two database systems:
- **Qdrant Cloud**: Vector embeddings and content chunks
- **Neon Postgres**: Relational data (users, sessions, chat history)

## Entity Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        QDRANT CLOUD                             │
├─────────────────────────────────────────────────────────────────┤
│  ContentChunk                                                   │
│  ├── id: uuid                                                   │
│  ├── vector: float[1536]                                        │
│  └── payload:                                                   │
│      ├── text: string                                           │
│      ├── module_id: string                                      │
│      ├── chapter_id: string                                     │
│      ├── section_heading: string                                │
│      └── chunk_index: int                                       │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                       NEON POSTGRES                             │
├─────────────────────────────────────────────────────────────────┤
│  User                          UserProfile                      │
│  ├── id: uuid (PK)             ├── user_id: uuid (PK, FK)       │
│  ├── email: string (unique)    ├── software_level: enum         │
│  ├── password_hash: string     ├── robotics_level: enum         │
│  ├── created_at: timestamp     └── updated_at: timestamp        │
│  └── updated_at: timestamp                                      │
│         │                                                       │
│         │ 1:N                                                   │
│         ▼                                                       │
│  ChatSession                   ChatMessage                      │
│  ├── id: uuid (PK)             ├── id: uuid (PK)                │
│  ├── user_id: uuid (FK, null)  ├── session_id: uuid (FK)        │
│  ├── started_at: timestamp     ├── role: enum (user/assistant)  │
│  └── ended_at: timestamp       ├── content: text                │
│         │                      ├── sources: jsonb               │
│         │ 1:N                  ├── was_refusal: boolean         │
│         ▼                      └── created_at: timestamp        │
│  ChatMessage ◄─────────────────┘                                │
└─────────────────────────────────────────────────────────────────┘
```

## Entities

### ContentChunk (Qdrant)

Semantically complete sections of textbook content for RAG retrieval.

| Field | Type | Description |
|-------|------|-------------|
| id | UUID | Unique identifier |
| vector | float[1536] | OpenAI text-embedding-3-small embedding |
| text | string | Raw text content of the chunk |
| module_id | string | e.g., "module-1-physical-ai" |
| chapter_id | string | e.g., "01-embodied-intelligence" |
| section_heading | string | e.g., "## What is Embodied Intelligence?" |
| chunk_index | int | Order within chapter (0, 1, 2...) |

**Validation Rules**:
- text must be non-empty
- text should be 100-1000 tokens
- module_id must match existing module
- vector dimension must be exactly 1536

---

### User (Postgres)

Registered learner account (Bonus feature).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL | Login email |
| password_hash | VARCHAR(255) | NOT NULL | bcrypt hash |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Registration time |
| updated_at | TIMESTAMP | NOT NULL | Last modification |

**Validation Rules**:
- email must be valid email format
- password_hash must be bcrypt (60 chars)

---

### UserProfile (Postgres)

User background information for personalization (Bonus feature).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | UUID | PK, FK(User) | Links to User |
| software_level | ENUM | NOT NULL | beginner/intermediate/advanced |
| robotics_level | ENUM | NOT NULL | beginner/intermediate/advanced |
| updated_at | TIMESTAMP | NOT NULL | Last profile update |

**Enum Values**:
```sql
CREATE TYPE experience_level AS ENUM ('beginner', 'intermediate', 'advanced');
```

**Validation Rules**:
- Both levels must be set during signup
- Personalization MUST NOT change if levels are null (show default content)

---

### ChatSession (Postgres)

Groups related chat messages for a user session.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| user_id | UUID | FK(User), NULLABLE | Null for anonymous users |
| started_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Session start |
| ended_at | TIMESTAMP | NULLABLE | Session end (null if active) |

**Validation Rules**:
- Anonymous sessions allowed (user_id = null)
- ended_at >= started_at when set

---

### ChatMessage (Postgres)

Individual message in a chat exchange.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| session_id | UUID | FK(ChatSession), NOT NULL | Parent session |
| role | ENUM | NOT NULL | user/assistant |
| content | TEXT | NOT NULL | Message text |
| sources | JSONB | NULLABLE | Retrieved chunk references |
| was_refusal | BOOLEAN | NOT NULL, DEFAULT FALSE | True if assistant refused |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Message time |

**Enum Values**:
```sql
CREATE TYPE message_role AS ENUM ('user', 'assistant');
```

**Sources Schema** (JSONB):
```json
{
  "chunks": [
    {
      "chunk_id": "uuid",
      "module_id": "string",
      "chapter_id": "string",
      "section_heading": "string",
      "score": 0.85
    }
  ],
  "used_selected_text": true
}
```

**Validation Rules**:
- If role = 'assistant' and was_refusal = false, sources must be non-empty
- If role = 'user', sources must be null

---

## Relationships

| Relationship | Type | Description |
|--------------|------|-------------|
| User → UserProfile | 1:1 | Each user has exactly one profile |
| User → ChatSession | 1:N | User can have many sessions |
| ChatSession → ChatMessage | 1:N | Session contains many messages |

## Indexes

### Postgres
```sql
-- User lookups
CREATE UNIQUE INDEX idx_user_email ON users(email);

-- Session queries
CREATE INDEX idx_session_user ON chat_sessions(user_id);
CREATE INDEX idx_session_time ON chat_sessions(started_at DESC);

-- Message queries
CREATE INDEX idx_message_session ON chat_messages(session_id);
CREATE INDEX idx_message_time ON chat_messages(created_at DESC);
```

### Qdrant
- Primary index: HNSW on vector field (automatic)
- Payload indexes on: module_id, chapter_id (for filtering)

## State Transitions

### ChatMessage States
```
User sends question
    │
    ▼
[Create user message]
    │
    ▼
Retrieve chunks from Qdrant
    │
    ├── Sufficient chunks (score > 0.7)
    │       │
    │       ▼
    │   Generate grounded answer
    │       │
    │       ▼
    │   [Create assistant message]
    │   was_refusal = false
    │   sources = retrieved chunks
    │
    └── Insufficient chunks
            │
            ▼
        Generate refusal
            │
            ▼
        [Create assistant message]
        was_refusal = true
        sources = null
```

## Migration Strategy

### Initial Setup
1. Create Qdrant collection with vector dimension 1536
2. Run Postgres migrations for all tables
3. Seed ContentChunks by processing textbook content

### Content Updates
1. Re-chunk updated chapters
2. Delete old chunks for chapter
3. Insert new chunks
4. No Postgres migrations needed

## Privacy Considerations

- Passwords stored as bcrypt hashes only
- Chat history linked to user_id (can be deleted on request)
- Anonymous sessions have null user_id (no tracking)
- No PII stored in Qdrant (content only)

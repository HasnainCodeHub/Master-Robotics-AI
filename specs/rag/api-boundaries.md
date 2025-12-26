# FastAPI API Boundaries Specification

**Phase**: 3 — Backend & RAG
**Date**: 2024-12-24
**Status**: LOCKED
**Contract Reference**: specs/001-physical-ai-textbook/contracts/rag-api.yaml

---

## Overview

This document defines the API endpoints for the RAG backend. These boundaries are strict and must align with the OpenAPI contract.

---

## Base Configuration

| Setting | Value |
|---------|-------|
| Base URL | `https://{railway-app}.railway.app` |
| Local Dev | `http://localhost:8001` |
| API Prefix | `/api` |
| Content-Type | `application/json` |
| CORS Origins | GitHub Pages, localhost:3000 |

---

## Endpoints

### POST /api/chat

**Purpose**: Send a question to the RAG chatbot and receive a grounded answer or refusal.

**Request**:
```json
{
  "question": "string (required, 1-1000 chars)",
  "selected_text": "string (optional, max 5000 chars)",
  "session_id": "uuid (optional)"
}
```

**Response (200 - Success)**:
```json
{
  "answer": "string",
  "sources": [
    {
      "chunk_id": "uuid",
      "module_id": "string",
      "chapter_id": "string",
      "section_heading": "string",
      "score": 0.89
    }
  ],
  "was_refusal": false,
  "refusal_reason": null,
  "session_id": "uuid"
}
```

**Response (200 - Refusal)**:
```json
{
  "answer": "I can only answer questions from the textbook content...",
  "sources": [],
  "was_refusal": true,
  "refusal_reason": "insufficient_retrieval",
  "session_id": "uuid"
}
```

**Error Responses**:
| Status | Reason |
|--------|--------|
| 400 | Invalid request (missing question, too long) |
| 500 | Server error (database, OpenAI API failure) |

---

### GET /api/content/chunks

**Purpose**: Debug/admin endpoint to inspect content chunks for a module/chapter.

**Query Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| module_id | string | Yes | Module identifier |
| chapter_id | string | No | Chapter identifier (optional filter) |

**Response (200)**:
```json
{
  "chunks": [
    {
      "id": "uuid",
      "text": "string (first 200 chars preview)",
      "module_id": "string",
      "chapter_id": "string",
      "section_heading": "string",
      "chunk_index": 0
    }
  ],
  "total": 15
}
```

**Note**: This endpoint is for debugging. Consider auth protection in production.

---

### GET /health

**Purpose**: Health check endpoint for Railway deployment and monitoring.

**Response (200)**:
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "postgres_connected": true,
  "version": "1.0.0",
  "timestamp": "2024-12-24T10:30:00Z"
}
```

**Response (503 - Unhealthy)**:
```json
{
  "status": "unhealthy",
  "qdrant_connected": false,
  "postgres_connected": true,
  "error": "Qdrant connection timeout"
}
```

---

## Error Response Schema

All error responses follow this format:

```json
{
  "error": "error_code",
  "message": "Human-readable description",
  "details": {
    "field": "additional context"
  }
}
```

**Error Codes**:
| Code | HTTP Status | Description |
|------|-------------|-------------|
| `validation_error` | 400 | Invalid request body |
| `question_too_long` | 400 | Question exceeds 1000 chars |
| `selected_text_too_long` | 400 | Selected text exceeds 5000 chars |
| `internal_error` | 500 | Unexpected server error |
| `qdrant_error` | 500 | Qdrant connection/query failed |
| `openai_error` | 500 | OpenAI API failed |

---

## CORS Configuration

```python
from fastapi.middleware.cors import CORSMiddleware

ALLOWED_ORIGINS = [
    "https://<username>.github.io",  # GitHub Pages
    "http://localhost:3000",          # Local Docusaurus dev
    "http://localhost:8001",          # Local backend dev
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["*"],
)
```

---

## Rate Limiting (Future)

Not implemented in Phase 3, but planned structure:

| Endpoint | Limit |
|----------|-------|
| POST /api/chat | 30 requests/minute per IP |
| GET /api/content/chunks | 60 requests/minute per IP |
| GET /health | Unlimited |

---

## Request Validation

### Question Field
- Required
- Min length: 1 character
- Max length: 1000 characters
- Trimmed of leading/trailing whitespace

### Selected Text Field
- Optional
- Max length: 5000 characters
- If empty string, treated as `None`

### Session ID Field
- Optional
- Must be valid UUID v4 format
- If not provided, new session created

---

## FastAPI Route Structure

```
backend/app/api/
├── __init__.py
├── deps.py              # Dependencies (DB connections, auth)
└── routes/
    ├── __init__.py
    ├── chat.py          # POST /api/chat
    ├── content.py       # GET /api/content/chunks
    └── health.py        # GET /health
```

---

## Response Headers

All responses include:

| Header | Value |
|--------|-------|
| Content-Type | application/json |
| X-Request-ID | UUID for tracing |
| X-Response-Time | Request duration in ms |

---

## Excluded Endpoints (Phase 3)

The following endpoints from rag-api.yaml are **NOT** implemented in Phase 3:

| Endpoint | Reason | Phase |
|----------|--------|-------|
| GET /api/chat/sessions | Requires auth | Phase 5 |
| GET /api/chat/sessions/{id}/messages | Requires auth | Phase 5 |
| POST /api/translate | Translation feature | Phase 6 |
| POST /api/auth/* | Authentication | Phase 5 |

---

## Validation Checklist

Before deployment, verify:

- [ ] POST /api/chat accepts valid requests
- [ ] POST /api/chat returns refusal when appropriate
- [ ] GET /api/content/chunks returns chunk previews
- [ ] GET /health returns connection status
- [ ] CORS allows GitHub Pages origin
- [ ] Error responses follow schema
- [ ] Request validation rejects invalid input

---

## Sign-off

These API boundaries are **LOCKED** for Phase 3.

Only /api/chat, /api/content/chunks, and /health are in scope.

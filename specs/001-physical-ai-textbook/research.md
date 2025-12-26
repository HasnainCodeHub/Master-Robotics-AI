# Research: Physical AI & Humanoid Robotics - AI-Native Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2024-12-24
**Phase**: 0 - Outline & Research

## Research Questions

### 1. Docusaurus + Embedded Chatbot Integration

**Question**: How to embed a React chatbot component in Docusaurus that communicates with an external FastAPI backend?

**Decision**: Use Docusaurus custom React components with swizzling

**Rationale**:
- Docusaurus is built on React, allowing custom component injection
- Components can be placed in `/src/components/` and imported in MDX
- API calls use standard fetch/axios from the component
- CORS must be configured on FastAPI backend

**Alternatives Considered**:
- iframe embedding: Rejected - poor UX, styling issues, no text selection integration
- External chat widget (Intercom-style): Rejected - doesn't integrate with selected text
- Server-side rendering: Rejected - Docusaurus is static site generator

**Implementation Pattern**:
```jsx
// src/components/Chatbot/index.tsx
const Chatbot = () => {
  const [selectedText, setSelectedText] = useState('');
  // Listen for text selection events
  // Send to /api/chat with selected context
};
```

---

### 2. RAG Grounding with Strict Refusal

**Question**: How to implement RAG that refuses to answer when retrieval is insufficient?

**Decision**: Use confidence threshold + explicit refusal logic

**Rationale**:
- Qdrant returns similarity scores with each result
- Set minimum threshold (e.g., 0.7) for "sufficient" retrieval
- If no results above threshold, return explicit refusal
- Never pass question to LLM without grounding context

**Alternatives Considered**:
- LLM self-judgment: Rejected - hallucination risk, LLM may claim it "knows"
- Always answer with disclaimer: Rejected - violates constitution (refusal preferred)
- Hybrid retrieval: Rejected - adds complexity, free tier limits

**Refusal Logic**:
```python
def answer_question(question: str, selected_text: str = None):
    if selected_text:
        context = selected_text  # Use only selected text
    else:
        chunks = qdrant.search(embed(question), limit=5)
        relevant = [c for c in chunks if c.score > 0.7]
        if not relevant:
            return RefusalResponse(
                message="I can only answer questions from the textbook content. "
                        "This topic doesn't appear to be covered.",
                reason="insufficient_retrieval"
            )
        context = "\n".join([c.text for c in relevant])

    return generate_grounded_answer(question, context)
```

---

### 3. Qdrant Cloud Free Tier Limits

**Question**: What are the limits and how do they affect architecture?

**Decision**: Design for free tier constraints

**Rationale**:
- Free tier: 1GB storage, 1M vectors (sufficient for ~100 chapters)
- Rate limits: Adequate for 100 concurrent users
- Single cluster: No redundancy, but acceptable for hackathon

**Constraints**:
- Embedding dimension: Use text-embedding-3-small (1536 dims) for efficiency
- Chunk size: ~500 tokens to maximize semantic coherence
- Expected vectors: ~100 chunks × 1536 dims = well within limits

---

### 4. Neon Serverless Postgres Usage

**Question**: What data belongs in Postgres vs Qdrant?

**Decision**: Postgres for structured data, Qdrant for vectors only

**Rationale**:
- Qdrant: Content chunk embeddings + source metadata
- Postgres: User accounts, profiles, chat history, session state
- Separation prevents coupling and simplifies queries

**Schema Allocation**:
| Data | Storage | Reason |
|------|---------|--------|
| Content chunks | Qdrant | Vector similarity search |
| Chunk metadata | Qdrant payload | Returned with search results |
| User accounts | Postgres | Relational, auth queries |
| User profiles | Postgres | Personalization queries |
| Chat history | Postgres | Audit trail, user-specific |

---

### 5. Better Auth Integration with Docusaurus

**Question**: How to integrate Better Auth with a static Docusaurus site?

**Decision**: Better Auth runs on FastAPI backend; frontend uses session cookies

**Rationale**:
- Better Auth is a backend library
- Docusaurus calls `/api/auth/*` endpoints on FastAPI
- Session stored in HTTP-only cookies
- Frontend checks auth state via `/api/auth/session`

**Flow**:
1. User clicks "Sign Up" in Docusaurus
2. Modal/page calls `POST /api/auth/signup` with credentials
3. Backend creates user, returns session cookie
4. Frontend stores cookie, shows personalized UI
5. Subsequent API calls include cookie automatically

---

### 6. Urdu Translation Approach

**Question**: How to translate prose while preserving code blocks?

**Decision**: Use LLM translation with explicit code block preservation

**Rationale**:
- Parse MDX to identify prose vs code blocks
- Send only prose segments for translation
- Recombine with untouched code blocks
- Cache translations to avoid repeated API calls

**Alternatives Considered**:
- Pre-translate all content: Rejected - storage doubles, maintenance burden
- Google Translate API: Rejected - may miss technical nuance
- Human translation: Rejected - time/cost for hackathon

**Implementation Pattern**:
```python
def translate_chapter(chapter_mdx: str) -> str:
    segments = parse_mdx_segments(chapter_mdx)
    translated = []
    for seg in segments:
        if seg.type == "code":
            translated.append(seg.content)  # Unchanged
        else:
            translated.append(llm_translate(seg.content, target="ur"))
    return recombine_mdx(translated)
```

---

### 7. GitHub Pages + Railway CORS Configuration

**Question**: How to configure CORS between static site and API?

**Decision**: Configure FastAPI CORS middleware with GitHub Pages origin

**Rationale**:
- GitHub Pages serves from `https://<user>.github.io/<repo>/`
- FastAPI needs to allow this origin
- Also allow localhost for development

**Configuration**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://<user>.github.io",
        "http://localhost:3000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

### 8. Content Chunking Strategy

**Question**: How to chunk textbook content for optimal RAG retrieval?

**Decision**: Section-based chunking with overlap

**Rationale**:
- Each section (## heading) becomes one chunk
- Add 50-token overlap for context continuity
- Include heading as prefix for semantic clarity
- Store source chapter/section in metadata

**Chunking Rules**:
1. Split on `## ` (H2 headings)
2. If section > 1000 tokens, split on `### ` (H3)
3. If still > 1000 tokens, split on paragraphs
4. Add 50-token overlap between chunks
5. Prefix each chunk with its heading hierarchy

---

## Summary

All research questions resolved. No NEEDS CLARIFICATION items remain.

| Topic | Decision | Risk Level |
|-------|----------|------------|
| Chatbot embedding | Custom React component | Low |
| RAG refusal | Confidence threshold | Low |
| Qdrant limits | Design for free tier | Medium |
| Postgres usage | Structured data only | Low |
| Better Auth | Backend integration | Low |
| Urdu translation | LLM with code preservation | Medium |
| CORS | FastAPI middleware | Low |
| Chunking | Section-based with overlap | Low |

**Ready for Phase 1**: Data model and contract design

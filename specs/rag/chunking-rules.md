# Semantic Chunking Rules

**Phase**: 3 — Backend & RAG
**Date**: 2024-12-24
**Status**: LOCKED
**Source**: research.md (Section 8: Content Chunking Strategy)

---

## Overview

This document defines how textbook content is split into chunks for vector storage and retrieval. Proper chunking ensures semantic coherence and accurate retrieval.

---

## Chunking Strategy: Section-Based with Overlap

Each section (identified by heading) becomes one chunk, with overlap between adjacent chunks for context continuity.

---

## Chunk Boundaries

### Primary Split: H2 Headings (`##`)

Split content at every H2 heading. Each H2 section becomes one chunk.

```markdown
## What is a ROS 2 Topic? {#ros2-topic}
<-- CHUNK 1 STARTS -->
A topic is a named bus over which nodes exchange messages...
...
<-- CHUNK 1 ENDS -->

## Publishers and Subscribers {#pub-sub}
<-- CHUNK 2 STARTS -->
A publisher sends messages to a topic...
...
```

### Secondary Split: H3 Headings (`###`)

If an H2 section exceeds 1000 tokens, split at H3 headings within that section.

```markdown
## Gazebo World Building {#gazebo-world}
<-- This section is 1500 tokens, split at H3 -->

### Creating an Empty World {#empty-world}
<-- CHUNK 1 -->
...

### Adding Objects {#adding-objects}
<-- CHUNK 2 -->
...
```

### Tertiary Split: Paragraphs

If an H3 section still exceeds 1000 tokens (rare), split at paragraph boundaries.

---

## Chunk Size Constraints

| Metric | Value | Rationale |
|--------|-------|-----------|
| Minimum | 100 tokens | Avoid fragments that lack context |
| Target | 300-600 tokens | Optimal for semantic search |
| Maximum | 1000 tokens | LLM context window efficiency |

**Token Counting**: Use OpenAI tiktoken with `cl100k_base` encoding (same as text-embedding-3-small).

---

## Overlap Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Overlap size | 50 tokens | Preserve context at boundaries |
| Overlap direction | Trailing | End of previous chunk overlaps into next |

**Example**:
```
Chunk N:   [...tokens 1-500...] [tokens 451-500 overlap]
Chunk N+1: [tokens 451-500 overlap] [...tokens 501-1000...]
```

---

## Heading Prefix

Each chunk MUST include its heading hierarchy as a prefix for semantic clarity.

**Format**:
```
[Module: Physical AI Foundations]
[Chapter: Embodied Intelligence]
[Section: What is Physical AI?]

<actual content>
```

**Implementation**:
```python
def create_chunk(content: str, module: str, chapter: str, section: str) -> str:
    prefix = f"[Module: {module}]\n[Chapter: {chapter}]\n[Section: {section}]\n\n"
    return prefix + content
```

---

## Metadata Schema

Each chunk stored in Qdrant includes:

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| id | UUID | Unique chunk identifier | `550e8400-e29b-...` |
| vector | float[1536] | Embedding from text-embedding-3-small | `[0.012, -0.003, ...]` |
| text | string | Full chunk content with prefix | `[Module: ...]` |
| module_id | string | Module slug | `module-2-ros2` |
| chapter_id | string | Chapter filename | `chapter-2-topics-pubsub` |
| section_heading | string | Original H2/H3 heading | `## What is a ROS 2 Topic?` |
| chunk_index | int | Order within chapter | `0, 1, 2, ...` |

---

## One-Primary-Concept Rule

Each chunk should express ONE primary concept. If a section covers multiple distinct concepts, consider manual review.

**Signs of multi-concept sections**:
- Multiple H3 headings with unrelated topics
- Transition phrases like "Now let's look at something different"
- Significant topic shifts mid-paragraph

**Resolution**: Split at concept boundaries during ingestion review.

---

## Content Exclusions

The following content is **NOT** chunked:

| Content Type | Reason |
|--------------|--------|
| YAML frontmatter | Metadata, not content |
| Navigation elements | "What's Next", "Prerequisites" |
| Self-assessment questions | May leak answer format |
| Table of contents | Redundant with metadata |

---

## Code Block Handling

Code blocks are included in chunks as-is. They provide important context for technical questions.

```markdown
## Creating a Publisher

To create a publisher, use the following pattern:

\`\`\`python
publisher = self.create_publisher(String, 'topic', 10)
\`\`\`

This creates a publisher that sends String messages to 'topic'.
```

**Full section (including code) becomes one chunk.**

---

## Chunking Algorithm

```python
def chunk_chapter(chapter_content: str, module_id: str, chapter_id: str) -> List[Chunk]:
    chunks = []
    sections = split_by_h2(chapter_content)

    for section_idx, section in enumerate(sections):
        heading = extract_heading(section)
        content = remove_excluded_content(section)

        if count_tokens(content) > 1000:
            # Split by H3 if too large
            subsections = split_by_h3(section)
            for sub_idx, subsection in enumerate(subsections):
                chunk = create_chunk_with_overlap(
                    content=subsection,
                    module_id=module_id,
                    chapter_id=chapter_id,
                    section_heading=heading,
                    chunk_index=section_idx * 100 + sub_idx
                )
                chunks.append(chunk)
        else:
            chunk = create_chunk_with_overlap(
                content=content,
                module_id=module_id,
                chapter_id=chapter_id,
                section_heading=heading,
                chunk_index=section_idx
            )
            chunks.append(chunk)

    return chunks
```

---

## Expected Chunk Count

Based on Phase 2 content (46 files):

| Module | Chapters | Estimated Chunks |
|--------|----------|------------------|
| Module 1 | 4 | 20-30 |
| Module 2 | 6 | 35-50 |
| Module 3 | 6 | 35-50 |
| Module 4 | 6 | 35-50 |
| Module 5 | 6 | 35-50 |
| Capstone | 5 | 25-35 |
| Appendix | 4 | 20-30 |
| Glossary | 1 | 10-15 |
| **Total** | 38 | **~215-310 chunks** |

Well within Qdrant free tier limits (1M vectors).

---

## Validation Checklist

Before ingestion, verify:

- [ ] All chunks have 100-1000 tokens
- [ ] Heading prefixes are correctly formatted
- [ ] Overlap is 50 tokens between adjacent chunks
- [ ] Metadata includes all required fields
- [ ] Code blocks are preserved intact
- [ ] Excluded content is removed
- [ ] chunk_index is sequential within chapter

---

## Sign-off

These chunking rules are **LOCKED** for Phase 3 implementation.

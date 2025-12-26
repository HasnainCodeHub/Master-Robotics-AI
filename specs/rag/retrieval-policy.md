# Retrieval Policy Specification

**Phase**: 3 — Backend & RAG
**Date**: 2024-12-24
**Status**: LOCKED
**Constitution Reference**: Principle III (RAG Safety - NON-NEGOTIABLE)

---

## Overview

This document defines the retrieval-only answering policy for the Physical AI & Humanoid Robotics textbook RAG chatbot. These rules are **NON-NEGOTIABLE** and must be enforced at the code level.

---

## Core Principles

### 1. Retrieval-Only Answering

The chatbot MUST answer questions using ONLY content retrieved from the textbook vector database.

**Enforcement**:
```python
# CORRECT: Only use retrieved context
answer = generate_answer(question, retrieved_chunks)

# WRONG: Never do this
answer = generate_answer(question)  # No context = FORBIDDEN
```

### 2. No External Knowledge

The LLM MUST NOT use any knowledge from its training data. The system prompt must explicitly prohibit this.

**System Prompt Template**:
```
You are a teaching assistant for the Physical AI & Humanoid Robotics textbook.

RULES (NON-NEGOTIABLE):
1. Answer ONLY using the provided context
2. If the context does not contain the answer, say "I can only answer questions from the textbook content"
3. NEVER use information from your training data
4. NEVER speculate or infer beyond the provided text
5. NEVER say "I think" or "In general" - only cite the textbook

CONTEXT:
{retrieved_chunks}

QUESTION:
{user_question}
```

### 3. No Inference Beyond Retrieved Text

The chatbot MUST NOT make logical inferences, extrapolations, or connections that are not explicitly stated in the retrieved content.

**Examples**:
| Question | Retrieved Context | Correct Response |
|----------|-------------------|------------------|
| "What is a ROS 2 topic?" | "A topic is a named bus..." | "According to the textbook, a topic is a named bus..." |
| "Is ROS 2 better than ROS 1?" | (only ROS 2 content) | "I can only answer about ROS 2 based on the textbook. The course does not compare ROS 1 and ROS 2." |
| "Can I use ROS 2 on Windows?" | (no Windows content) | "I can only answer questions from the textbook content. This topic doesn't appear to be covered." |

### 4. Refusal Priority Over Partial Answers

When retrieval is insufficient, the chatbot MUST refuse to answer rather than provide a partial or speculative response.

**Priority Order**:
1. **REFUSE** if no relevant chunks retrieved
2. **REFUSE** if confidence score < 0.7
3. **REFUSE** if context doesn't directly answer the question
4. **ANSWER** only if context fully supports the response

---

## Retrieval Confidence Threshold

| Threshold | Action |
|-----------|--------|
| score >= 0.7 | Use chunk for answer generation |
| 0.5 <= score < 0.7 | Log warning, exclude from context |
| score < 0.5 | Ignore completely |

**Threshold Rationale**: 0.7 balances precision (avoiding irrelevant content) with recall (capturing relevant content). This threshold was chosen based on typical embedding model behavior with technical content.

---

## Selected Text Mode

When the user selects text and asks a question:

1. **ONLY** use the selected text as context
2. **DO NOT** retrieve additional chunks from the vector database
3. If selected text is insufficient to answer, **REFUSE**

**Implementation**:
```python
if selected_text:
    context = selected_text  # Use ONLY this
else:
    context = retrieve_chunks(question)  # Normal retrieval
```

---

## Prohibited Patterns

The following response patterns are **FORBIDDEN**:

| Pattern | Why Prohibited |
|---------|----------------|
| "In general, ..." | Implies external knowledge |
| "Typically, ..." | Implies experience beyond text |
| "I believe that ..." | Introduces speculation |
| "Based on my understanding ..." | Implies training knowledge |
| "You might also consider ..." | Goes beyond retrieved content |
| "As a best practice ..." | Implies external expertise |

---

## Audit Trail

Every response MUST include:

1. **was_refusal**: Boolean flag indicating if the response was a refusal
2. **sources**: List of chunk IDs and similarity scores used
3. **refusal_reason**: If refused, the specific reason (empty_retrieval, low_confidence, out_of_scope, insufficient_context)

**Response Schema**:
```json
{
  "answer": "string",
  "was_refusal": false,
  "sources": [
    {"chunk_id": "uuid", "score": 0.89, "section": "..."}
  ],
  "refusal_reason": null
}
```

---

## Validation Checklist

Before deployment, verify:

- [ ] LLM never called without retrieved context
- [ ] Confidence threshold (0.7) enforced in code
- [ ] Refusal messages are user-friendly
- [ ] was_refusal flag set correctly
- [ ] Selected text mode uses ONLY selected text
- [ ] System prompt explicitly prohibits external knowledge
- [ ] Prohibited patterns are never generated

---

## Sign-off

This retrieval policy is **LOCKED** and **NON-NEGOTIABLE**.

Any violation of these rules is a critical safety failure.

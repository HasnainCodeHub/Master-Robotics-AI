# Refusal Rules Specification

**Phase**: 3 — Backend & RAG
**Date**: 2024-12-24
**Status**: LOCKED
**Constitution Reference**: Principle III (RAG Safety - NON-NEGOTIABLE)

---

## Overview

This document defines when and how the RAG chatbot must refuse to answer. Refusal is ALWAYS preferred over speculation or partial answers.

---

## Refusal Triggers

### 1. Empty Retrieval

**Condition**: Vector search returns zero chunks above the minimum threshold.

**Threshold**: No chunks with score >= 0.5

**Response**:
```json
{
  "answer": "I can only answer questions from the textbook content. This topic doesn't appear to be covered in the course materials.",
  "was_refusal": true,
  "sources": [],
  "refusal_reason": "empty_retrieval"
}
```

### 2. Insufficient Context

**Condition**: Retrieved chunks exist but none exceed the confidence threshold for answer generation.

**Threshold**: All chunks have score < 0.7

**Response**:
```json
{
  "answer": "I found some related content, but not enough to provide a confident answer. Please try rephrasing your question or selecting specific text from the chapter.",
  "was_refusal": true,
  "sources": [],
  "refusal_reason": "insufficient_context"
}
```

### 3. Out-of-Scope Query

**Condition**: The question explicitly asks about topics outside the curriculum.

**Detection**: Keyword/pattern matching for known out-of-scope topics.

**Out-of-Scope Topics** (from spec.md):
- Advanced control theory (PID tuning depth, optimal control, MPC internals)
- Low-level motor control (PWM, current control, driver implementation)
- Custom hardware design (PCB design, mechanical CAD, sensor fabrication)
- Training foundation models
- ROS 1 (course is ROS 2 only)
- Other simulation platforms not covered (Webots, CoppeliaSim)

**Response**:
```json
{
  "answer": "This topic is outside the scope of this course. The Physical AI & Humanoid Robotics textbook focuses on [relevant scope summary]. For advanced control theory, please refer to specialized resources.",
  "was_refusal": true,
  "sources": [],
  "refusal_reason": "out_of_scope"
}
```

### 4. Selected-Text-Only Violations

**Condition**: User provides selected text, but the text is insufficient to answer the question.

**Detection**: Selected text provided but doesn't contain relevant information.

**Response**:
```json
{
  "answer": "The selected text doesn't contain enough information to answer your question. Please select a different section or ask without selected text to search the full textbook.",
  "was_refusal": true,
  "sources": [],
  "refusal_reason": "selected_text_insufficient"
}
```

---

## Refusal Reason Enum

```python
class RefusalReason(str, Enum):
    EMPTY_RETRIEVAL = "empty_retrieval"
    INSUFFICIENT_CONTEXT = "insufficient_context"
    OUT_OF_SCOPE = "out_of_scope"
    SELECTED_TEXT_INSUFFICIENT = "selected_text_insufficient"
```

---

## Refusal Message Templates

### Empty Retrieval
```
I can only answer questions from the textbook content. This topic doesn't appear to be covered in the course materials.

If you're looking for information on {detected_topic}, this may be outside the scope of the Physical AI & Humanoid Robotics course.
```

### Insufficient Context
```
I found some related content, but not enough to provide a confident answer.

You can try:
1. Rephrasing your question with more specific terms
2. Selecting specific text from the chapter for targeted explanation
3. Asking about a narrower aspect of the topic
```

### Out of Scope
```
This topic is outside the scope of this course.

The Physical AI & Humanoid Robotics textbook covers:
- Physical AI Foundations
- ROS 2 Fundamentals
- Simulation & Digital Twin
- NVIDIA Isaac Ecosystem
- Vision-Language-Action Systems
- Integrated Humanoid Capstone

For {out_of_scope_topic}, please refer to specialized resources.
```

### Selected Text Insufficient
```
The selected text doesn't contain enough information to answer your question.

You can:
1. Select a different section that covers this topic
2. Remove the selection to search the full textbook
3. Rephrase your question based on what's in the selected text
```

---

## Refusal Flow

```
User sends question
    │
    ▼
Is selected_text provided?
    │
    ├── YES: Use only selected_text
    │   │
    │   └── Is selected_text sufficient?
    │       │
    │       ├── NO → REFUSE (selected_text_insufficient)
    │       │
    │       └── YES → Generate answer
    │
    └── NO: Retrieve from Qdrant
        │
        ▼
    Any chunks retrieved?
        │
        ├── NO → REFUSE (empty_retrieval)
        │
        └── YES: Check confidence scores
            │
            ▼
        Any chunks with score >= 0.7?
            │
            ├── NO → REFUSE (insufficient_context)
            │
            └── YES: Check if out-of-scope
                │
                ▼
            Is question out-of-scope?
                │
                ├── YES → REFUSE (out_of_scope)
                │
                └── NO → Generate answer from chunks
```

---

## Implementation Pattern

```python
def process_question(question: str, selected_text: str = None) -> ChatResponse:
    # 1. Handle selected text mode
    if selected_text:
        if not is_sufficient(selected_text, question):
            return RefusalResponse(
                message=SELECTED_TEXT_INSUFFICIENT_MESSAGE,
                reason=RefusalReason.SELECTED_TEXT_INSUFFICIENT
            )
        context = selected_text
    else:
        # 2. Retrieve chunks
        chunks = qdrant.search(embed(question), limit=5)

        # 3. Check for empty retrieval
        if not chunks or all(c.score < 0.5 for c in chunks):
            return RefusalResponse(
                message=EMPTY_RETRIEVAL_MESSAGE,
                reason=RefusalReason.EMPTY_RETRIEVAL
            )

        # 4. Filter by confidence threshold
        relevant = [c for c in chunks if c.score >= 0.7]

        if not relevant:
            return RefusalResponse(
                message=INSUFFICIENT_CONTEXT_MESSAGE,
                reason=RefusalReason.INSUFFICIENT_CONTEXT
            )

        # 5. Check for out-of-scope
        if is_out_of_scope(question):
            return RefusalResponse(
                message=OUT_OF_SCOPE_MESSAGE.format(topic=detect_topic(question)),
                reason=RefusalReason.OUT_OF_SCOPE
            )

        context = "\n".join([c.text for c in relevant])

    # 6. Only reach here if we have sufficient context
    return generate_grounded_answer(question, context, sources=relevant)
```

---

## Out-of-Scope Detection

### Keyword Patterns

```python
OUT_OF_SCOPE_PATTERNS = [
    # Control theory
    r"\b(PID tuning|optimal control|MPC|model predictive)\b",
    r"\b(Kalman filter|LQR|sliding mode)\b",

    # Low-level motor control
    r"\b(PWM|pulse width|current control|motor driver)\b",
    r"\b(H-bridge|MOSFET|brushless DC)\b",

    # Hardware design
    r"\b(PCB|printed circuit|mechanical CAD|SolidWorks)\b",
    r"\b(fabrication|3D print|machining)\b",

    # Training models
    r"\b(train|training|fine-tune|finetuning)\s+(a\s+)?(model|LLM|neural)\b",
    r"\b(backpropagation|gradient descent|loss function)\b",

    # ROS 1
    r"\bROS\s*1\b",
    r"\b(catkin|rosbuild|roslaunch.*\.launch)\b",

    # Other simulators
    r"\b(Webots|CoppeliaSim|V-REP|PyBullet)\b",
]
```

---

## Logging Requirements

Every refusal MUST be logged with:

| Field | Value |
|-------|-------|
| timestamp | ISO 8601 |
| session_id | UUID |
| question | User's original question |
| refusal_reason | Enum value |
| chunks_retrieved | Count (0 if empty) |
| max_score | Highest chunk score (null if empty) |

**Log Format**:
```json
{
  "event": "refusal",
  "timestamp": "2024-12-24T10:30:00Z",
  "session_id": "550e8400-...",
  "question": "How do I train a custom LLM?",
  "refusal_reason": "out_of_scope",
  "chunks_retrieved": 3,
  "max_score": 0.45
}
```

---

## Validation Checklist

Before deployment, verify:

- [ ] Empty retrieval triggers refusal (not LLM call)
- [ ] Low-confidence chunks trigger refusal
- [ ] Out-of-scope patterns are detected
- [ ] Selected text mode refuses when insufficient
- [ ] Refusal messages are user-friendly
- [ ] was_refusal flag is set correctly
- [ ] refusal_reason is populated
- [ ] All refusals are logged
- [ ] LLM is NEVER called on refusal path

---

## Sign-off

These refusal rules are **LOCKED** and **NON-NEGOTIABLE**.

Refusal is ALWAYS preferred over speculation.

---
name: retrieval-guardrails
description: Ensure answers are grounded strictly in retrieved content.
---

## Purpose
Prevent hallucinations in RAG responses.

## When to Use
- Answering user questions
- Using selected-text context

## How It Works
1. Limit context to retrieved text
2. Refuse unsupported answers

## Output
Grounded responses only.

---
id: ml-concepts-roboticists
title: "Appendix A: ML Concepts for Roboticists"
sidebar_label: "A. ML Concepts"
sidebar_position: 2
---

# Appendix A: Machine Learning Concepts for Roboticists

This appendix provides a conceptual foundation for understanding the machine learning components used in this course, particularly in Module 5 (Vision-Language-Action Systems). You don't need to implement these systems from scratch—the goal is understanding how to use them effectively.

---

## What You Need to Know (and What You Don't) {#scope}

### Required Understanding

| Concept | Why It Matters |
|---------|----------------|
| What neural networks do | Understand capability boundaries |
| How embeddings work | Configure VLM queries effectively |
| Why transformers process sequences | Debug context length issues |
| What latency to expect | Budget time in VLA pipeline |

### Not Required

- Backpropagation mathematics
- Model training procedures
- Architecture design decisions
- Loss function derivation

---

## Neural Networks: Function Approximators {#neural-networks}

### The Core Idea

A neural network is a **function approximator**—it learns to map inputs to outputs from examples.

```
Input (image) → Neural Network → Output (classification)
Input (text)  → Neural Network → Output (text)
Input (audio) → Neural Network → Output (transcript)
```

### Why This Matters for Robotics

Neural networks can learn mappings that are difficult to program explicitly:

| Task | Traditional Approach | Neural Network Approach |
|------|---------------------|------------------------|
| Object detection | Hand-coded features + rules | Learn features from examples |
| Speech recognition | Acoustic models + grammars | End-to-end learning |
| Navigation | Explicit path planning | Learn from demonstrations |

### Key Limitation: Training Distribution

Neural networks work well when inputs match their training data. They may fail unpredictably on:
- Novel objects not in training set
- Unusual lighting or camera angles
- Accents or background noise not in training
- Environments very different from simulation

**Implication**: Always verify neural network outputs in safety-critical applications.

---

## Embeddings: Meaning as Numbers {#embeddings}

### What Are Embeddings?

Embeddings are **numerical representations of meaning**. They convert text, images, or other data into vectors (lists of numbers) where similar items have similar vectors.

```
"dog"  → [0.2, 0.8, 0.1, ...]
"cat"  → [0.3, 0.7, 0.2, ...]  ← Similar to dog
"car"  → [0.9, 0.1, 0.3, ...]  ← Different from dog
```

### Similarity Through Distance

Similar concepts have embeddings that are close in vector space:

```
cosine_similarity("dog", "cat") = 0.92  ← High (similar)
cosine_similarity("dog", "car") = 0.21  ← Low (different)
```

### Why This Matters for Robotics

**Semantic Search**: Find relevant information by meaning, not just keywords.

```
Query: "How do I make the robot arm move?"
Embedding matches:
  - "Joint trajectory control"
  - "MoveIt motion planning"
  - "Arm kinematics"
```

**VLM Grounding**: Match language to visual features.

```
"the red mug" → embedding → find object with similar visual embedding
```

### Key Numbers to Know

| Model | Embedding Dimensions | Typical Use |
|-------|---------------------|-------------|
| OpenAI text-embedding-3 | 1536 | Text similarity |
| CLIP | 512 | Image-text matching |
| Sentence-BERT | 384-768 | Semantic search |

---

## Transformers: Context Matters {#transformers}

### The Problem: Order and Context

Traditional neural networks process each input independently. But meaning often depends on context:

```
"I saw the bat" - Is it an animal or sports equipment?
"Pick up the mug" - Which mug if there are several?
```

### The Solution: Attention

Transformers use **attention** to relate each part of the input to every other part:

```
"The red mug on the left"
      │      │        │
      └──────┼────────┘
             │
    attention: "red" and "left" both describe "mug"
```

### Why This Matters for Robotics

**LLM Task Planning**: The model considers the entire command when generating plans.

```
"Pick up the mug and put it on the shelf"
                    │
                    └── "it" refers to "mug" (resolved via attention)
```

**VLM Scene Understanding**: The model relates image regions to text queries.

```
Image: [table, mug, keyboard, mouse]
Query: "What is next to the keyboard?"
Answer: "mouse" (attention links spatial relationship to objects)
```

### Key Limitations

**Context Window**: Maximum input length (e.g., 8K, 32K tokens). Plan for truncation.

**Latency**: Scales with sequence length. Longer inputs = slower responses.

---

## Model Types in VLA Systems {#model-types}

### Speech Recognition (Whisper)

**Input**: Audio waveform (16kHz samples)
**Output**: Text transcript

```
Audio: [waveform samples] → Whisper → "Pick up the red mug"
```

**Key Parameters**:
- Model size: tiny (fastest) to large (most accurate)
- Language: English, multilingual
- Task: transcribe (same language) or translate (to English)

### Large Language Models (LLMs)

**Input**: Text (prompt + context)
**Output**: Text (completion)

```
Prompt: "Convert to robot commands: pick up the mug"
Output: {"action": "pick", "target": "mug"}
```

**Key Parameters**:
- Temperature: 0 (deterministic) to 1 (creative)
- Max tokens: Output length limit
- System prompt: Instructions that guide behavior

### Vision-Language Models (VLMs)

**Input**: Image + Text query
**Output**: Text answer about the image

```
Image: [photo of desk]
Query: "Where is the red object?"
Output: "The red mug is on the left side of the desk"
```

**Key Considerations**:
- Latency: 1-10 seconds typical
- Hallucination: May describe objects that don't exist
- Resolution: Image may be downsampled internally

---

## Latency Expectations {#latency}

### Typical Inference Times

| Component | Model | Typical Latency |
|-----------|-------|-----------------|
| Speech (Whisper) | base | 100-200ms |
| Speech (Whisper) | small | 250-500ms |
| LLM (GPT-4) | API | 500-2000ms |
| LLM (local 7B) | GPU | 200-500ms |
| VLM (LLaVA) | 7B, GPU | 1000-3000ms |
| VLM (GPT-4V) | API | 2000-5000ms |
| Embedding | text | 10-50ms |
| Object Detection | YOLOv8 | 10-30ms |

### Budget Allocation for VLA

```
Voice command to robot action:
├── Speech capture: 2000ms (user speaking)
├── Whisper: 200ms
├── LLM planning: 500ms
├── VLM (if needed): 2000ms
├── Grounding: 50ms
├── Safety check: 10ms
└── Total: ~2760ms (without VLM) or ~4760ms (with VLM)
```

---

## Common Failure Modes {#failures}

### Hallucination

**What**: Model generates plausible but incorrect information.

**Example**:
```
VLM: "I see three mugs on the table"
Reality: Only two mugs present
```

**Mitigation**: Cross-check with object detection; don't trust counts.

### Out-of-Distribution

**What**: Input differs from training data.

**Example**:
```
Trained on: Indoor office environments
Deployed in: Industrial warehouse
Result: Poor object recognition
```

**Mitigation**: Test on target environment; use domain randomization.

### Context Length Overflow

**What**: Input exceeds model's maximum context.

**Example**:
```
Long conversation + large image + detailed prompt = truncation
```

**Mitigation**: Summarize history; limit prompt size.

### Latency Spike

**What**: Unexpectedly slow inference.

**Example**:
```
Normal: 500ms
Spike: 5000ms (due to API throttling or GPU contention)
```

**Mitigation**: Set timeouts; implement fallbacks.

---

## Practical Guidelines {#guidelines}

### When to Use Neural Networks

**Good Fit**:
- Pattern recognition (objects, speech, faces)
- Semantic understanding (language, scenes)
- Complex mappings difficult to hand-code

**Poor Fit**:
- Safety-critical decisions (use traditional verification)
- Precise numeric computation (use explicit calculation)
- Real-time control loops (latency may be too high)

### Integration Best Practices

1. **Validate outputs**: Never trust neural network output for safety
2. **Set timeouts**: Handle slow/failed inference gracefully
3. **Log everything**: Record inputs and outputs for debugging
4. **Test boundaries**: Identify where models fail
5. **Cache when possible**: Repeated queries → cached results

### Questions to Ask Before Using a Model

| Question | Why It Matters |
|----------|----------------|
| What was it trained on? | Predicts where it will work |
| What's the latency? | Fits your timing budget? |
| What's the failure mode? | How to handle errors? |
| What hardware does it need? | GPU requirements |
| What's the cost? | API fees, compute costs |

---

## Summary

For effective use of ML in robotics, understand:

1. **Neural networks** approximate functions from data—they may fail on unfamiliar inputs.

2. **Embeddings** represent meaning as numbers—enabling semantic similarity search.

3. **Transformers** use attention to process context—enabling language and vision understanding.

4. **Latency varies significantly**—budget time carefully in VLA pipelines.

5. **Failure modes are predictable**—design systems to handle hallucination, timeouts, and out-of-distribution inputs.

You don't need to train these models—but you do need to understand their capabilities and limitations to use them safely and effectively in robotic systems.

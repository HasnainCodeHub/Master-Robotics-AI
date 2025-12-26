# Personalization Rules Specification

**Feature**: Presentation-Level Personalization
**Phase**: 5 - Authentication & Personalization (Bonus)
**Owner**: identity-personalization agent
**Created**: 2024-12-24
**Updated**: 2024-12-25
**Status**: LOCKED

---

## Overview

This specification defines how user profile influences content PRESENTATION ONLY. Personalization MUST NOT affect RAG retrieval, answer generation, or curriculum meaning.

> **CRITICAL NON-NEGOTIABLE PRINCIPLE**
>
> RAG outputs MUST be IDENTICAL for all users. Personalization is PRESENTATION-ONLY.
> This is not a guideline. This is an absolute requirement.

---

## Core Architecture: Separation of Concerns

```
+------------------+     +------------------+     +------------------+
|   User Request   | --> |   RAG Pipeline   | --> |  Presentation    |
|   (no profile)   |     |  (user-blind)    |     |  Layer (profile) |
+------------------+     +------------------+     +------------------+
                                |                        |
                                v                        v
                         IDENTICAL OUTPUT         STYLED OUTPUT
                         (same for all)           (per user level)
```

The RAG pipeline has **NO KNOWLEDGE** of user profile.
Personalization happens **AFTER** the answer is generated.

---

## Section 1: Allowed Presentation Adjustments

### 1.1 Explanation Depth

**Based on**: `software_level` + `robotics_level`

| Combined Level | Adjustment |
|---------------|------------|
| Both Beginner | Show expanded explanations, more analogies, expand acronyms |
| Mixed | Show standard explanations |
| Both Advanced | Show concise explanations, hide introductory scaffolding |

**Implementation**:
```tsx
// BeginnerHint component - shown only for beginners
<BeginnerHint visible={profile.software_level === 'beginner'}>
  Additional explanation for beginners...
</BeginnerHint>

// AdvancedDeepDive component - shown only for advanced
<AdvancedDeepDive visible={profile.software_level === 'advanced'}>
  Advanced implementation details...
</AdvancedDeepDive>
```

### 1.2 Optional Hints

**Based on**: `software_level`

| Level | Hint Behavior |
|-------|--------------|
| Beginner | Hints expanded by default |
| Intermediate | Hints collapsed, click to expand |
| Advanced | Hints hidden, toggle to show |

**Implementation**:
```tsx
<Hint
  defaultExpanded={profile.software_level === 'beginner'}
  hidden={profile.software_level === 'advanced' && !showAdvancedHints}
>
  Helpful context about this concept...
</Hint>
```

### 1.3 Code Example Comments

**Based on**: `software_level`

| Level | Code Display |
|-------|-------------|
| Beginner | Full code with extensive inline comments |
| Intermediate | Standard code with key comments |
| Advanced | Minimal comments, optional verbose mode |

**Implementation**:
- CSS classes toggle comment visibility
- NO code content changes, only display

```css
/* Beginner sees all comments */
[data-level="beginner"] .code-comment { display: block; }
[data-level="beginner"] .code-comment-basic { display: block; }

/* Intermediate sees key comments */
[data-level="intermediate"] .code-comment { display: block; }
[data-level="intermediate"] .code-comment-basic { display: none; }

/* Advanced sees minimal comments */
[data-level="advanced"] .code-comment { display: none; }
[data-level="advanced"] .code-comment-basic { display: none; }
```

### 1.4 Terminology Tooltips

**Based on**: `robotics_level`

| Level | Tooltip Behavior |
|-------|-----------------|
| Beginner | Auto-show tooltips on hover |
| Intermediate | Show tooltips on hover |
| Advanced | Tooltips on click only |

**Implementation**:
```tsx
<Term
  definition="A ROS 2 message passing mechanism"
  autoShow={profile.robotics_level === 'beginner'}
>
  topic
</Term>
```

### 1.5 Hardware-Aware Hints

**Based on**: `hardware_access`

| Hardware | Example Priority |
|----------|-----------------|
| Simulation Only | Highlight Gazebo/Isaac examples |
| Jetson Device | Highlight Jetson deployment notes |
| Physical Robot | Highlight hardware integration examples |

**Implementation**:
- ALL examples remain visible
- Relevant examples highlighted with "Recommended for you" badge
- No content is hidden based on hardware

---

## Section 2: Presentation Personalization Examples

### 2.1 Same RAG Answer, Different Presentation

The RAG system returns **IDENTICAL** answers. The presentation layer styles them differently.

**Example 1: Acronym Expansion**

RAG returns:
```
ROS 2 is a middleware framework that provides communication
primitives for building robotic systems.
```

Beginner sees:
```
ROS 2 (Robot Operating System 2) is a middleware framework that
provides communication primitives for building robotic systems.

[Beginner Hint: A middleware framework acts as a "glue" that
helps different parts of your robot software talk to each other.]
```

Advanced sees:
```
ROS 2 is a middleware framework that provides communication
primitives for building robotic systems.
```

**The RAG output is identical. Only the presentation differs.**

---

**Example 2: Code with Different Comment Visibility**

RAG returns code example:
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
```

Beginner sees (with expanded comments):
```python
# Import the ROS 2 Python client library
import rclpy
# Import the base Node class that all ROS 2 nodes inherit from
from rclpy.node import Node

# Define our publisher node by inheriting from Node
class MinimalPublisher(Node):
    def __init__(self):
        # Call the parent class constructor with our node name
        super().__init__('minimal_publisher')
        # Create a publisher that sends String messages to 'topic'
        # The '10' is the queue size - how many messages to buffer
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        # Create a timer that calls timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)

    # This method is called every 0.5 seconds by the timer
    def timer_callback(self):
        # Create a new String message
        msg = String()
        # Set the message content
        msg.data = 'Hello World'
        # Publish the message to all subscribers
        self.publisher_.publish(msg)
```

Advanced sees (minimal comments):
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
```

**The code is identical. Comments are added/removed at presentation layer ONLY.**

---

**Example 3: Terminology Presentation**

RAG returns:
```
The node publishes messages to a topic, which subscribers can listen to.
```

Beginner sees:
```
The node publishes messages to a topic [tooltip: A named channel
for sending messages between nodes], which subscribers [tooltip:
Programs that listen for messages on a topic] can listen to.
```

Advanced sees:
```
The node publishes messages to a topic, which subscribers can listen to.
```

**The text is identical. Tooltips are UI decoration.**

---

## Section 3: EXPLICIT FORBIDDEN ACTIONS (NON-NEGOTIABLE)

These actions are **ABSOLUTELY FORBIDDEN** and constitute critical safety failures.

### 3.1 RAG Query Manipulation

```python
# FORBIDDEN - NEVER DO THIS
def retrieve_chunks(question: str, user_level: str):
    # Adjusting query based on user level = VIOLATION
    if user_level == 'beginner':
        question = f"explain simply: {question}"  # FORBIDDEN
    return vector_db.search(question)
```

**Rule**: The retrieval query MUST be EXACTLY the user's question. No modification.

### 3.2 Chunk Filtering Based on Profile

```python
# FORBIDDEN - NEVER DO THIS
def filter_chunks(chunks: list, user_profile: dict):
    if user_profile['level'] == 'beginner':
        # Filtering out "advanced" chunks = VIOLATION
        return [c for c in chunks if c.difficulty != 'advanced']
    return chunks
```

**Rule**: ALL retrieved chunks MUST be used. No filtering by user profile.

### 3.3 LLM Prompt Modification

```python
# FORBIDDEN - NEVER DO THIS
def generate_answer(question: str, chunks: list, user_profile: dict):
    if user_profile['level'] == 'beginner':
        system_prompt = "Explain in simple terms..."  # VIOLATION
    else:
        system_prompt = "Be concise..."
    return llm.generate(system_prompt, question, chunks)
```

**Rule**: The LLM prompt MUST be IDENTICAL for all users.

### 3.4 Semantic Meaning Changes

```
FORBIDDEN:
- "ROS 2 is complex" -> "ROS 2 is easy" (for beginners)
- Removing technical details for beginners
- Adding caveats or warnings based on user level
- Simplifying explanations at the answer generation stage
```

**Rule**: The semantic meaning of answers MUST be preserved.

### 3.5 Adding External Facts

```
FORBIDDEN:
- Adding context not in retrieved chunks
- Including "helpful tips" not in the textbook
- Supplementing answers with general knowledge
```

**Rule**: ONLY retrieved content may be in the answer.

### 3.6 Information Hiding

```
FORBIDDEN:
- Hiding advanced concepts from beginners
- Blocking content based on user level
- Gating curriculum sections by experience
```

**Rule**: ALL content is accessible to ALL users.

---

## Section 4: Implementation Rules

### 4.1 Personalization Timing

```
1. User asks question
2. RAG retrieves chunks (NO user profile)
3. LLM generates answer (NO user profile)
4. Answer returned (IDENTICAL for all users)
5. Presentation layer applies styling (user profile used HERE ONLY)
6. User sees styled answer
```

Personalization happens at step 5 ONLY.

### 4.2 Pure Presentation Layer

The personalization code:
- Receives the COMPLETE, UNMODIFIED RAG answer
- Applies CSS/visual styling only
- Adds/hides UI elements (hints, tooltips)
- Expands/collapses sections
- NEVER modifies the answer text

### 4.3 User Toggle

Users MUST be able to disable personalization:

```tsx
<PersonalizationToggle
  enabled={profile.personalization_enabled}
  onChange={(enabled) => updateProfile({ personalization_enabled: enabled })}
/>
```

When disabled:
- Show content as if user is "intermediate" level
- All optional hints collapsed
- Standard tooltips
- All examples shown equally
- Same experience as anonymous users

### 4.4 Code Separation

```
backend/
  rag/
    retrieval.py      # NO user profile imports
    generation.py     # NO user profile imports

frontend/
  presentation/
    PersonalizationProvider.tsx   # User profile used HERE
    BeginnerHint.tsx
    AdvancedDeepDive.tsx
    TermTooltip.tsx
```

RAG code and personalization code are in SEPARATE modules with no imports between them.

---

## Section 5: Audit Requirements

### 5.1 RAG Pipeline Isolation

**Requirement**: The RAG pipeline MUST have no knowledge of user profile.

**Verification**:
```bash
# Check that RAG files never import user profile
grep -r "user_profile\|UserProfile\|profile" backend/rag/
# Expected: NO MATCHES
```

**Code Review Checklist**:
- [ ] `retrieval.py` has no user profile parameters
- [ ] `generation.py` has no user profile parameters
- [ ] System prompt is hardcoded, not user-dependent
- [ ] Chunk retrieval query is user's exact question

### 5.2 Profile Data Isolation

**Requirement**: Profile data is NEVER sent to embedding or LLM services.

**Verification**:
```python
# Log all API calls to OpenAI/embedding service
# Verify user profile fields are NEVER in the payload

def audit_api_call(payload: dict):
    forbidden_fields = ['software_level', 'robotics_level', 'hardware_access', 'user_id']
    for field in forbidden_fields:
        if field in str(payload):
            raise AuditViolation(f"Profile field {field} in API payload")
```

### 5.3 Answer Identity Verification

**Requirement**: Same question produces identical RAG answers for all users.

**Test Case**:
```python
def test_answer_identity():
    question = "What is a ROS 2 topic?"

    # Get answer as beginner
    answer_beginner = rag_service.answer(question)

    # Get answer as advanced
    answer_advanced = rag_service.answer(question)

    # Answers MUST be identical
    assert answer_beginner == answer_advanced, "RAG answers differ by user!"
```

### 5.4 Audit Logging

Every RAG request MUST log:
```json
{
  "request_id": "uuid",
  "question": "user question",
  "user_profile_in_request": false,
  "chunks_retrieved": ["chunk_id_1", "chunk_id_2"],
  "answer_hash": "sha256_of_answer"
}
```

If `user_profile_in_request` is ever `true`, trigger security alert.

---

## Section 6: Frontend Components

### 6.1 Component Reference

| Component | Purpose | Shows For |
|-----------|---------|-----------|
| `<BeginnerHint>` | Extra explanations | Beginners only |
| `<AdvancedDeepDive>` | Advanced patterns | Advanced only |
| `<TermTooltip>` | Term definitions | All (behavior varies) |
| `<Hint>` | Collapsible hints | All (state varies) |
| `<PersonalizationToggle>` | On/off switch | Logged-in users |
| `<PersonalizationBadge>` | Profile indicator | Logged-in users |
| `<HardwareRecommendation>` | Hardware-specific highlights | Based on hardware_access |

### 6.2 CSS Classes

```css
/* Show only for beginners */
.beginner-only { display: none; }
[data-level="beginner"] .beginner-only { display: block; }

/* Show only for advanced */
.advanced-only { display: none; }
[data-level="advanced"] .advanced-only { display: block; }

/* Personalization disabled = intermediate default */
[data-personalization="disabled"] .beginner-only { display: none; }
[data-personalization="disabled"] .advanced-only { display: none; }

/* Hardware-specific highlighting (visual only) */
[data-hardware="jetson"] .jetson-example {
  border-left: 3px solid #76b900;
  background: rgba(118, 185, 0, 0.1);
}
[data-hardware="simulation"] .simulation-example {
  border-left: 3px solid #0066cc;
  background: rgba(0, 102, 204, 0.1);
}
```

---

## Section 7: Verification Criteria

### 7.1 How to Verify Presentation-Only

1. **Same RAG Response**
   - Log RAG requests for different users
   - Verify identical question produces identical chunks and answer

2. **Same Page Source**
   - HTML content (excluding personalization wrappers) identical for all users
   - Only CSS visibility changes

3. **Same Learning Path**
   - Course navigation identical
   - No locked/unlocked content based on level

### 7.2 Test Cases

| Test | Expected Result |
|------|-----------------|
| Beginner asks "What is ROS 2?" | Same RAG answer as advanced user |
| Advanced user sees same definitions | Yes, just less expanded hints |
| Beginner can access advanced content | Yes, nothing is locked |
| Personalization off shows default | Yes, intermediate experience |
| RAG logs show no user profile | Yes, profile never in logs |
| Different users, same question | Identical answer hash |

---

## Section 8: Constraints Summary

| What | Personalized? | Notes |
|------|--------------|-------|
| Hint expansion | YES | Default state varies |
| Tooltip behavior | YES | Auto-show for beginners |
| Section visibility | YES | BeginnerHint/AdvancedDeepDive |
| Example highlighting | YES | "Recommended" badge |
| Code comment visibility | YES | CSS-controlled display |
| Core content | NO | Always visible |
| Definitions | NO | Never changed |
| RAG answers | NO | Identical for all |
| Refusal messages | NO | Identical for all |
| Course structure | NO | Same navigation |
| Retrieval query | NO | Exactly user's question |
| LLM prompt | NO | Hardcoded, not user-dependent |
| Chunk selection | NO | All relevant chunks used |

---

## Section 9: Constitution Compliance

This specification enforces:

- **Principle III (RAG Safety)**: No external knowledge, no inference beyond retrieved text
- **Principle II (Curriculum Authority)**: Personalization does not alter curriculum
- **Principle V (Agent Boundaries)**: Identity agent does not modify RAG behavior

---

## Sign-off

This personalization rules specification is **LOCKED** and **NON-NEGOTIABLE**.

Any violation of RAG isolation is a **CRITICAL SAFETY FAILURE**.

Personalization is a presentation convenience, not a content modification system.

**Version**: 1.1.0 | **Updated**: 2024-12-25

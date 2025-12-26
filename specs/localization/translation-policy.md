# Translation Policy Specification

**Feature**: Urdu Translation for Physical AI Textbook
**Phase**: 6 - Urdu Translation Toggle (BONUS)
**Owner**: localization-manager agent
**Version**: 1.0.0
**Created**: 2024-12-25
**Status**: ACTIVE

---

## 1. Overview

This specification defines the policy for on-demand Urdu translation of textbook content. Translation is a **presentation-only** feature that does not alter canonical content, RAG behavior, or curriculum meaning.

### 1.1 Scope

**In Scope**:
- On-demand translation of visible chapter text
- User-controlled language toggle
- Preservation of technical terminology
- RTL (right-to-left) text rendering for Urdu

**Out of Scope**:
- Pre-translation of all content
- Persistent storage of translations
- Translation of RAG responses
- Translation of code blocks or commands
- Multiple language support (Urdu only for Phase 6)

### 1.2 Core Principle

> **CRITICAL NON-NEGOTIABLE PRINCIPLE**
>
> Translation is PRESENTATION ONLY. The canonical English content is the
> authoritative source. Translation MUST NOT alter meaning, remove content,
> or influence RAG/AI behavior in any way.

---

## 2. Translation Rules

### 2.1 On-Demand Translation Only

Translation occurs **only when requested** by the user:

```
User clicks toggle → Frontend sends text → Backend translates → Frontend displays
```

**Rules**:
1. Translation happens at runtime, not at build time
2. Original English content is ALWAYS preserved in memory
3. User can switch back to English instantly
4. No server-side caching of translations (stateless)

### 2.2 No Persistence of Translated Content

Translated text is **never persisted**:

| Storage Location | Stores Translation? | Reason |
|-----------------|---------------------|--------|
| Database | NO | Canonical content is English only |
| LocalStorage | NO | Avoid stale translations |
| SessionStorage | Optional (cache) | May cache for session performance |
| Server Cache | NO | Stateless API design |
| CDN | NO | Dynamic translation only |

**Rationale**: Persisting translations creates version control issues and potential for meaning drift over time.

### 2.3 Technical Term Preservation

The following terms MUST NOT be translated and should remain in English:

#### Robotics & ROS 2 Terms
- ROS 2, ROS 2 Humble, ROS 2 Iron
- node, topic, service, action, launch
- publisher, subscriber, client, server
- message, srv, action (file types)
- rclpy, rclcpp
- colcon, ament
- URDF, SDF, Xacro
- tf2, nav2, MoveIt 2

#### NVIDIA & Simulation Terms
- NVIDIA, CUDA, cuDNN, TensorRT
- Isaac Sim, Isaac ROS, Omniverse
- Gazebo, Gazebo Fortress
- Digital Twin
- Jetson, Jetson Orin, Jetson Nano

#### AI/ML Terms
- VLA (Vision-Language-Action)
- LLM, transformer, embedding
- inference, model, weights
- OpenAI, Claude, GPT
- PyTorch, TensorFlow

#### Programming Terms
- Python, C++, CMake
- pip, conda, Docker
- git, GitHub
- API, REST, JSON
- function, class, method (when referring to code)

#### Hardware Terms
- GPU, CPU, RAM
- sensor, actuator, motor
- lidar, camera, IMU
- RGB, depth, point cloud

**Implementation**: These terms should be wrapped in `<span class="preserve-term">` tags and excluded from translation.

### 2.4 Fallback to English

If translation fails or is ambiguous, **fallback to English**:

| Scenario | Behavior |
|----------|----------|
| API error | Show original English + error toast |
| Timeout | Show original English + retry option |
| Ambiguous term | Keep in English |
| Empty response | Show original English |
| Partial translation | Show original English (no partial) |

**User Message on Fallback**:
```
"Translation unavailable. Showing original English content."
```

---

## 3. Content Classification

### 3.1 Translatable Content

| Content Type | Translate? | Notes |
|--------------|------------|-------|
| Paragraph text | YES | Main educational content |
| Headings | YES | Section titles |
| Lists (ul/ol) | YES | Bullet points |
| Captions | YES | Figure/table captions |
| Tooltips | YES | Help text |
| UI labels | YES | Button text, navigation |
| Error messages | YES | User-facing errors |

### 3.2 Non-Translatable Content

| Content Type | Translate? | Notes |
|--------------|------------|-------|
| Code blocks | NO | Preserve exactly |
| Code comments | NO | Part of code |
| Terminal commands | NO | Must be exact |
| File paths | NO | System paths |
| URLs | NO | Links must work |
| Variable names | NO | Code identifiers |
| Technical terms | NO | Per preservation list |
| Mathematical formulas | NO | LaTeX/MathJax |
| API responses | NO | JSON/data |

### 3.3 Special Handling

**Inline Code** (`backtick` text):
- Keep in English
- Wrap in non-translated span

**Mixed Content** (English term in Urdu sentence):
- Use `unicode-bidi: embed` for proper rendering
- Term appears LTR within RTL context

---

## 4. Translation Quality Requirements

### 4.1 Accuracy Standards

| Requirement | Standard |
|-------------|----------|
| Factual accuracy | 100% (no false information) |
| Technical accuracy | 100% (correct concepts) |
| Semantic preservation | 100% (same meaning) |
| Fluency | High (natural Urdu) |
| Consistency | Terms translated same way throughout |

### 4.2 Prohibited Translations

The following MUST NEVER occur:

1. **Meaning Change**: "ROS 2 is required" → "ROS 2 is optional" ❌
2. **Content Removal**: Skipping paragraphs or sections ❌
3. **Content Addition**: Adding opinions or information ❌
4. **Instruction Change**: "Run this command" → different command ❌
5. **Safety Compromise**: Removing warnings or cautions ❌

### 4.3 Quality Validation

Before deploying translation:
1. Review sample translations with Urdu speaker
2. Verify technical terms preserved
3. Check RTL rendering in UI
4. Validate no meaning drift

---

## 5. RAG Isolation

### 5.1 Translation-RAG Boundary

```
+------------------+     +------------------+     +------------------+
|   User Question  | --> |   RAG Pipeline   | --> |  English Answer  |
+------------------+     +------------------+     +------------------+
                                                         |
                         [RAG BOUNDARY - NO CROSSING]    |
                                                         v
                         +------------------+     +------------------+
                         |  Translation API | <-- |  Display Layer   |
                         +------------------+     +------------------+
```

### 5.2 Isolation Rules

1. **RAG Never Sees Translation**: User questions go to RAG in original form
2. **Translation Never Sees RAG**: Translation API has no RAG imports
3. **Answers Not Translated**: RAG answers displayed in English only
4. **Same Question = Same Answer**: Regardless of UI language

### 5.3 Code Verification

```python
# Translation service MUST NOT import:
# from app.services.rag import ...       ❌
# from app.services.generation import ...  ❌
# from app.db.qdrant import ...           ❌

# Translation service MAY import:
# from app.core.config import ...         ✓
# from app.core.logging import ...        ✓
# from openai import OpenAI               ✓ (for translation only)
```

---

## 6. User Control

### 6.1 Toggle Behavior

| Action | Result |
|--------|--------|
| Toggle ON (to Urdu) | Translate visible content |
| Toggle OFF (to English) | Restore original instantly |
| Navigate to new page | Remember preference |
| Refresh page | Remember preference |
| Log out | Reset to English (default) |

### 6.2 Preference Storage

- Store language preference in `localStorage`
- Key: `textbook_language`
- Values: `en` (default) | `ur`
- Do NOT store in user profile (backend)

### 6.3 Default State

- New visitors: English
- Logged-in users: English (unless localStorage says otherwise)
- Anonymous users: English

---

## 7. Performance Requirements

| Metric | Target |
|--------|--------|
| Translation latency | < 3 seconds for average chapter |
| Toggle response | < 100ms (show loading state) |
| Memory overhead | < 2x (store original + translated) |
| API timeout | 30 seconds max |

### 7.1 Optimization Strategies

1. **Chunked Translation**: Translate visible viewport first
2. **Lazy Loading**: Translate below-fold content on scroll
3. **Session Cache**: Cache translations in sessionStorage
4. **Debounce**: Don't re-translate on rapid toggles

---

## 8. Error Handling

### 8.1 Error States

| Error | User Message | Action |
|-------|--------------|--------|
| API unreachable | "Translation service unavailable" | Show English |
| Rate limited | "Too many requests. Please wait." | Retry after delay |
| Invalid response | "Translation failed" | Show English |
| Timeout | "Translation timed out" | Show English + retry |

### 8.2 Graceful Degradation

If translation fails:
1. Log error for monitoring
2. Show user-friendly message
3. Display original English content
4. Offer retry option
5. Never show partial/corrupted translation

---

## 9. Acceptance Criteria

### 9.1 Policy Compliance

- [ ] Translation is on-demand only (not pre-built)
- [ ] No translated content persisted to database
- [ ] Technical terms preserved per Section 2.3
- [ ] Fallback to English on any error
- [ ] Code blocks never translated
- [ ] RAG pipeline completely isolated

### 9.2 Quality Criteria

- [ ] Meaning preserved in all translations
- [ ] No content added or removed
- [ ] RTL rendering works correctly
- [ ] Toggle is instant and reversible
- [ ] Performance targets met

---

## 10. Related Specifications

- [Translation UX Specification](/specs/localization/translation-ui.md)
- [Personalization Rules](/specs/auth/personalization-rules.md) - Similar isolation pattern
- [RAG Retrieval Policy](/specs/rag/retrieval-policy.md) - Must not be affected

---

**Signed**: localization-manager agent
**Date**: 2024-12-25

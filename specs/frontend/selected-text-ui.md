# Selected-Text Interaction Specification

**Feature**: Text Selection Capture for Contextual Q&A
**Phase**: 4 - Frontend Platform & Embedded Chatbot
**Owner**: frontend-platform agent
**Created**: 2024-12-24

## Overview

Users can select text within chapter content to provide focused context for their questions. The selected text defines the ONLY scope for answering - no silent context augmentation is permitted.

## Text Selection Capture Rules

### Selection Scope
- **Allowed Areas**: Chapter content within `<article>` or `.markdown` containers
- **Disallowed Areas**: Navigation, headers, footers, chatbot panel, code blocks
- **Minimum Length**: 50 characters (shorter selections are ignored)
- **Maximum Length**: 5000 characters (truncated with warning)

### Selection Events
- Capture on `mouseup` event (desktop)
- Capture on `touchend` event (mobile)
- Debounce: 200ms to avoid partial selections

### Selection Validation
```typescript
function isValidSelection(selection: Selection): boolean {
  // Must have text
  if (!selection.toString().trim()) return false;

  // Must be in allowed container
  const range = selection.getRangeAt(0);
  const container = range.commonAncestorContainer;
  if (!isInChapterContent(container)) return false;

  // Must meet minimum length
  if (selection.toString().length < 50) return false;

  return true;
}
```

## Visual Scope Indicator

### Selection Highlight
When valid text is selected and chatbot is open:

1. **Highlight Style**:
   - Background: Light blue (#E3F2FD)
   - Border: 2px dashed blue (#2196F3)
   - Animation: Subtle pulse on first selection

2. **Selection Badge**:
   - Floating badge appears near selection
   - Shows: "Selected for Q&A (X chars)"
   - Includes: Clear selection button (×)

### Scope Indicator in Chat
When text is selected, show in chat input area:

```
┌──────────────────────────────────────┐
│ 📝 Using selected text (234 chars)   │
│ "Physical AI systems must account... │
│ [View Full] [Clear Selection]        │
├──────────────────────────────────────┤
│ Ask about the selected text...       │
│                              [Send]  │
└──────────────────────────────────────┘
```

### Clear Selection Behavior
- Click "Clear Selection" removes selection
- Navigate away from page clears selection
- Selecting new text replaces old selection
- Pressing Escape clears selection

## User Confirmation of Limited Context

### Confirmation UI
Before sending a question with selected text:

```
┌────────────────────────────────────────────┐
│ 🔒 Context-Limited Query                   │
│                                            │
│ Your question will ONLY be answered using  │
│ the text you selected:                     │
│                                            │
│ "Physical AI systems must account for      │
│ real-world constraints including force     │
│ limits, sensor noise, and actuator..."     │
│ [Show more]                                │
│                                            │
│ The chatbot will NOT search the full       │
│ textbook for this question.                │
│                                            │
│ [Ask About Selection] [Search Full Book]  │
└────────────────────────────────────────────┘
```

### Confirmation Rules
- Show confirmation when:
  - User has selected text AND
  - User submits a question
- Skip confirmation if:
  - User previously checked "Don't show again this session"
  - Selection was just cleared

### "Don't Show Again" Preference
- Checkbox: "Don't ask me again this session"
- Stored in sessionStorage
- Resets on new session

## Selection in Chat History

### Visual Indicator
Messages sent with selected text show:

```
You (with selection):
"What are the force limits?"

📌 Context: "Physical AI systems must account
for real-world constraints including force
limits, sensor noise, and..." [234 chars]
```

### Selection Metadata
Store with message:
```typescript
interface MessageWithSelection {
  // ... base message fields
  selectedText?: {
    text: string;
    length: number;
    truncated: boolean;
  };
}
```

## Mobile Considerations

### Touch Selection
- Native text selection via long-press
- Selection handles visible
- "Use for Q&A" floating action appears after selection

### Selection Toolbar
On mobile after selection:
```
┌──────────────────────────┐
│ Copy │ Search │ Ask AI 💬│
└──────────────────────────┘
```

### Viewport Adjustment
- When chatbot opens with selection, scroll to keep selection visible
- Keyboard should not overlap selection preview

## Constraints

### CRITICAL: No Silent Context Augmentation
- **NEVER** add surrounding paragraphs
- **NEVER** include chapter context not selected
- **NEVER** expand selection programmatically
- Selected text is the ONLY context sent to backend

### Transparency Rules
- User MUST see exactly what text is being used
- User MUST confirm before sending (unless opted out)
- User MUST be able to clear selection easily
- Visual distinction between selected-text queries and full-book queries

### Backend Contract
When selected text is present, request must include:
```json
{
  "question": "user's question",
  "selected_text": "exact selected text, not augmented",
  "session_id": "uuid"
}
```

## Component Structure

```
TextSelectionProvider/
├── SelectionHighlight (visual overlay)
├── SelectionBadge (floating indicator)
├── useTextSelection (hook)
│   ├── selection state
│   ├── validation logic
│   └── clear function
└── SelectionContext (React context)

ChatInput/
├── SelectionPreview
│   ├── text preview
│   ├── char count
│   └── clear button
├── ConfirmationModal
│   ├── context display
│   ├── action buttons
│   └── remember preference checkbox
└── InputField
```

## State Management

```typescript
interface SelectionState {
  selectedText: string | null;
  selectionRange: Range | null;
  isValid: boolean;
  charCount: number;
  isTruncated: boolean;
}

interface SelectionPreferences {
  skipConfirmation: boolean; // sessionStorage
}
```

## Acceptance Criteria

1. ✓ Text selection captured only from chapter content
2. ✓ Selection visually highlighted with clear scope
3. ✓ User sees exactly what text will be used
4. ✓ Confirmation shown before sending selected-text query
5. ✓ No hidden context expansion
6. ✓ Clear selection mechanism available
7. ✓ Mobile-friendly selection handling
8. ✓ Selection state visible in message history

# Chatbot UI Specification

**Feature**: Embedded RAG Chatbot for Physical AI Textbook
**Phase**: 4 - Frontend Platform & Embedded Chatbot
**Owner**: frontend-platform agent
**Created**: 2024-12-24

## Overview

An embedded chatbot component for the Docusaurus textbook that provides grounded Q&A based on textbook content. The chatbot is transparent about its limitations and clearly communicates when it cannot answer.

## Placement and Toggle Behavior

### Chatbot Container
- **Position**: Fixed bottom-right corner of viewport
- **Collapsed State**: Floating action button (FAB) with chat icon
- **Expanded State**: Chat panel (400px width × 500px height on desktop)
- **Z-Index**: Above all content but below modals (z-index: 1000)

### Toggle Behavior
- Click FAB to expand chat panel
- Click close button (×) or FAB again to collapse
- ESC key closes the panel
- Panel persists across page navigation within session
- Chat history maintained during session

### Responsive Behavior
- **Desktop (>768px)**: Side panel, 400×500px
- **Mobile (<768px)**: Full-width bottom sheet, 100% width × 70% height
- Keyboard adjustments on mobile for text input

## Message Flow and States

### Message Types
1. **User Message**: Right-aligned, distinct background
2. **Assistant Message**: Left-aligned, different background
3. **Refusal Message**: Left-aligned with warning styling
4. **Loading State**: Typing indicator animation

### Message States

#### Initial State
```
Welcome message displayed:
"Ask me anything about the Physical AI & Humanoid Robotics textbook.
I can only answer from the course content."
```

#### Loading State
- Disable send button
- Show animated typing indicator (three dots pulsing)
- User message visible above loading indicator

#### Answer State
- Display grounded answer
- Show source references (collapsible section)
- Sources display: Module > Chapter > Section

#### Refusal State
- Distinct visual treatment (amber/yellow background)
- Clear explanation of why answer is unavailable
- Suggestion for how to proceed

### Input Area
- Text input field (multi-line, max 1000 chars)
- Send button (disabled when empty or loading)
- Character count indicator when > 800 chars
- Placeholder: "Ask about the textbook content..."

## Source Transparency Indicators

### Source Reference Display
When the chatbot provides an answer, sources MUST be visible:

```
[Answer text here...]

📚 Sources (3):
├─ Module 1: Physical AI Foundations
│  └─ Chapter 2: Sensors & Actuators § Sensor Types
├─ Module 1: Physical AI Foundations
│  └─ Chapter 3: Physical Constraints § Force Limits
└─ Module 2: ROS 2 Fundamentals
   └─ Chapter 1: Nodes & Topics § Message Types
```

### Source Visibility Rules
- Always show source count: "Sources (N)"
- Default: Sources collapsed (click to expand)
- Each source shows: Module → Chapter → Section
- Confidence score shown as filled circles (●●●○○ for 0.6)
- Link to navigate to source section (if applicable)

## Refusal Display Rules

### Refusal Visual Treatment
- Background: Amber/yellow tint (not red - not an error)
- Icon: Information icon (ℹ️) not error icon
- Border: Left accent border in amber

### Refusal Message Types

#### Empty Retrieval
```
ℹ️ Topic Not Covered

I can only answer questions from the textbook content.
This topic doesn't appear to be covered in the course materials.

💡 Try asking about: Physical AI, ROS 2, Simulation, Isaac, or VLA systems.
```

#### Insufficient Context
```
ℹ️ Need More Context

I found some related content, but not enough to provide a confident answer.

💡 Try:
• Rephrasing your question more specifically
• Selecting text from the chapter to narrow the scope
```

#### Out of Scope
```
ℹ️ Outside Course Scope

This topic is outside the scope of this course. The textbook covers:
• Physical AI Foundations
• ROS 2 Fundamentals
• Simulation & Digital Twin
• NVIDIA Isaac Ecosystem
• Vision-Language-Action Systems

For [detected topic], please refer to specialized resources.
```

#### Selected Text Insufficient
```
ℹ️ Selection Too Limited

The selected text doesn't contain enough information to answer your question.

💡 Try:
• Selecting a different section
• Asking without text selection to search the full textbook
```

## Error Handling

### Network Errors
```
⚠️ Connection Error

Unable to reach the server. Please check your connection and try again.

[Retry Button]
```

### Server Errors (5xx)
```
⚠️ Service Temporarily Unavailable

The chatbot service is experiencing issues. Please try again in a moment.

[Retry Button]
```

### Validation Errors (4xx)
- Display specific validation message from API
- Highlight problematic input field

## Accessibility Requirements

- ARIA labels for all interactive elements
- Focus management: trap focus in open panel
- Keyboard navigation: Tab through elements, Enter to send
- Screen reader announcements for new messages
- High contrast mode support
- Minimum touch target size: 44×44px

## Component Structure

```
ChatbotRoot/
├── ChatbotFAB (toggle button)
├── ChatbotPanel/
│   ├── ChatHeader (title, close button)
│   ├── ChatMessages/
│   │   ├── WelcomeMessage
│   │   ├── MessageList/
│   │   │   ├── UserMessage
│   │   │   ├── AssistantMessage
│   │   │   │   └── SourceReferences
│   │   │   └── RefusalMessage
│   │   └── LoadingIndicator
│   └── ChatInput/
│       ├── TextInput
│       ├── CharCounter
│       └── SendButton
└── ErrorBoundary
```

## State Management

### Chat State
```typescript
interface ChatState {
  isOpen: boolean;
  messages: Message[];
  isLoading: boolean;
  sessionId: string | null;
  error: string | null;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: SourceReference[];
  isRefusal?: boolean;
  refusalReason?: string;
}
```

### Persistence
- Session ID stored in sessionStorage
- Chat open/closed state persists during session
- Message history cleared on session end

## Constraints

1. **No Direct LLM Calls**: Frontend ONLY communicates with FastAPI backend
2. **No Context Expansion**: Cannot add context beyond user input
3. **Visible Limitations**: Must clearly show AI boundaries
4. **Source Required**: No answer without source references (except refusals)

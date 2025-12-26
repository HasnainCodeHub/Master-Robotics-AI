# Translation UX Specification

**Feature**: Urdu Translation UI for Physical AI Textbook
**Phase**: 6 - Urdu Translation Toggle (BONUS)
**Owner**: frontend-platform agent
**Version**: 1.0.0
**Created**: 2024-12-25
**Status**: ACTIVE

---

## 1. Overview

This specification defines the user experience for the Urdu translation feature, including toggle placement, visual indicators, loading states, and RTL layout considerations.

### 1.1 Design Principles

1. **Clarity**: User always knows current language state
2. **Reversibility**: Toggle back to English is instant
3. **Non-intrusive**: Translation UI doesn't disrupt reading
4. **Accessible**: Works with keyboard and screen readers
5. **Mobile-first**: Responsive on all devices

---

## 2. Toggle Placement

### 2.1 Location

The translation toggle appears in **two locations**:

1. **Chapter Header**: Sticky toggle near chapter title
2. **Floating Action Button (FAB)**: Bottom-left corner (opposite to chatbot FAB)

```
+----------------------------------------------------------+
|  [Logo]  Home | Modules | About           [EN|اُردُو] [User] |  <- Navbar
+----------------------------------------------------------+
|                                                          |
|  Chapter 1: Introduction to Physical AI    [EN|اُردُو]   |  <- Chapter header toggle
|  =====================================================   |
|                                                          |
|  Content area...                                         |
|                                                          |
|                                                          |
|  [🌐]                                            [💬]    |  <- FAB toggles
+----------------------------------------------------------+
```

### 2.2 Toggle Design

**Segmented Control Style**:
```
+--------+----------+
|   EN   |  اُردُو  |
+--------+----------+
   ↑ Active = highlighted background
```

**Dimensions**:
- Width: Auto (fits content)
- Height: 32px
- Border radius: 6px
- Font size: 14px

**States**:
| State | EN Button | Urdu Button |
|-------|-----------|-------------|
| English Active | Primary bg, white text | Transparent bg, muted text |
| Urdu Active | Transparent bg, muted text | Primary bg, white text |
| Loading | Disabled, spinner | Disabled, spinner |
| Error | Normal | Normal + error indicator |

### 2.3 FAB Design

**Position**: Bottom-left, 20px from edge
**Size**: 48px diameter
**Icon**: Globe icon (🌐)
**Tooltip**: "Switch to Urdu" / "Switch to English"

**Mobile**: FAB is primary toggle (header toggle hidden on mobile)

---

## 3. Language State Indicators

### 3.1 Visual Indicators

**Badge in Toggle**:
- EN = "EN" or "English"
- UR = "اُردُو" (Urdu in Urdu script)

**Document-level Indicator**:
- `<html lang="en">` or `<html lang="ur" dir="rtl">`
- Visible badge in page header

**Content Indicator**:
- Subtle label: "Translated from English" at top of translated content

### 3.2 State Persistence

```typescript
// localStorage key
const LANGUAGE_KEY = 'textbook_language';

// Values
type Language = 'en' | 'ur';

// Default
const DEFAULT_LANGUAGE: Language = 'en';

// On page load
const savedLang = localStorage.getItem(LANGUAGE_KEY) || DEFAULT_LANGUAGE;
```

### 3.3 State Transitions

```
                    +------------------+
                    |  Page Load       |
                    +------------------+
                            |
                            v
                    +------------------+
                    | Check localStorage|
                    +------------------+
                            |
              +-------------+-------------+
              |                           |
              v                           v
     +-----------------+         +-----------------+
     | lang = 'en'     |         | lang = 'ur'     |
     | Show English    |         | Trigger translate|
     +-----------------+         +-----------------+
              |                           |
              |    [User clicks toggle]   |
              +<------------------------->+
```

---

## 4. Loading States

### 4.1 Initial Translation Load

When user first toggles to Urdu:

```
+----------------------------------------------------------+
|  Chapter 1: Introduction...              [EN|⏳ اُردُو]   |
+----------------------------------------------------------+
|                                                          |
|  ████████████████████████████                            |
|  ████████████████████                                    |
|  ████████████████████████████████████                    |
|                                                          |
|  [Skeleton loading animation for content]                |
|                                                          |
+----------------------------------------------------------+
```

**Loading Indicators**:
1. Toggle button shows spinner
2. Content area shows skeleton loader
3. "Translating..." label below header

**Duration Expectation**: < 3 seconds

### 4.2 Subsequent Loads

If translation is cached in sessionStorage:
- Instant switch (< 100ms)
- No loading indicator needed

### 4.3 Long Translation

For chapters > 5000 words:
1. Show progress indicator: "Translating... 50%"
2. Progressive rendering: Show translated sections as they arrive
3. Timeout warning at 20s: "Translation taking longer than expected..."

---

## 5. Error States

### 5.1 Error Types and Messages

| Error | Toast Message | UI Behavior |
|-------|---------------|-------------|
| Network error | "Unable to connect. Showing English." | Revert to English |
| API error | "Translation service error. Try again later." | Revert to English |
| Timeout | "Translation timed out. Showing English." | Revert to English |
| Rate limit | "Too many requests. Please wait a moment." | Keep current state |

### 5.2 Error UI

```
+----------------------------------------------------------+
|  ⚠️ Translation unavailable                    [Dismiss]  |
|  Showing original English content. [Retry]               |
+----------------------------------------------------------+
```

**Toast Position**: Top-center, below navbar
**Auto-dismiss**: 5 seconds (with manual dismiss option)
**Retry Button**: Visible for transient errors

### 5.3 Graceful Degradation

If translation fails:
1. Show error toast
2. Keep original English content visible
3. Reset toggle to "EN" state
4. Allow retry

---

## 6. RTL Layout

### 6.1 RTL Requirements

When Urdu is active:

```css
/* Document-level */
html[lang="ur"] {
  direction: rtl;
}

/* Content area */
.translated-content {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
}

/* Preserve LTR for code and terms */
.preserve-term,
code,
pre {
  direction: ltr;
  unicode-bidi: embed;
}
```

### 6.2 Mixed Content

When English terms appear in Urdu text:

```html
<p dir="rtl">
  یہ ایک <span class="preserve-term" dir="ltr">ROS 2 node</span> ہے۔
</p>
```

**Rendering**: "یہ ایک ROS 2 node ہے۔" with ROS 2 node in LTR

### 6.3 Layout Mirroring

| Element | English | Urdu |
|---------|---------|------|
| Text alignment | Left | Right |
| List markers | Left | Right |
| Navigation | Left-to-right | Right-to-left |
| Icons with text | Icon left | Icon right |
| Code blocks | LTR (unchanged) | LTR (unchanged) |

### 6.4 Components NOT Mirrored

- Code blocks (always LTR)
- Mathematical formulas
- Diagrams and images
- Navigation URLs
- Chatbot (separate component)

---

## 7. Component Specifications

### 7.1 TranslationToggle Component

```typescript
interface TranslationToggleProps {
  /** Current language */
  language: 'en' | 'ur';
  /** Loading state */
  isLoading: boolean;
  /** Error state */
  error: string | null;
  /** Toggle handler */
  onToggle: () => void;
  /** Variant: header or fab */
  variant: 'header' | 'fab';
}
```

**Behavior**:
- Click toggles between EN and UR
- Disabled during loading
- Shows error indicator if error

### 7.2 TranslationContext

```typescript
interface TranslationContextValue {
  /** Current language */
  language: 'en' | 'ur';
  /** Is translation in progress */
  isTranslating: boolean;
  /** Translation error if any */
  error: string | null;
  /** Original English content (always available) */
  originalContent: string | null;
  /** Translated content (null if not translated) */
  translatedContent: string | null;
  /** Toggle language */
  toggleLanguage: () => Promise<void>;
  /** Reset to English */
  resetToEnglish: () => void;
  /** Clear error */
  clearError: () => void;
}
```

### 7.3 LanguageIndicator Component

```typescript
interface LanguageIndicatorProps {
  /** Current language */
  language: 'en' | 'ur';
  /** Show full name or abbreviation */
  showFullName?: boolean;
}
```

**Output**:
- EN: "English" or "EN"
- UR: "اُردُو" or "UR"

---

## 8. Accessibility

### 8.1 ARIA Attributes

```html
<div role="group" aria-label="Language selection">
  <button
    role="radio"
    aria-checked="true"
    aria-label="English"
  >
    EN
  </button>
  <button
    role="radio"
    aria-checked="false"
    aria-label="Urdu"
  >
    اُردُو
  </button>
</div>
```

### 8.2 Keyboard Navigation

| Key | Action |
|-----|--------|
| Tab | Focus toggle |
| Enter/Space | Activate toggle |
| Arrow Left/Right | Switch language (when focused) |
| Escape | Close FAB menu (if open) |

### 8.3 Screen Reader Announcements

| Event | Announcement |
|-------|--------------|
| Toggle to Urdu | "Translating to Urdu" |
| Translation complete | "Content now in Urdu" |
| Toggle to English | "Switched to English" |
| Error | "Translation failed. Showing English." |

### 8.4 Focus Management

- After translation completes, focus returns to toggle
- Content area does not steal focus
- Error toast is announced but doesn't steal focus

---

## 9. Responsive Design

### 9.1 Breakpoints

| Breakpoint | Toggle Location | FAB Visible |
|------------|-----------------|-------------|
| Desktop (> 996px) | Header + FAB | Yes |
| Tablet (768-996px) | Header only | Optional |
| Mobile (< 768px) | FAB only | Yes |

### 9.2 Mobile Considerations

- FAB is primary toggle on mobile
- Header toggle hidden (navbar space limited)
- Touch target: minimum 44x44px
- Swipe gesture: Not implemented (avoid conflicts)

### 9.3 Mobile Layout

```
+---------------------------+
|  [☰]  Physical AI  [👤]   |  <- Compact navbar
+---------------------------+
|                           |
|  Chapter 1                |
|  Introduction...          |
|                           |
|  [Content]                |
|                           |
|                           |
|  [🌐]              [💬]   |  <- FAB row
+---------------------------+
```

---

## 10. Integration Points

### 10.1 Provider Hierarchy

```tsx
<AuthProvider>
  <PersonalizationProvider>
    <TranslationProvider>      {/* NEW */}
      <ChatbotProvider>
        {children}
        <Chatbot />
        <TranslationFAB />     {/* NEW */}
      </ChatbotProvider>
    </TranslationProvider>
  </PersonalizationProvider>
</AuthProvider>
```

### 10.2 Content Detection

Translation targets content within:
- `<article>` elements
- `.markdown` class
- `[data-translatable="true"]` attribute

Translation excludes:
- `<code>`, `<pre>` elements
- `.preserve-term` class
- `[data-translatable="false"]` attribute

### 10.3 Event Flow

```
1. User clicks toggle
2. TranslationContext.toggleLanguage() called
3. If switching to Urdu:
   a. Set isTranslating = true
   b. Extract translatable content from DOM
   c. POST to /api/translate
   d. Receive translated text
   e. Store original in state
   f. Replace DOM content with translation
   g. Set language = 'ur', isTranslating = false
4. If switching to English:
   a. Restore original content from state
   b. Set language = 'en'
```

---

## 11. Acceptance Criteria

### 11.1 Functional Requirements

- [ ] Toggle visible in chapter header (desktop) and FAB (all devices)
- [ ] Click toggles between EN and UR
- [ ] Loading state shown during translation
- [ ] Error toast on failure with retry option
- [ ] Original English always restorable instantly
- [ ] Language preference persisted in localStorage

### 11.2 Visual Requirements

- [ ] Clear visual distinction between EN and UR states
- [ ] RTL layout applied for Urdu content
- [ ] Code blocks remain LTR
- [ ] Technical terms remain in English
- [ ] Responsive on mobile devices

### 11.3 Accessibility Requirements

- [ ] Keyboard navigable
- [ ] Screen reader compatible
- [ ] ARIA attributes present
- [ ] Focus management correct

---

## 12. Wireframes

### 12.1 Desktop Header Toggle

```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│   Chapter 1: Introduction to Physical AI                   │
│   ═══════════════════════════════════════                  │
│                                                            │
│   ┌─────────────────┐                                      │
│   │  EN  │  اُردُو  │  ← Segmented toggle                  │
│   └─────────────────┘                                      │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### 12.2 FAB Toggle

```
┌──────────┐
│    🌐    │  ← Globe icon
│          │
│   EN     │  ← Current language badge
└──────────┘

[Hover/Focus State]
┌──────────────────────┐
│  Switch to Urdu      │  ← Tooltip
└──────────────────────┘
```

### 12.3 Loading State

```
┌────────────────────────────────────────────────────────────┐
│   Chapter 1: Introduction...        [EN │ ⏳ Translating ] │
├────────────────────────────────────────────────────────────┤
│                                                            │
│   ████████████████████████████████████████                 │
│   ████████████████████████████                             │
│   ████████████████████████████████████████████             │
│                                                            │
│   [Skeleton loader pulses...]                              │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

---

## 13. Related Specifications

- [Translation Policy](/specs/localization/translation-policy.md)
- [Chatbot UI](/specs/frontend/chatbot-ui.md) - Similar FAB pattern
- [Personalization Rules](/specs/auth/personalization-rules.md) - Similar toggle pattern

---

**Signed**: frontend-platform agent
**Date**: 2024-12-25

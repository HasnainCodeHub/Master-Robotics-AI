/**
 * Chat Input Component
 * Handles user input, selection preview, and message submission.
 */

import React, { useState, useCallback, useRef, useEffect } from 'react';
import { useChatbot } from './ChatbotContext';
import type { ChatMode } from './types';
import styles from './styles.module.css';

interface ChatInputProps {
  onSubmit: (question: string, selectedText: string | null, mode?: ChatMode) => Promise<void>;
  disabled: boolean;
}

const MAX_QUESTION_LENGTH = 1000;
const WARN_THRESHOLD = 800;

export function ChatInput({ onSubmit, disabled }: ChatInputProps) {
  const [question, setQuestion] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const { selectionState, clearSelection } = useChatbot();

  const charCount = question.length;
  const isOverLimit = charCount > MAX_QUESTION_LENGTH;
  const showWarning = charCount > WARN_THRESHOLD;
  const canSubmit = question.trim().length > 0 && !isOverLimit && !disabled;

  // Auto-resize textarea
  useEffect(() => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = `${Math.min(textarea.scrollHeight, 100)}px`;
    }
  }, [question]);

  const handleSubmit = useCallback(async () => {
    if (!canSubmit) return;

    const selectedText = selectionState.isValid ? selectionState.selectedText : null;
    const trimmedQuestion = question.trim();

    setQuestion('');
    clearSelection();

    await onSubmit(trimmedQuestion, selectedText);
  }, [canSubmit, question, selectionState, clearSelection, onSubmit]);

  const handleKeyDown = useCallback((e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  }, [handleSubmit]);

  return (
    <div className={styles.inputArea}>
      {/* Selection Preview */}
      {selectionState.isValid && selectionState.selectedText && (
        <div className={styles.selectionPreview}>
          <div className={styles.selectionHeader}>
            <span className={styles.selectionLabel}>
              📝 Using selected text ({selectionState.charCount} chars)
              {selectionState.isTruncated && ' - truncated'}
            </span>
            <button
              className={styles.clearSelection}
              onClick={clearSelection}
              aria-label="Clear selection"
            >
              Clear
            </button>
          </div>
          <div className={styles.selectionText}>
            "{selectionState.selectedText.slice(0, 100)}..."
          </div>
        </div>
      )}

      {/* Input Row */}
      <div className={styles.inputRow}>
        <textarea
          ref={textareaRef}
          className={styles.textInput}
          value={question}
          onChange={(e) => setQuestion(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={
            selectionState.isValid
              ? 'Ask about the selected text...'
              : 'Ask about the textbook content...'
          }
          disabled={disabled}
          aria-label="Type your question"
          rows={1}
        />
        <button
          className={styles.sendButton}
          onClick={handleSubmit}
          disabled={!canSubmit}
          aria-label="Send message"
        >
          <svg
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <line x1="22" y1="2" x2="11" y2="13"></line>
            <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
          </svg>
        </button>
      </div>

      {/* Character Count */}
      {showWarning && (
        <div
          className={`${styles.charCount} ${isOverLimit ? styles.charCountWarning : ''}`}
        >
          {charCount}/{MAX_QUESTION_LENGTH}
        </div>
      )}
    </div>
  );
}

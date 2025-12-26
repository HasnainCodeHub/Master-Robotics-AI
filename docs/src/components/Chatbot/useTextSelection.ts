/**
 * Text Selection Hook
 * Captures text selection from chapter content for contextual Q&A.
 *
 * CRITICAL RULE: No silent context augmentation.
 * The selected text is the ONLY context sent to the backend.
 */

import { useEffect, useCallback } from 'react';
import { useChatbot } from './ChatbotContext';

const MIN_SELECTION_LENGTH = 50;
const DEBOUNCE_MS = 200;

/**
 * Check if the selection is within allowed chapter content.
 */
function isInChapterContent(node: Node): boolean {
  let current: Node | null = node;

  while (current) {
    if (current instanceof HTMLElement) {
      // Check for chapter content containers
      if (
        current.tagName === 'ARTICLE' ||
        current.classList.contains('markdown') ||
        current.classList.contains('theme-doc-markdown')
      ) {
        return true;
      }

      // Exclude chatbot panel, navigation, footer
      if (
        current.classList.contains('chatPanel') ||
        current.tagName === 'NAV' ||
        current.tagName === 'FOOTER' ||
        current.tagName === 'HEADER' ||
        current.tagName === 'CODE' ||
        current.tagName === 'PRE'
      ) {
        return false;
      }
    }

    current = current.parentNode;
  }

  return false;
}

/**
 * Validate a text selection.
 */
function isValidSelection(selection: Selection | null): boolean {
  if (!selection || selection.isCollapsed) {
    return false;
  }

  const text = selection.toString().trim();

  // Check minimum length
  if (text.length < MIN_SELECTION_LENGTH) {
    return false;
  }

  // Check if in allowed container
  try {
    const range = selection.getRangeAt(0);
    const container = range.commonAncestorContainer;
    if (!isInChapterContent(container)) {
      return false;
    }
  } catch {
    return false;
  }

  return true;
}

/**
 * Hook to capture text selection from chapter content.
 */
export function useTextSelection() {
  const { state, setSelection, clearSelection } = useChatbot();

  const handleSelectionChange = useCallback(() => {
    // Only capture when chatbot is open
    if (!state.isOpen) {
      return;
    }

    const selection = window.getSelection();

    if (isValidSelection(selection)) {
      const text = selection!.toString().trim();
      setSelection(text);
    }
  }, [state.isOpen, setSelection]);

  // Debounced selection handler
  useEffect(() => {
    if (typeof window === 'undefined') return;

    let timeoutId: ReturnType<typeof setTimeout>;

    const debouncedHandler = () => {
      clearTimeout(timeoutId);
      timeoutId = setTimeout(handleSelectionChange, DEBOUNCE_MS);
    };

    // Listen for mouseup (desktop) and touchend (mobile)
    document.addEventListener('mouseup', debouncedHandler);
    document.addEventListener('touchend', debouncedHandler);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('mouseup', debouncedHandler);
      document.removeEventListener('touchend', debouncedHandler);
    };
  }, [handleSelectionChange]);

  // Clear selection on page navigation
  useEffect(() => {
    return () => {
      clearSelection();
    };
  }, [clearSelection]);

  // Clear selection on Escape
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        clearSelection();
        window.getSelection()?.removeAllRanges();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [clearSelection]);
}

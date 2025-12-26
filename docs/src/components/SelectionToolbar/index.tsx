/**
 * SelectionToolbar Component
 *
 * Floating contextual toolbar that appears when text is selected in textbook content.
 * Provides quick actions: "Ask Tutor" and "Translate to Urdu".
 *
 * RULES:
 * - Only appears for selections >= 10 characters
 * - Only appears within textbook content (not code blocks, nav, etc.)
 * - Disappears on deselect or outside click
 * - Routes actions to chatbot with auto-send
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useChatbot } from '../Chatbot/ChatbotContext';
import { useAuth } from '../Auth';
import styles from './styles.module.css';

const MIN_SELECTION_LENGTH = 10;
const DEBOUNCE_MS = 150;

interface ToolbarPosition {
  top: number;
  left: number;
}

/**
 * Check if selection is within allowed textbook content.
 */
function isInTextbookContent(node: Node): boolean {
  let current: Node | null = node;

  while (current) {
    if (current instanceof HTMLElement) {
      // Check for textbook content containers
      if (
        current.tagName === 'ARTICLE' ||
        current.classList.contains('markdown') ||
        current.classList.contains('theme-doc-markdown')
      ) {
        return true;
      }

      // Exclude: code blocks, nav, footer, header, chatbot, selection toolbar
      if (
        current.classList.contains('chatPanel') ||
        current.classList.contains('selectionToolbar') ||
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
 * Get selection position for toolbar placement.
 */
function getSelectionPosition(): ToolbarPosition | null {
  const selection = window.getSelection();
  if (!selection || selection.rangeCount === 0) return null;

  try {
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Position above the selection
    return {
      top: rect.top + window.scrollY - 10,
      left: rect.left + window.scrollX + rect.width / 2,
    };
  } catch {
    return null;
  }
}

export function SelectionToolbar() {
  const { triggerAction } = useChatbot();
  const { isAuthenticated } = useAuth();
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [position, setPosition] = useState<ToolbarPosition | null>(null);
  const [isVisible, setIsVisible] = useState(false);
  const toolbarRef = useRef<HTMLDivElement>(null);

  /**
   * Handle text selection changes.
   */
  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();

    if (!selection || selection.isCollapsed) {
      setIsVisible(false);
      setSelectedText(null);
      return;
    }

    const text = selection.toString().trim();

    // Check minimum length
    if (text.length < MIN_SELECTION_LENGTH) {
      setIsVisible(false);
      setSelectedText(null);
      return;
    }

    // Check if in textbook content
    try {
      const range = selection.getRangeAt(0);
      if (!isInTextbookContent(range.commonAncestorContainer)) {
        setIsVisible(false);
        setSelectedText(null);
        return;
      }
    } catch {
      setIsVisible(false);
      setSelectedText(null);
      return;
    }

    // Get position and show toolbar
    const pos = getSelectionPosition();
    if (pos) {
      setPosition(pos);
      setSelectedText(text);
      setIsVisible(true);
    }
  }, []);

  /**
   * Handle "Ask Tutor" click.
   */
  const handleAskTutor = useCallback(() => {
    if (selectedText) {
      triggerAction(selectedText, 'explain_selected_text');
      setIsVisible(false);
      window.getSelection()?.removeAllRanges();
    }
  }, [selectedText, triggerAction]);

  /**
   * Handle "Translate to Urdu" click.
   */
  const handleTranslate = useCallback(() => {
    if (selectedText) {
      triggerAction(selectedText, 'translate_urdu');
      setIsVisible(false);
      window.getSelection()?.removeAllRanges();
    }
  }, [selectedText, triggerAction]);

  /**
   * Handle click outside toolbar.
   */
  const handleClickOutside = useCallback((e: MouseEvent) => {
    if (toolbarRef.current && !toolbarRef.current.contains(e.target as Node)) {
      // Don't immediately hide if clicking within selection
      const selection = window.getSelection();
      if (!selection || selection.isCollapsed) {
        setIsVisible(false);
      }
    }
  }, []);

  // Debounced selection listener
  useEffect(() => {
    if (typeof window === 'undefined') return;

    let timeoutId: ReturnType<typeof setTimeout>;

    const debouncedHandler = () => {
      clearTimeout(timeoutId);
      timeoutId = setTimeout(handleSelectionChange, DEBOUNCE_MS);
    };

    // Listen for mouseup (desktop) and touchend (mobile long-press)
    document.addEventListener('mouseup', debouncedHandler);
    document.addEventListener('touchend', debouncedHandler);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('mouseup', debouncedHandler);
      document.removeEventListener('touchend', debouncedHandler);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [handleSelectionChange, handleClickOutside]);

  // Hide on scroll
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleScroll = () => {
      if (isVisible) {
        setIsVisible(false);
      }
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    return () => window.removeEventListener('scroll', handleScroll);
  }, [isVisible]);

  // Hide on Escape key
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        setIsVisible(false);
        window.getSelection()?.removeAllRanges();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, []);

  // Don't render if not visible or not authenticated
  if (!isVisible || !position || !isAuthenticated) {
    return null;
  }

  return (
    <div
      ref={toolbarRef}
      className={styles.toolbar}
      style={{
        top: position.top,
        left: position.left,
      }}
      role="toolbar"
      aria-label="Text selection actions"
    >
      <button
        className={styles.toolbarButton}
        onClick={handleAskTutor}
        aria-label="Ask AI Tutor about selected text"
      >
        <span className={styles.buttonIcon}>🧠</span>
        <span className={styles.buttonLabel}>Ask Tutor</span>
      </button>
      <div className={styles.divider} />
      <button
        className={styles.toolbarButton}
        onClick={handleTranslate}
        aria-label="Translate selected text to Urdu"
      >
        <span className={styles.buttonIcon}>🌐</span>
        <span className={styles.buttonLabel}>Translate</span>
      </button>
    </div>
  );
}

export default SelectionToolbar;

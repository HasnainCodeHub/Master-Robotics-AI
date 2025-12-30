/**
 * Search Modal Component
 * Centered overlay with search input and results.
 *
 * Features:
 * - Autofocus on input when opened
 * - Real-time search results
 * - Navigate to book sections
 * - Send results to Tutor Agent
 */

import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Link from '@docusaurus/Link';
import { useSearch } from './SearchContext';
import { useChatbot } from '../Chatbot/ChatbotContext';
import { useAuth } from '../Auth';
import { searchTextbook } from '../Chatbot/api';
import type { SearchResult } from './types';
import styles from './styles.module.css';

// Debounce delay for search
const SEARCH_DEBOUNCE_MS = 300;

// Toast auto-hide delay
const TOAST_DURATION_MS = 4000;

// Module display names for user-friendly labels
const MODULE_DISPLAY_NAMES: Record<string, string> = {
  'module-1': 'Module 1: Physical AI Foundations',
  'module-2': 'Module 2: ROS 2 Fundamentals',
  'module-3': 'Module 3: Simulation & Digital Twin',
  'module-4': 'Module 4: NVIDIA Isaac Ecosystem',
  'module-5': 'Module 5: Vision-Language-Action',
  'capstone': 'Capstone: Integrated Humanoid System',
  'appendix': 'Appendices',
  'root': 'General',
};

function getModuleDisplayName(moduleId: string): string {
  return MODULE_DISPLAY_NAMES[moduleId] || moduleId.replace(/-/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
}

export function SearchModal() {
  const { state, closeSearch, setQuery, setResults, setLoading, setError } = useSearch();
  const { triggerAction, openChat } = useChatbot();
  const { isAuthenticated } = useAuth();
  const history = useHistory();

  const inputRef = useRef<HTMLInputElement>(null);
  const debounceRef = useRef<NodeJS.Timeout | null>(null);
  const toastTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const [selectedIndex, setSelectedIndex] = useState(-1);
  const [showAuthToast, setShowAuthToast] = useState(false);

  // Focus input when modal opens
  useEffect(() => {
    if (state.isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [state.isOpen]);

  // Reset selection when results change
  useEffect(() => {
    setSelectedIndex(-1);
  }, [state.results]);

  // Perform search with debounce
  const performSearch = useCallback(async (query: string) => {
    if (query.length < 2) {
      setResults([]);
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await searchTextbook({ query, limit: 10 });
      setResults(response.results);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Search failed');
      setResults([]);
    }
  }, [setResults, setLoading, setError]);

  // Handle input change with debounce
  const handleInputChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const value = e.target.value;
    setQuery(value);

    // Clear existing debounce
    if (debounceRef.current) {
      clearTimeout(debounceRef.current);
    }

    // Debounce search
    debounceRef.current = setTimeout(() => {
      performSearch(value);
    }, SEARCH_DEBOUNCE_MS);
  }, [setQuery, performSearch]);

  // Get base URL for proper routing on GitHub Pages
  const textbookBaseUrl = useBaseUrl('/textbook');

  // Navigate to result with proper baseUrl handling
  const handleNavigate = useCallback((result: SearchResult) => {
    const { module_id, chapter_id } = result;
    let path: string;

    // Handle root-level documents (glossary, intro)
    if (module_id === 'root' || !module_id) {
      path = `${textbookBaseUrl}/${chapter_id}`;
    } else if (chapter_id === 'index' || !chapter_id) {
      // Handle index pages - just link to the module
      path = `${textbookBaseUrl}/${module_id}`;
    } else {
      // Standard module/chapter URLs
      path = `${textbookBaseUrl}/${module_id}/${chapter_id}`;
    }

    closeSearch();
    history.push(path);
  }, [closeSearch, history, textbookBaseUrl]);

  // Show auth toast with auto-hide
  const showAuthToastWithTimeout = useCallback(() => {
    // Clear any existing timeout
    if (toastTimeoutRef.current) {
      clearTimeout(toastTimeoutRef.current);
    }

    setShowAuthToast(true);

    toastTimeoutRef.current = setTimeout(() => {
      setShowAuthToast(false);
    }, TOAST_DURATION_MS);
  }, []);

  // Ask tutor about result
  const handleAskTutor = useCallback((result: SearchResult, e: React.MouseEvent) => {
    e.stopPropagation();

    // Check authentication
    if (!isAuthenticated) {
      showAuthToastWithTimeout();
      return;
    }

    closeSearch();

    // Create context from the result
    const context = `From "${result.section_heading}" in ${result.module_id}:\n\n${result.text}`;
    triggerAction(context, 'explain_selected_text');
  }, [isAuthenticated, closeSearch, triggerAction, showAuthToastWithTimeout]);

  // Navigate to login
  const handleLoginRedirect = useCallback(() => {
    closeSearch();
    history.push('/login');
  }, [closeSearch, history]);

  // Cleanup toast timeout on unmount
  useEffect(() => {
    return () => {
      if (toastTimeoutRef.current) {
        clearTimeout(toastTimeoutRef.current);
      }
    };
  }, []);

  // Keyboard navigation
  const handleKeyDown = useCallback((e: React.KeyboardEvent) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      setSelectedIndex(prev => Math.min(prev + 1, state.results.length - 1));
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setSelectedIndex(prev => Math.max(prev - 1, -1));
    } else if (e.key === 'Enter' && selectedIndex >= 0) {
      e.preventDefault();
      handleNavigate(state.results[selectedIndex]);
    }
  }, [state.results, selectedIndex, handleNavigate]);

  // Click outside to close
  const handleBackdropClick = useCallback((e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      closeSearch();
    }
  }, [closeSearch]);

  if (!state.isOpen) {
    return null;
  }

  return (
    <div className={styles.backdrop} onClick={handleBackdropClick}>
      <div className={styles.modal} role="dialog" aria-label="Search textbook">
        {/* Search Input */}
        <div className={styles.inputContainer}>
          <svg
            className={styles.searchIcon}
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <circle cx="11" cy="11" r="8" />
            <line x1="21" y1="21" x2="16.65" y2="16.65" />
          </svg>
          <input
            ref={inputRef}
            type="text"
            className={styles.searchInput}
            placeholder="Search the textbook..."
            value={state.query}
            onChange={handleInputChange}
            onKeyDown={handleKeyDown}
            aria-label="Search query"
          />
          <div className={styles.shortcutHint}>
            <kbd>ESC</kbd> to close
          </div>
        </div>

        {/* Results Area */}
        <div className={styles.resultsArea}>
          {/* Auth Required */}
          {!isAuthenticated && state.query.length >= 2 && (
            <div className={styles.authMessage}>
              Please login to search the textbook.
            </div>
          )}

          {/* Loading */}
          {state.isLoading && (
            <div className={styles.loadingMessage}>
              Searching...
            </div>
          )}

          {/* Error */}
          {state.error && (
            <div className={styles.errorMessage}>
              {state.error}
            </div>
          )}

          {/* No Results */}
          {!state.isLoading && !state.error && state.query.length >= 2 && state.results.length === 0 && isAuthenticated && (
            <div className={styles.noResults}>
              No results found for "{state.query}"
            </div>
          )}

          {/* Results List */}
          {!state.isLoading && state.results.length > 0 && (
            <ul className={styles.resultsList}>
              {state.results.map((result, index) => (
                <li
                  key={result.chunk_id}
                  className={`${styles.resultItem} ${index === selectedIndex ? styles.resultItemSelected : ''}`}
                  onClick={() => handleNavigate(result)}
                >
                  <div className={styles.resultHeader}>
                    <span className={styles.resultModule}>{getModuleDisplayName(result.module_id)}</span>
                    <span className={styles.resultScore}>
                      {Math.round(result.score * 100)}% match
                    </span>
                  </div>
                  <div className={styles.resultHeading}>
                    {result.section_heading || 'Untitled Section'}
                  </div>
                  <div className={styles.resultSnippet}>
                    {result.highlight || result.text.slice(0, 150) + '...'}
                  </div>
                  <div className={styles.resultActions}>
                    <button
                      className={styles.askTutorButton}
                      onClick={(e) => handleAskTutor(result, e)}
                      title="Ask the AI tutor about this section"
                    >
                      <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
                      </svg>
                      Ask Tutor
                    </button>
                  </div>
                </li>
              ))}
            </ul>
          )}

          {/* Empty State */}
          {state.query.length < 2 && (
            <div className={styles.emptyState}>
              <div className={styles.emptyIcon}>
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
                  <circle cx="11" cy="11" r="8" />
                  <line x1="21" y1="21" x2="16.65" y2="16.65" />
                </svg>
              </div>
              <p>Type to search the Physical AI & Humanoid Robotics textbook</p>
              <p className={styles.emptyHint}>
                Search for concepts, topics, or keywords
              </p>
            </div>
          )}
        </div>

        {/* Footer */}
        <div className={styles.footer}>
          <div className={styles.footerHint}>
            <kbd>&uarr;</kbd> <kbd>&darr;</kbd> to navigate
            <span className={styles.footerSeparator}>|</span>
            <kbd>Enter</kbd> to open
            <span className={styles.footerSeparator}>|</span>
            <kbd>Ctrl</kbd>+<kbd>K</kbd> to toggle
          </div>
        </div>

        {/* Auth Toast */}
        {showAuthToast && (
          <div className={styles.authToast}>
            <div className={styles.authToastContent}>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
                <path d="M7 11V7a5 5 0 0 1 10 0v4" />
              </svg>
              <span>Login required to ask the Tutor</span>
              <button
                className={styles.authToastButton}
                onClick={handleLoginRedirect}
              >
                Login
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

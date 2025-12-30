/**
 * Main Chatbot Component
 * Embedded RAG chatbot for the Physical AI & Humanoid Robotics textbook.
 *
 * SAFETY RULES:
 * - Frontend ONLY communicates with FastAPI backend (NO direct LLM calls)
 * - All answers come from retrieved textbook content
 * - Refusals are displayed clearly (not as errors)
 * - Sources are always visible for grounded answers
 * - Chatbot is ONLY available to authenticated users
 */

import React, { useCallback, useEffect, useRef, useState } from 'react';
import Link from '@docusaurus/Link';
import { useChatbot } from './ChatbotContext';
import { ChatMessages } from './ChatMessages';
import { ChatInput } from './ChatInput';
import { ChatSidebar } from './ChatSidebar';
import { sendChatMessage, createSession } from './api';
import { translateToUrdu } from '../Translation/api';
import { useAuth } from '../Auth';
import type { Message, ChatMode } from './types';
import styles from './styles.module.css';

function generateId(): string {
  return `msg-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`;
}

export function Chatbot() {
  const { isAuthenticated, isLoading: authLoading } = useAuth();

  const {
    state,
    selectionState,
    pendingAction,
    toggleChat,
    closeChat,
    addMessage,
    setLoading,
    setError,
    setSessionId,
    clearPendingAction,
    loadMessages,
    clearMessages,
  } = useChatbot();

  // Track if we've processed the pending action
  const pendingActionProcessed = useRef(false);
  const [showSidebar, setShowSidebar] = useState(false);

  // Handle creating a new chat session
  const handleNewChat = useCallback(async () => {
    try {
      // Clear current messages first
      clearMessages();
      // Create a new session on the backend
      const response = await createSession();
      setSessionId(response.session_id);
    } catch (err) {
      // If creation fails, just clear locally (session will be created on first message)
      clearMessages();
      setSessionId(null);
    }
  }, [clearMessages, setSessionId]);

  const handleSubmit = useCallback(async (question: string, selectedText: string | null, mode: ChatMode = 'general') => {
    // Add user message
    const userMessage: Message = {
      id: generateId(),
      role: 'user',
      content: question,
      timestamp: new Date(),
      selectedText: selectedText ? {
        text: selectedText,
        length: selectedText.length,
        truncated: selectedText.length > 5000,
      } : undefined,
    };
    addMessage(userMessage);

    // Set loading state
    setLoading(true);
    setError(null);

    try {
      // Handle translation mode separately - use translation API directly
      if (mode === 'translate_urdu' && selectedText) {
        const translationResponse = await translateToUrdu(selectedText);

        // Add assistant message with translation
        const assistantMessage: Message = {
          id: generateId(),
          role: 'assistant',
          content: `**Urdu Translation:**\n\n${translationResponse.translated_text}\n\n---\n*Technical terms preserved: ${translationResponse.preserved_terms.join(', ') || 'None'}*`,
          timestamp: new Date(),
          isRefusal: false,
        };
        addMessage(assistantMessage);
      } else {
        // Send to RAG backend for explain/general modes
        const response = await sendChatMessage({
          question,
          selected_text: selectedText,
          session_id: state.sessionId,
          mode,
        });

        // Save session ID
        if (response.session_id && response.session_id !== state.sessionId) {
          setSessionId(response.session_id);
        }

        // Add assistant message
        const assistantMessage: Message = {
          id: generateId(),
          role: 'assistant',
          content: response.answer,
          timestamp: new Date(),
          sources: response.sources,
          isRefusal: response.was_refusal,
          refusalReason: response.refusal_reason,
        };
        addMessage(assistantMessage);
      }

    } catch (error) {
      // Handle errors gracefully
      const errorMessage = error instanceof Error
        ? error.message
        : 'An unexpected error occurred. Please try again.';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  }, [state.sessionId, addMessage, setLoading, setError, setSessionId]);

  // Process pending action when chat opens
  useEffect(() => {
    if (
      pendingAction &&
      state.isOpen &&
      isAuthenticated &&
      !state.isLoading &&
      !pendingActionProcessed.current
    ) {
      pendingActionProcessed.current = true;

      // Build the question based on mode
      let question: string;
      if (pendingAction.mode === 'explain_selected_text') {
        question = `Please explain this passage from the textbook:\n\n"${pendingAction.text}"`;
      } else if (pendingAction.mode === 'translate_urdu') {
        // For translation, show what we're translating
        question = `Translate to Urdu:\n\n"${pendingAction.text.substring(0, 200)}${pendingAction.text.length > 200 ? '...' : ''}"`;
      } else {
        question = pendingAction.text;
      }

      // Auto-send the message (translation will use the full text via selectedText)
      handleSubmit(question, pendingAction.text, pendingAction.mode);
      clearPendingAction();
    }
  }, [pendingAction, state.isOpen, isAuthenticated, state.isLoading, handleSubmit, clearPendingAction]);

  // Reset processed flag when pending action changes
  useEffect(() => {
    if (!pendingAction) {
      pendingActionProcessed.current = false;
    }
  }, [pendingAction]);

  return (
    <>
      {/* Floating Action Button */}
      {!state.isOpen && (
        <button
          className={styles.chatbotFab}
          onClick={toggleChat}
          aria-label="Open chatbot"
          title="Ask AI about the textbook"
        >
          <svg
            className={styles.fabIcon}
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat Panel */}
      {state.isOpen && (
        <div
          className={styles.chatPanel}
          role="dialog"
          aria-label="Chatbot"
          aria-modal="false"
        >
          {/* Sidebar */}
          {showSidebar && isAuthenticated && (
            <ChatSidebar onClose={() => setShowSidebar(false)} />
          )}

          {/* Header */}
          <div className={styles.chatHeader}>
            <h2 className={styles.chatTitle}>Ask the Textbook</h2>
            <div className={styles.headerActions}>
              {isAuthenticated && (
                <>
                  <button
                    className={styles.newChatHeaderButton}
                    onClick={handleNewChat}
                    aria-label="New chat"
                    title="Start new chat"
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
                      <line x1="12" y1="5" x2="12" y2="19" />
                      <line x1="5" y1="12" x2="19" y2="12" />
                    </svg>
                  </button>
                  <button
                    className={styles.historyButton}
                    onClick={() => setShowSidebar(!showSidebar)}
                    aria-label="Chat history"
                    title="Chat history"
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
                      <circle cx="12" cy="12" r="10" />
                      <polyline points="12 6 12 12 16 14" />
                    </svg>
                  </button>
                </>
              )}
              <button
                className={styles.closeButton}
                onClick={closeChat}
                aria-label="Close chatbot"
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
                  <line x1="18" y1="6" x2="6" y2="18"></line>
                  <line x1="6" y1="6" x2="18" y2="18"></line>
                </svg>
              </button>
            </div>
          </div>

          {/* Authentication Check */}
          {!isAuthenticated && !authLoading ? (
            <div className={styles.authRequired}>
              <div className={styles.authIcon}>
                <svg
                  width="48"
                  height="48"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="1.5"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
                  <path d="M7 11V7a5 5 0 0 1 10 0v4" />
                </svg>
              </div>
              <p className={styles.authMessage}>
                Please login to use the AI tutor.
              </p>
              <Link
                to="/login"
                className={styles.authButton}
              >
                Login / Sign Up
              </Link>
            </div>
          ) : (
            <>
              {/* Messages */}
              <ChatMessages
                messages={state.messages}
                isLoading={state.isLoading}
              />

              {/* Error Display */}
              {state.error && (
                <div className={styles.errorMessage}>
                  <span>⚠️</span>
                  <div>
                    <div>{state.error}</div>
                    <button
                      className={styles.retryButton}
                      onClick={() => setError(null)}
                    >
                      Dismiss
                    </button>
                  </div>
                </div>
              )}

              {/* Input */}
              <ChatInput
                onSubmit={handleSubmit}
                disabled={state.isLoading}
              />
            </>
          )}
        </div>
      )}
    </>
  );
}

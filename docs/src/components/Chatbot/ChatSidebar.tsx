/**
 * Chat Sidebar Component
 *
 * Shows list of previous chat sessions and allows:
 * - Viewing session history
 * - Switching between sessions
 * - Creating new sessions
 * - Deleting sessions
 */

import React, { useEffect, useState, useCallback } from 'react';
import { useChatbot } from './ChatbotContext';
import { listSessions, getSession, createSession, deleteSession } from './api';
import type { SessionSummary, Message } from './types';
import styles from './styles.module.css';

interface ChatSidebarProps {
  onClose: () => void;
}

export function ChatSidebar({ onClose }: ChatSidebarProps) {
  const {
    state,
    setSessionId,
    loadMessages,
    clearMessages,
  } = useChatbot();

  const [sessions, setSessions] = useState<SessionSummary[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Load sessions on mount
  useEffect(() => {
    const loadSessionList = async () => {
      try {
        setIsLoading(true);
        setError(null);
        const response = await listSessions();
        setSessions(response.sessions);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to load sessions');
      } finally {
        setIsLoading(false);
      }
    };

    loadSessionList();
  }, []);

  // Switch to a different session
  const handleSwitchSession = useCallback(async (sessionId: string) => {
    if (sessionId === state.sessionId) {
      onClose();
      return;
    }

    try {
      setError(null);
      const sessionDetail = await getSession(sessionId);

      // Convert MessageResponse to Message format
      const messages: Message[] = sessionDetail.messages.map(msg => ({
        id: msg.id,
        role: msg.role,
        content: msg.content,
        timestamp: new Date(msg.created_at),
        sources: msg.sources || undefined,
        isRefusal: msg.was_refusal,
        refusalReason: msg.refusal_reason,
      }));

      setSessionId(sessionId);
      loadMessages(messages);
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load session');
    }
  }, [state.sessionId, setSessionId, loadMessages, onClose]);

  // Create a new session
  const handleNewSession = useCallback(async () => {
    try {
      setError(null);
      const response = await createSession();
      setSessionId(response.session_id);
      clearMessages();
      onClose();

      // Refresh sessions list
      const listResponse = await listSessions();
      setSessions(listResponse.sessions);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to create session');
    }
  }, [setSessionId, clearMessages, onClose]);

  // Delete a session
  const handleDeleteSession = useCallback(async (sessionId: string, e: React.MouseEvent) => {
    e.stopPropagation();

    if (!confirm('Delete this chat session?')) {
      return;
    }

    try {
      setError(null);
      await deleteSession(sessionId);

      // Remove from list
      setSessions(prev => prev.filter(s => s.id !== sessionId));

      // If we deleted the current session, clear messages
      if (sessionId === state.sessionId) {
        clearMessages();
        // Create a new session
        const response = await createSession();
        setSessionId(response.session_id);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to delete session');
    }
  }, [state.sessionId, clearMessages, setSessionId]);

  // Format date for display
  const formatDate = (dateStr: string) => {
    const date = new Date(dateStr);
    const now = new Date();
    const diff = now.getTime() - date.getTime();
    const days = Math.floor(diff / (1000 * 60 * 60 * 24));

    if (days === 0) {
      return 'Today';
    } else if (days === 1) {
      return 'Yesterday';
    } else if (days < 7) {
      return `${days} days ago`;
    } else {
      return date.toLocaleDateString();
    }
  };

  return (
    <div className={styles.sidebar}>
      <div className={styles.sidebarHeader}>
        <h3 className={styles.sidebarTitle}>Chat History</h3>
        <button
          className={styles.sidebarClose}
          onClick={onClose}
          aria-label="Close sidebar"
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        </button>
      </div>

      <button className={styles.newChatButton} onClick={handleNewSession}>
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <line x1="12" y1="5" x2="12" y2="19" />
          <line x1="5" y1="12" x2="19" y2="12" />
        </svg>
        New Chat
      </button>

      {error && (
        <div className={styles.sidebarError}>
          {error}
        </div>
      )}

      <div className={styles.sessionList}>
        {isLoading ? (
          <div className={styles.sidebarLoading}>Loading sessions...</div>
        ) : sessions.length === 0 ? (
          <div className={styles.sidebarEmpty}>No previous chats</div>
        ) : (
          sessions.map(session => (
            <div
              key={session.id}
              className={`${styles.sessionItem} ${session.id === state.sessionId ? styles.sessionItemActive : ''}`}
              onClick={() => handleSwitchSession(session.id)}
            >
              <div className={styles.sessionInfo}>
                <div className={styles.sessionPreview}>
                  {session.preview || 'Empty chat'}
                </div>
                <div className={styles.sessionMeta}>
                  <span className={styles.sessionDate}>{formatDate(session.started_at)}</span>
                  <span className={styles.sessionMessages}>{session.message_count} messages</span>
                </div>
              </div>
              <button
                className={styles.sessionDelete}
                onClick={(e) => handleDeleteSession(session.id, e)}
                aria-label="Delete session"
              >
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <polyline points="3 6 5 6 21 6" />
                  <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
                </svg>
              </button>
            </div>
          ))
        )}
      </div>
    </div>
  );
}

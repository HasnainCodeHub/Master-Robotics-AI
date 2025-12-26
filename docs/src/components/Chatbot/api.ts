/**
 * API client for the RAG chatbot backend.
 * IMPORTANT: Frontend ONLY communicates with FastAPI backend.
 * NO direct LLM calls are permitted.
 * SECURITY: All chat requests require authentication.
 *
 * BROWSER-SAFE: No process.env usage. Uses getApiBaseUrl() utility.
 */

import type {
  ChatRequest,
  ChatResponse,
  SessionListResponse,
  SessionDetail,
  CreateSessionResponse,
  SearchRequest,
  SearchResponse,
} from './types';
import { getStoredToken } from '../Auth';
import { getApiBaseUrl } from '../../utils/config';

/**
 * Get API base URL lazily (not at module import time).
 * This prevents SSR crashes and allows runtime configuration.
 */
function getBaseUrl(): string {
  return getApiBaseUrl();
}

/**
 * Send a chat message to the RAG backend.
 * Requires authentication - will return 401 if not authenticated.
 *
 * @param request - The chat request with question and optional selected text
 * @returns The chat response with answer and sources
 * @throws Error on network or server errors
 */
export async function sendChatMessage(request: ChatRequest): Promise<ChatResponse> {
  const token = getStoredToken();

  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };

  // Add authorization header if token exists
  if (token) {
    (headers as Record<string, string>)['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${getBaseUrl()}/api/chat`, {
    method: 'POST',
    headers,
    credentials: 'include',
    body: JSON.stringify({
      question: request.question,
      selected_text: request.selected_text || null,
      session_id: request.session_id || null,
      mode: request.mode || 'general',
    }),
  });

  if (!response.ok) {
    // Handle authentication errors
    if (response.status === 401) {
      throw new Error('Authentication required. Please login to use the AI tutor.');
    }

    // Handle different error types
    if (response.status >= 500) {
      throw new Error('Service temporarily unavailable. Please try again in a moment.');
    }

    // Try to get validation error details
    try {
      const errorData = await response.json();
      if (errorData.detail) {
        // Pydantic validation error
        if (Array.isArray(errorData.detail)) {
          const messages = errorData.detail.map((d: { msg: string }) => d.msg).join(', ');
          throw new Error(messages);
        }
        throw new Error(errorData.detail);
      }
    } catch (e) {
      if (e instanceof Error && e.message !== 'Service temporarily unavailable. Please try again in a moment.') {
        throw e;
      }
    }

    throw new Error(`Request failed with status ${response.status}`);
  }

  return response.json();
}

/**
 * Check if the backend is healthy.
 *
 * @returns True if backend is reachable
 */
export async function checkHealth(): Promise<boolean> {
  try {
    const response = await fetch(`${getBaseUrl()}/api/health`, {
      method: 'GET',
    });
    return response.ok;
  } catch {
    return false;
  }
}


// ============================================
// Session Management API
// ============================================

/**
 * Helper to get auth headers.
 */
function getAuthHeaders(): HeadersInit {
  const token = getStoredToken();
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
  };
  if (token) {
    (headers as Record<string, string>)['Authorization'] = `Bearer ${token}`;
  }
  return headers;
}

/**
 * List all chat sessions for the current user.
 *
 * @returns List of session summaries
 */
export async function listSessions(): Promise<SessionListResponse> {
  const response = await fetch(`${getBaseUrl()}/api/sessions`, {
    method: 'GET',
    headers: getAuthHeaders(),
    credentials: 'include',
  });

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Authentication required');
    }
    throw new Error(`Failed to list sessions: ${response.status}`);
  }

  return response.json();
}

/**
 * Get the last active session with messages.
 *
 * @returns Session detail or null if no sessions exist
 */
export async function getLastSession(): Promise<SessionDetail | null> {
  const response = await fetch(`${getBaseUrl()}/api/sessions/last`, {
    method: 'GET',
    headers: getAuthHeaders(),
    credentials: 'include',
  });

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Authentication required');
    }
    throw new Error(`Failed to get last session: ${response.status}`);
  }

  const data = await response.json();
  return data || null;
}

/**
 * Get a specific session with all messages.
 *
 * @param sessionId - Session ID
 * @returns Session detail
 */
export async function getSession(sessionId: string): Promise<SessionDetail> {
  const response = await fetch(`${getBaseUrl()}/api/sessions/${sessionId}`, {
    method: 'GET',
    headers: getAuthHeaders(),
    credentials: 'include',
  });

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Authentication required');
    }
    if (response.status === 404) {
      throw new Error('Session not found');
    }
    throw new Error(`Failed to get session: ${response.status}`);
  }

  return response.json();
}

/**
 * Create a new chat session.
 *
 * @returns New session info
 */
export async function createSession(): Promise<CreateSessionResponse> {
  const response = await fetch(`${getBaseUrl()}/api/sessions`, {
    method: 'POST',
    headers: getAuthHeaders(),
    credentials: 'include',
  });

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Authentication required');
    }
    throw new Error(`Failed to create session: ${response.status}`);
  }

  return response.json();
}

/**
 * Delete a chat session.
 *
 * @param sessionId - Session ID to delete
 */
export async function deleteSession(sessionId: string): Promise<void> {
  const response = await fetch(`${getBaseUrl()}/api/sessions/${sessionId}`, {
    method: 'DELETE',
    headers: getAuthHeaders(),
    credentials: 'include',
  });

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Authentication required');
    }
    if (response.status === 404) {
      throw new Error('Session not found');
    }
    throw new Error(`Failed to delete session: ${response.status}`);
  }
}


// ============================================
// Search API
// ============================================

/**
 * Search the textbook content.
 *
 * @param request - Search request with query and optional limit
 * @returns Search results with ranked matches
 */
export async function searchTextbook(request: SearchRequest): Promise<SearchResponse> {
  const response = await fetch(`${getBaseUrl()}/api/search`, {
    method: 'POST',
    headers: getAuthHeaders(),
    credentials: 'include',
    body: JSON.stringify({
      query: request.query,
      limit: request.limit || 10,
    }),
  });

  if (!response.ok) {
    if (response.status === 401) {
      throw new Error('Authentication required. Please login to search.');
    }
    if (response.status >= 500) {
      throw new Error('Search service temporarily unavailable.');
    }
    throw new Error(`Search failed: ${response.status}`);
  }

  return response.json();
}

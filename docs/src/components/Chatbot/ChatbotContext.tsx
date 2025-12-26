/**
 * Chatbot Context Provider
 * Manages global chat state and text selection state.
 *
 * SSR-SAFE: Uses isClient guard to prevent hydration mismatches.
 * Browser-only APIs (sessionStorage, window) are only accessed after client mount.
 */

import React, { createContext, useContext, useReducer, useCallback, useEffect, useState, ReactNode } from 'react';
import type { Message, ChatState, SelectionState, ChatMode, SessionSummary } from './types';

// Pending action for auto-send
interface PendingAction {
  text: string;
  mode: ChatMode;
}

// Session management state
interface SessionState {
  sessions: SessionSummary[];
  isLoadingSessions: boolean;
  showSidebar: boolean;
}

// Chat State
interface ChatContextType {
  state: ChatState;
  selectionState: SelectionState;
  sessionState: SessionState;
  pendingAction: PendingAction | null;
  openChat: () => void;
  closeChat: () => void;
  toggleChat: () => void;
  addMessage: (message: Message) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  setSessionId: (sessionId: string) => void;
  setSelection: (text: string | null) => void;
  clearSelection: () => void;
  clearMessages: () => void;
  triggerAction: (text: string, mode: ChatMode) => void;
  clearPendingAction: () => void;
  // Session management
  loadSessions: () => Promise<void>;
  switchSession: (sessionId: string) => Promise<void>;
  createNewSession: () => Promise<void>;
  deleteSessionById: (sessionId: string) => Promise<void>;
  toggleSidebar: () => void;
  loadMessages: (messages: Message[]) => void;
}

const ChatContext = createContext<ChatContextType | null>(null);

// Actions
type ChatAction =
  | { type: 'OPEN' }
  | { type: 'CLOSE' }
  | { type: 'TOGGLE' }
  | { type: 'ADD_MESSAGE'; payload: Message }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'SET_SESSION_ID'; payload: string }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'LOAD_MESSAGES'; payload: Message[] };

type SelectionAction =
  | { type: 'SET_SELECTION'; payload: string }
  | { type: 'CLEAR_SELECTION' };

const MAX_SELECTION_LENGTH = 5000;
const MIN_SELECTION_LENGTH = 50;

// Reducers
function chatReducer(state: ChatState, action: ChatAction): ChatState {
  switch (action.type) {
    case 'OPEN':
      return { ...state, isOpen: true };
    case 'CLOSE':
      return { ...state, isOpen: false };
    case 'TOGGLE':
      return { ...state, isOpen: !state.isOpen };
    case 'ADD_MESSAGE':
      return { ...state, messages: [...state.messages, action.payload] };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload };
    case 'SET_SESSION_ID':
      return { ...state, sessionId: action.payload };
    case 'CLEAR_MESSAGES':
      return { ...state, messages: [] };
    case 'LOAD_MESSAGES':
      return { ...state, messages: action.payload };
    default:
      return state;
  }
}

function selectionReducer(state: SelectionState, action: SelectionAction): SelectionState {
  switch (action.type) {
    case 'SET_SELECTION': {
      const text = action.payload;
      const isTruncated = text.length > MAX_SELECTION_LENGTH;
      const finalText = isTruncated ? text.slice(0, MAX_SELECTION_LENGTH) : text;
      const isValid = finalText.length >= MIN_SELECTION_LENGTH;

      return {
        selectedText: finalText,
        isValid,
        charCount: finalText.length,
        isTruncated,
      };
    }
    case 'CLEAR_SELECTION':
      return {
        selectedText: null,
        isValid: false,
        charCount: 0,
        isTruncated: false,
      };
    default:
      return state;
  }
}

// Initial states
const initialChatState: ChatState = {
  isOpen: false,
  messages: [],
  isLoading: false,
  sessionId: null,
  error: null,
};

const initialSelectionState: SelectionState = {
  selectedText: null,
  isValid: false,
  charCount: 0,
  isTruncated: false,
};

const initialSessionState: SessionState = {
  sessions: [],
  isLoadingSessions: false,
  showSidebar: false,
};

// Provider
interface ChatbotProviderProps {
  children: ReactNode;
}

/**
 * ChatbotProvider component.
 * SSR-SAFE: Uses isClient state to delay browser API access.
 */
export function ChatbotProvider({ children }: ChatbotProviderProps) {
  // SSR-safe: Track if we're on the client
  const [isClient, setIsClient] = useState(false);

  const [chatState, chatDispatch] = useReducer(chatReducer, initialChatState);
  const [selectionState, selectionDispatch] = useReducer(selectionReducer, initialSelectionState);
  const [pendingAction, setPendingAction] = useState<PendingAction | null>(null);
  const [sessionState, setSessionState] = useState<SessionState>(initialSessionState);

  /**
   * Mark as client-side mounted.
   */
  useEffect(() => {
    setIsClient(true);
  }, []);

  // Restore session ID from sessionStorage (client-only)
  useEffect(() => {
    if (!isClient) return;

    const storedSessionId = sessionStorage.getItem('chatbot_session_id');
    if (storedSessionId) {
      chatDispatch({ type: 'SET_SESSION_ID', payload: storedSessionId });
    }
  }, [isClient]);

  // Actions
  const openChat = useCallback(() => chatDispatch({ type: 'OPEN' }), []);
  const closeChat = useCallback(() => chatDispatch({ type: 'CLOSE' }), []);
  const toggleChat = useCallback(() => chatDispatch({ type: 'TOGGLE' }), []);

  const addMessage = useCallback((message: Message) => {
    chatDispatch({ type: 'ADD_MESSAGE', payload: message });
  }, []);

  const setLoading = useCallback((loading: boolean) => {
    chatDispatch({ type: 'SET_LOADING', payload: loading });
  }, []);

  const setError = useCallback((error: string | null) => {
    chatDispatch({ type: 'SET_ERROR', payload: error });
  }, []);

  const setSessionId = useCallback((sessionId: string) => {
    chatDispatch({ type: 'SET_SESSION_ID', payload: sessionId });
    // SSR-safe: Only access sessionStorage on client
    if (typeof window !== 'undefined') {
      try {
        sessionStorage.setItem('chatbot_session_id', sessionId);
      } catch {
        // Ignore storage errors (e.g., private browsing)
      }
    }
  }, []);

  const setSelection = useCallback((text: string | null) => {
    if (text) {
      selectionDispatch({ type: 'SET_SELECTION', payload: text });
    } else {
      selectionDispatch({ type: 'CLEAR_SELECTION' });
    }
  }, []);

  const clearSelection = useCallback(() => {
    selectionDispatch({ type: 'CLEAR_SELECTION' });
  }, []);

  const clearMessages = useCallback(() => {
    chatDispatch({ type: 'CLEAR_MESSAGES' });
  }, []);

  /**
   * Trigger a contextual action from text selection.
   * Opens the chat and queues an auto-send action.
   */
  const triggerAction = useCallback((text: string, mode: ChatMode) => {
    setPendingAction({ text, mode });
    chatDispatch({ type: 'OPEN' });
  }, []);

  const clearPendingAction = useCallback(() => {
    setPendingAction(null);
  }, []);

  const loadMessages = useCallback((messages: Message[]) => {
    chatDispatch({ type: 'LOAD_MESSAGES', payload: messages });
  }, []);

  const toggleSidebar = useCallback(() => {
    setSessionState(prev => ({ ...prev, showSidebar: !prev.showSidebar }));
  }, []);

  // Session management functions will be added via the api module
  // These are placeholder implementations that will be connected in the Chatbot component
  const loadSessions = useCallback(async () => {
    // Implemented in Chatbot component with API calls
  }, []);

  const switchSession = useCallback(async (_sessionId: string) => {
    // Implemented in Chatbot component with API calls
  }, []);

  const createNewSession = useCallback(async () => {
    // Implemented in Chatbot component with API calls
  }, []);

  const deleteSessionById = useCallback(async (_sessionId: string) => {
    // Implemented in Chatbot component with API calls
  }, []);

  // Handle ESC key to close chat (client-only)
  useEffect(() => {
    // SSR-safe: Only attach event listeners on client
    if (!isClient) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && chatState.isOpen) {
        closeChat();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isClient, chatState.isOpen, closeChat]);

  const value: ChatContextType = {
    state: chatState,
    selectionState,
    sessionState,
    pendingAction,
    openChat,
    closeChat,
    toggleChat,
    addMessage,
    setLoading,
    setError,
    setSessionId,
    setSelection,
    clearSelection,
    clearMessages,
    triggerAction,
    clearPendingAction,
    // Session management
    loadSessions,
    switchSession,
    createNewSession,
    deleteSessionById,
    toggleSidebar,
    loadMessages,
  };

  return (
    <ChatContext.Provider value={value}>
      {children}
    </ChatContext.Provider>
  );
}

export function useChatbot(): ChatContextType {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatbot must be used within a ChatbotProvider');
  }
  return context;
}

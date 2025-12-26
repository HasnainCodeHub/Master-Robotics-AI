/**
 * Type definitions for the RAG Chatbot component.
 * Mirrors backend schemas from backend/app/schemas/chat.py
 */

export interface SourceReference {
  chunk_id: string;
  module_id: string;
  chapter_id: string;
  section_heading: string;
  score: number;
}

export type ChatMode = 'general' | 'explain_selected_text' | 'translate_urdu';

export interface ChatRequest {
  question: string;
  selected_text?: string | null;
  session_id?: string | null;
  mode?: ChatMode;
}

export interface ChatResponse {
  answer: string;
  sources: SourceReference[];
  was_refusal: boolean;
  refusal_reason?: string | null;
  session_id: string;
}

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: SourceReference[];
  isRefusal?: boolean;
  refusalReason?: string | null;
  selectedText?: {
    text: string;
    length: number;
    truncated: boolean;
  };
}

export interface ChatState {
  isOpen: boolean;
  messages: Message[];
  isLoading: boolean;
  sessionId: string | null;
  error: string | null;
}

export type RefusalReason =
  | 'empty_retrieval'
  | 'insufficient_context'
  | 'out_of_scope'
  | 'selected_text_insufficient';

export interface SelectionState {
  selectedText: string | null;
  isValid: boolean;
  charCount: number;
  isTruncated: boolean;
}

// ============================================
// Session Management Types
// ============================================

export interface SessionSummary {
  id: string;
  started_at: string;
  message_count: number;
  preview: string | null;
}

export interface MessageResponse {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources: SourceReference[] | null;
  was_refusal: boolean;
  refusal_reason: string | null;
  created_at: string;
}

export interface SessionDetail {
  id: string;
  started_at: string;
  messages: MessageResponse[];
}

export interface SessionListResponse {
  sessions: SessionSummary[];
  total: number;
}

export interface CreateSessionResponse {
  session_id: string;
  created_at: string;
}

// ============================================
// Search Types
// ============================================

export interface SearchRequest {
  query: string;
  limit?: number;
}

export interface SearchResult {
  chunk_id: string;
  text: string;
  module_id: string;
  chapter_id: string;
  section_heading: string;
  score: number;
  highlight: string | null;
}

export interface SearchResponse {
  query: string;
  results: SearchResult[];
  total: number;
}

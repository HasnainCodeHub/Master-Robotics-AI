/**
 * Chatbot Component Exports
 * Main entry point for the embedded RAG chatbot.
 */

export { Chatbot } from './Chatbot';
export { ChatbotProvider, useChatbot } from './ChatbotContext';
export { useTextSelection } from './useTextSelection';
export type { Message, ChatRequest, ChatResponse, SourceReference, ChatMode } from './types';

/**
 * Custom Root Component
 * Wraps the entire Docusaurus app with:
 * - ErrorBoundary for runtime error handling
 * - AuthProvider for authentication state
 * - PersonalizationProvider for presentation-level personalization
 * - TranslationProvider for Urdu translation (Phase 6)
 * - ChatbotProvider for chatbot state
 * - Chatbot component with text selection
 * - Translation FAB and error toast
 *
 * Provider Order (outer to inner):
 * 1. ErrorBoundary - catches runtime errors, prevents blank screens
 * 2. AuthProvider - provides user/profile state
 * 3. PersonalizationProvider - consumes auth, provides personalization settings
 * 4. TranslationProvider - provides translation state (presentation only)
 * 5. ChatbotProvider - chatbot state management
 *
 * IMPORTANT: ErrorBoundary is OUTERMOST to catch errors in any provider.
 *
 * IMPORTANT: PersonalizationProvider must be AFTER AuthProvider
 * because it consumes useAuth() to read profile data.
 *
 * IMPORTANT: TranslationProvider is PRESENTATION ONLY and does NOT
 * affect RAG retrieval or answer generation.
 *
 * IMPORTANT: Browser-only components (Chatbot, TranslationToggle, etc.)
 * are wrapped in BrowserOnly to prevent SSR hydration issues.
 */

import React, { ReactNode } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ErrorBoundary } from '../components/ErrorBoundary';
import { AuthProvider } from '../components/Auth/AuthContext';
import { PersonalizationProvider } from '../components/Personalization';
import {
  TranslationProvider,
  TranslationToggle,
  TranslationErrorToast,
} from '../components/Translation';
import { ChatbotProvider, Chatbot, useTextSelection } from '../components/Chatbot';
import { SelectionToolbar } from '../components/SelectionToolbar';
import { SearchProvider, SearchModal } from '../components/Search';

interface RootProps {
  children: ReactNode;
}

/**
 * ChatbotWithSelection - Renders chatbot with text selection support.
 * Must be used inside ChatbotProvider context.
 */
function ChatbotWithSelection() {
  // Hook to capture text selection
  useTextSelection();
  return <Chatbot />;
}

/**
 * BrowserOnlyComponents - Components that require browser APIs.
 * Wrapped in BrowserOnly to prevent SSR hydration mismatches.
 */
function BrowserOnlyComponents() {
  return (
    <>
      <ChatbotWithSelection />
      <SelectionToolbar />
      <SearchModal />
      <TranslationToggle variant="fab" />
      <TranslationErrorToast />
    </>
  );
}

export default function Root({ children }: RootProps) {
  return (
    <ErrorBoundary>
      <AuthProvider>
        <PersonalizationProvider>
          <TranslationProvider>
            <ChatbotProvider>
              <SearchProvider>
                {children}
                <BrowserOnly fallback={null}>
                  {() => <BrowserOnlyComponents />}
                </BrowserOnly>
              </SearchProvider>
            </ChatbotProvider>
          </TranslationProvider>
        </PersonalizationProvider>
      </AuthProvider>
    </ErrorBoundary>
  );
}

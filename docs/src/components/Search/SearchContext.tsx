/**
 * Search Context Provider
 * Manages global search state and keyboard shortcuts.
 *
 * SSR-SAFE: Uses isClient guard for browser-only APIs.
 */

import React, {
  createContext,
  useContext,
  useReducer,
  useCallback,
  useEffect,
  useState,
  ReactNode,
} from 'react';
import type { SearchState, SearchResult } from './types';

interface SearchContextType {
  state: SearchState;
  openSearch: () => void;
  closeSearch: () => void;
  setQuery: (query: string) => void;
  setResults: (results: SearchResult[]) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  clearSearch: () => void;
}

const SearchContext = createContext<SearchContextType | null>(null);

type SearchAction =
  | { type: 'OPEN' }
  | { type: 'CLOSE' }
  | { type: 'SET_QUERY'; payload: string }
  | { type: 'SET_RESULTS'; payload: SearchResult[] }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'CLEAR' };

const initialState: SearchState = {
  isOpen: false,
  query: '',
  results: [],
  isLoading: false,
  error: null,
};

function searchReducer(state: SearchState, action: SearchAction): SearchState {
  switch (action.type) {
    case 'OPEN':
      return { ...state, isOpen: true };
    case 'CLOSE':
      return { ...state, isOpen: false };
    case 'SET_QUERY':
      return { ...state, query: action.payload };
    case 'SET_RESULTS':
      return { ...state, results: action.payload, isLoading: false, error: null };
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload, isLoading: false };
    case 'CLEAR':
      return { ...initialState };
    default:
      return state;
  }
}

interface SearchProviderProps {
  children: ReactNode;
}

export function SearchProvider({ children }: SearchProviderProps) {
  const [isClient, setIsClient] = useState(false);
  const [state, dispatch] = useReducer(searchReducer, initialState);

  // Mark as client-side mounted
  useEffect(() => {
    setIsClient(true);
  }, []);

  const openSearch = useCallback(() => dispatch({ type: 'OPEN' }), []);
  const closeSearch = useCallback(() => dispatch({ type: 'CLOSE' }), []);
  const setQuery = useCallback((query: string) => dispatch({ type: 'SET_QUERY', payload: query }), []);
  const setResults = useCallback((results: SearchResult[]) => dispatch({ type: 'SET_RESULTS', payload: results }), []);
  const setLoading = useCallback((loading: boolean) => dispatch({ type: 'SET_LOADING', payload: loading }), []);
  const setError = useCallback((error: string | null) => dispatch({ type: 'SET_ERROR', payload: error }), []);
  const clearSearch = useCallback(() => dispatch({ type: 'CLEAR' }), []);

  // Global keyboard shortcut: Ctrl/Cmd + K
  useEffect(() => {
    if (!isClient) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      // Ctrl+K or Cmd+K
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {
        e.preventDefault();
        if (state.isOpen) {
          closeSearch();
        } else {
          openSearch();
        }
      }

      // Escape to close
      if (e.key === 'Escape' && state.isOpen) {
        closeSearch();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isClient, state.isOpen, openSearch, closeSearch]);

  const value: SearchContextType = {
    state,
    openSearch,
    closeSearch,
    setQuery,
    setResults,
    setLoading,
    setError,
    clearSearch,
  };

  return (
    <SearchContext.Provider value={value}>
      {children}
    </SearchContext.Provider>
  );
}

export function useSearch(): SearchContextType {
  const context = useContext(SearchContext);
  if (!context) {
    throw new Error('useSearch must be used within a SearchProvider');
  }
  return context;
}

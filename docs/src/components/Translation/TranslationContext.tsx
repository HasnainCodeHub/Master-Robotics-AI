/**
 * TranslationContext - Global translation state management.
 *
 * CRITICAL ISOLATION:
 * - Translation is PRESENTATION ONLY
 * - Does NOT affect RAG retrieval or answers
 * - Original English content is ALWAYS preserved
 *
 * @see specs/localization/translation-policy.md
 */

import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  useMemo,
  ReactNode,
} from 'react';
import type { Language, TranslationContextValue } from './types';
import { translateToUrdu } from './api';

/**
 * LocalStorage key for language preference.
 */
const LANGUAGE_STORAGE_KEY = 'textbook_language';

/**
 * SessionStorage key for cached translations.
 */
const TRANSLATION_CACHE_KEY = 'textbook_translation_cache';

/**
 * Default language.
 */
const DEFAULT_LANGUAGE: Language = 'en';

/**
 * Default context value.
 */
const defaultContextValue: TranslationContextValue = {
  language: DEFAULT_LANGUAGE,
  isTranslating: false,
  error: null,
  originalContent: null,
  translatedContent: null,
  toggleLanguage: async () => {},
  resetToEnglish: () => {},
  clearError: () => {},
  translateText: async (text: string) => text,
};

/**
 * Translation context.
 */
const TranslationContext = createContext<TranslationContextValue>(defaultContextValue);

/**
 * Props for TranslationProvider.
 */
interface TranslationProviderProps {
  children: ReactNode;
}

/**
 * Get stored language preference.
 */
function getStoredLanguage(): Language {
  if (typeof window === 'undefined') return DEFAULT_LANGUAGE;
  const stored = localStorage.getItem(LANGUAGE_STORAGE_KEY);
  return stored === 'ur' ? 'ur' : 'en';
}

/**
 * Store language preference.
 */
function storeLanguage(language: Language): void {
  if (typeof window === 'undefined') return;
  localStorage.setItem(LANGUAGE_STORAGE_KEY, language);
}

/**
 * Get cached translation from session storage.
 */
function getCachedTranslation(key: string): string | null {
  if (typeof window === 'undefined') return null;
  try {
    const cache = sessionStorage.getItem(TRANSLATION_CACHE_KEY);
    if (!cache) return null;
    const parsed = JSON.parse(cache);
    return parsed[key] || null;
  } catch {
    return null;
  }
}

/**
 * Cache translation in session storage.
 */
function cacheTranslation(key: string, translation: string): void {
  if (typeof window === 'undefined') return;
  try {
    const existing = sessionStorage.getItem(TRANSLATION_CACHE_KEY);
    const cache = existing ? JSON.parse(existing) : {};
    cache[key] = translation;
    sessionStorage.setItem(TRANSLATION_CACHE_KEY, JSON.stringify(cache));
  } catch {
    // Ignore cache errors
  }
}

/**
 * Generate cache key from text.
 */
function getCacheKey(text: string): string {
  // Simple hash for cache key
  let hash = 0;
  for (let i = 0; i < text.length; i++) {
    const char = text.charCodeAt(i);
    hash = ((hash << 5) - hash) + char;
    hash = hash & hash;
  }
  return `ur_${hash}`;
}

/**
 * TranslationProvider component.
 * Wraps the application and provides translation state to all children.
 */
export function TranslationProvider({ children }: TranslationProviderProps): JSX.Element {
  const [language, setLanguage] = useState<Language>(DEFAULT_LANGUAGE);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);

  /**
   * Initialize language from localStorage on mount.
   */
  useEffect(() => {
    const stored = getStoredLanguage();
    setLanguage(stored);

    // Update document attributes
    updateDocumentLanguage(stored);
  }, []);

  /**
   * Update document language and direction attributes.
   */
  function updateDocumentLanguage(lang: Language): void {
    if (typeof document === 'undefined') return;

    const html = document.documentElement;
    html.setAttribute('lang', lang);
    html.setAttribute('dir', lang === 'ur' ? 'rtl' : 'ltr');
    html.setAttribute('data-language', lang);
  }

  /**
   * Translate text using the API.
   */
  const translateText = useCallback(async (text: string): Promise<string> => {
    if (!text || !text.trim()) {
      return text;
    }

    // Check cache first
    const cacheKey = getCacheKey(text);
    const cached = getCachedTranslation(cacheKey);
    if (cached) {
      return cached;
    }

    try {
      const response = await translateToUrdu(text);
      const translated = response.translated_text;

      // Cache the translation
      cacheTranslation(cacheKey, translated);

      return translated;
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Translation failed';
      throw new Error(message);
    }
  }, []);

  /**
   * Toggle between English and Urdu.
   */
  const toggleLanguage = useCallback(async () => {
    const newLanguage = language === 'en' ? 'ur' : 'en';

    if (newLanguage === 'ur') {
      // Switching to Urdu - need to translate
      setIsTranslating(true);
      setError(null);

      try {
        // Get translatable content from the page
        const content = getTranslatableContent();
        if (content) {
          setOriginalContent(content);
          const translated = await translateText(content);
          setTranslatedContent(translated);
          applyTranslation(translated);
        }

        setLanguage('ur');
        storeLanguage('ur');
        updateDocumentLanguage('ur');
      } catch (err) {
        const message = err instanceof Error ? err.message : 'Translation failed';
        setError(message);
        // Stay in English on error
      } finally {
        setIsTranslating(false);
      }
    } else {
      // Switching to English - instant
      if (originalContent) {
        restoreOriginalContent();
      }
      setLanguage('en');
      storeLanguage('en');
      updateDocumentLanguage('en');
      setTranslatedContent(null);
      setError(null);
    }
  }, [language, translateText, originalContent]);

  /**
   * Reset to English.
   */
  const resetToEnglish = useCallback(() => {
    if (originalContent) {
      restoreOriginalContent();
    }
    setLanguage('en');
    storeLanguage('en');
    updateDocumentLanguage('en');
    setTranslatedContent(null);
    setError(null);
  }, [originalContent]);

  /**
   * Clear error.
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  /**
   * Context value.
   */
  const value = useMemo<TranslationContextValue>(
    () => ({
      language,
      isTranslating,
      error,
      originalContent,
      translatedContent,
      toggleLanguage,
      resetToEnglish,
      clearError,
      translateText,
    }),
    [
      language,
      isTranslating,
      error,
      originalContent,
      translatedContent,
      toggleLanguage,
      resetToEnglish,
      clearError,
      translateText,
    ]
  );

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
}

/**
 * Get translatable content from the page.
 * Targets article content, excluding code blocks.
 */
function getTranslatableContent(): string | null {
  if (typeof document === 'undefined') return null;

  // Target main article content
  const article = document.querySelector('article.markdown, .markdown, [data-translatable="true"]');
  if (!article) return null;

  // Clone to avoid modifying the DOM
  const clone = article.cloneNode(true) as HTMLElement;

  // Remove non-translatable elements
  const nonTranslatable = clone.querySelectorAll(
    'code, pre, [data-translatable="false"], .preserve-term, script, style'
  );
  nonTranslatable.forEach((el) => el.remove());

  return clone.textContent || null;
}

/**
 * Apply translation to the page.
 * This is a simplified implementation - in production, you'd want more sophisticated DOM handling.
 */
function applyTranslation(translatedText: string): void {
  if (typeof document === 'undefined') return;

  // Add translated content indicator
  const article = document.querySelector('article.markdown, .markdown');
  if (article) {
    article.setAttribute('data-translated', 'true');
    article.classList.add('translated-content', 'urdu-text');
  }

  // Note: Full implementation would replace text nodes while preserving structure
  // For this MVP, we rely on CSS to handle RTL and the context provides the translated text
}

/**
 * Restore original English content.
 */
function restoreOriginalContent(): void {
  if (typeof document === 'undefined') return;

  const article = document.querySelector('article.markdown, .markdown');
  if (article) {
    article.removeAttribute('data-translated');
    article.classList.remove('translated-content', 'urdu-text');
  }
}

/**
 * useTranslation hook.
 * Access translation state from any component.
 */
export function useTranslation(): TranslationContextValue {
  return useContext(TranslationContext);
}

export default TranslationContext;

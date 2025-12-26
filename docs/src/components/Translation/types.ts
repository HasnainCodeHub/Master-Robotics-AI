/**
 * Type definitions for translation feature.
 */

export type Language = 'en' | 'ur';

export interface TranslateRequest {
  text: string;
  target_language: 'ur';
  preserve_terms?: string[];
}

export interface TranslateResponse {
  original_text: string;
  translated_text: string;
  target_language: 'ur';
  preserved_terms: string[];
  translation_provider: string;
}

export interface TranslationState {
  /** Current language */
  language: Language;
  /** Is translation in progress */
  isTranslating: boolean;
  /** Translation error if any */
  error: string | null;
  /** Original English content (always available) */
  originalContent: string | null;
  /** Translated content (null if not translated or in English mode) */
  translatedContent: string | null;
}

export interface TranslationContextValue extends TranslationState {
  /** Toggle between English and Urdu */
  toggleLanguage: () => Promise<void>;
  /** Reset to English */
  resetToEnglish: () => void;
  /** Clear error */
  clearError: () => void;
  /** Translate specific text */
  translateText: (text: string) => Promise<string>;
}

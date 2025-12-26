/**
 * Translation module exports.
 */

export { TranslationProvider, useTranslation } from './TranslationContext';
export { TranslationToggle } from './TranslationToggle';
export { LanguageIndicator } from './LanguageIndicator';
export { TranslationErrorToast } from './TranslationErrorToast';
export { translateToUrdu } from './api';
export type {
  Language,
  TranslateRequest,
  TranslateResponse,
  TranslationState,
  TranslationContextValue,
} from './types';

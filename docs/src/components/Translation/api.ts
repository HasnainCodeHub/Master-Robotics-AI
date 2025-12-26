/**
 * Translation API client.
 *
 * ISOLATION RULE: This client only calls /api/translate.
 * It does NOT call /api/chat or any RAG-related endpoints.
 *
 * BROWSER-SAFE: No process.env usage. Uses getApiBaseUrl() utility.
 */

import type { TranslateRequest, TranslateResponse } from './types';
import { getApiBaseUrl } from '../../utils/config';

/**
 * Get API base URL lazily (not at module import time).
 * This prevents SSR crashes and allows runtime configuration.
 */
function getBaseUrl(): string {
  return getApiBaseUrl();
}

/**
 * Translate text to Urdu.
 *
 * @param text - Text to translate
 * @param preserveTerms - Additional terms to preserve in English
 * @returns Translated text response
 * @throws Error if translation fails
 */
export async function translateToUrdu(
  text: string,
  preserveTerms?: string[],
): Promise<TranslateResponse> {
  const request: TranslateRequest = {
    text,
    target_language: 'ur',
    preserve_terms: preserveTerms,
  };

  const response = await fetch(`${getBaseUrl()}/api/translate`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail?.error || 'Translation failed');
  }

  return response.json();
}

/**
 * Check if translation service is available.
 */
export async function checkTranslationService(): Promise<boolean> {
  try {
    const response = await fetch(`${getBaseUrl()}/health`, {
      method: 'GET',
    });
    return response.ok;
  } catch {
    return false;
  }
}

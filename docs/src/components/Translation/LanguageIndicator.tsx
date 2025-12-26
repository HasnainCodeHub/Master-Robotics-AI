/**
 * LanguageIndicator - Shows current language state.
 *
 * Displays a badge indicating whether content is in English or Urdu.
 */

import React from 'react';
import { useTranslation } from './TranslationContext';
import styles from './styles.module.css';

/**
 * Props for LanguageIndicator.
 */
interface LanguageIndicatorProps {
  /** Show full name or abbreviation */
  showFullName?: boolean;
  /** Custom class name */
  className?: string;
}

/**
 * LanguageIndicator component.
 */
export function LanguageIndicator({
  showFullName = false,
  className = '',
}: LanguageIndicatorProps): JSX.Element {
  const { language, isTranslating } = useTranslation();

  const label = language === 'en'
    ? (showFullName ? 'English' : 'EN')
    : (showFullName ? 'اُردُو' : 'UR');

  return (
    <span
      className={`${styles.languageIndicator} ${styles[language]} ${className}`}
      aria-label={`Current language: ${language === 'en' ? 'English' : 'Urdu'}`}
    >
      {isTranslating ? (
        <span className={styles.loadingSpinnerSmall} />
      ) : (
        label
      )}
    </span>
  );
}

export default LanguageIndicator;

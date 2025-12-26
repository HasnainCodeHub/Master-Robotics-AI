/**
 * TranslationToggle - Language toggle button component.
 *
 * Displays a segmented control to switch between English and Urdu.
 * Can be rendered in header or FAB variants.
 *
 * @see specs/localization/translation-ui.md
 */

import React, { useCallback } from 'react';
import { useTranslation } from './TranslationContext';
import styles from './styles.module.css';

/**
 * Props for TranslationToggle.
 */
interface TranslationToggleProps {
  /** Visual variant */
  variant?: 'header' | 'fab';
  /** Custom class name */
  className?: string;
}

/**
 * TranslationToggle component.
 */
export function TranslationToggle({
  variant = 'header',
  className = '',
}: TranslationToggleProps): JSX.Element {
  const { language, isTranslating, error, toggleLanguage, clearError } = useTranslation();

  const handleToggle = useCallback(async () => {
    clearError();
    await toggleLanguage();
  }, [toggleLanguage, clearError]);

  const handleKeyDown = useCallback(
    (event: React.KeyboardEvent) => {
      if (event.key === 'Enter' || event.key === ' ') {
        event.preventDefault();
        handleToggle();
      }
    },
    [handleToggle]
  );

  if (variant === 'fab') {
    return (
      <button
        type="button"
        className={`${styles.translationFab} ${className}`}
        onClick={handleToggle}
        onKeyDown={handleKeyDown}
        disabled={isTranslating}
        aria-label={language === 'en' ? 'Switch to Urdu' : 'Switch to English'}
        title={language === 'en' ? 'Switch to Urdu' : 'Switch to English'}
      >
        <span className={styles.fabIcon} aria-hidden="true">
          {isTranslating ? (
            <span className={styles.loadingSpinner} />
          ) : (
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="10" />
              <line x1="2" y1="12" x2="22" y2="12" />
              <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
            </svg>
          )}
        </span>
        <span className={styles.fabLabel}>{language.toUpperCase()}</span>
      </button>
    );
  }

  // Header variant - segmented control
  return (
    <div
      className={`${styles.translationToggle} ${className} ${error ? styles.hasError : ''}`}
      role="group"
      aria-label="Language selection"
    >
      <button
        type="button"
        role="radio"
        aria-checked={language === 'en'}
        className={`${styles.toggleButton} ${language === 'en' ? styles.active : ''}`}
        onClick={language !== 'en' ? handleToggle : undefined}
        onKeyDown={handleKeyDown}
        disabled={isTranslating}
      >
        EN
      </button>
      <button
        type="button"
        role="radio"
        aria-checked={language === 'ur'}
        className={`${styles.toggleButton} ${language === 'ur' ? styles.active : ''}`}
        onClick={language !== 'ur' ? handleToggle : undefined}
        onKeyDown={handleKeyDown}
        disabled={isTranslating}
      >
        {isTranslating && language === 'en' ? (
          <span className={styles.loadingSpinnerSmall} />
        ) : (
          'اُردُو'
        )}
      </button>
    </div>
  );
}

export default TranslationToggle;

/**
 * TranslationErrorToast - Error display for translation failures.
 *
 * Shows a dismissible error message when translation fails.
 */

import React, { useEffect, useCallback } from 'react';
import { useTranslation } from './TranslationContext';
import styles from './styles.module.css';

/**
 * Props for TranslationErrorToast.
 */
interface TranslationErrorToastProps {
  /** Auto-dismiss after N milliseconds (0 to disable) */
  autoDismissMs?: number;
}

/**
 * TranslationErrorToast component.
 */
export function TranslationErrorToast({
  autoDismissMs = 5000,
}: TranslationErrorToastProps): JSX.Element | null {
  const { error, clearError, toggleLanguage } = useTranslation();

  // Auto-dismiss
  useEffect(() => {
    if (error && autoDismissMs > 0) {
      const timer = setTimeout(() => {
        clearError();
      }, autoDismissMs);
      return () => clearTimeout(timer);
    }
  }, [error, autoDismissMs, clearError]);

  const handleRetry = useCallback(async () => {
    clearError();
    await toggleLanguage();
  }, [clearError, toggleLanguage]);

  const handleDismiss = useCallback(() => {
    clearError();
  }, [clearError]);

  const handleKeyDown = useCallback(
    (event: React.KeyboardEvent) => {
      if (event.key === 'Escape') {
        handleDismiss();
      }
    },
    [handleDismiss]
  );

  if (!error) {
    return null;
  }

  return (
    <div
      className={styles.errorToast}
      role="alert"
      aria-live="polite"
      onKeyDown={handleKeyDown}
    >
      <div className={styles.errorToastContent}>
        <svg
          className={styles.errorIcon}
          width="20"
          height="20"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <circle cx="12" cy="12" r="10" />
          <line x1="12" y1="8" x2="12" y2="12" />
          <line x1="12" y1="16" x2="12.01" y2="16" />
        </svg>
        <div className={styles.errorMessage}>
          <strong>Translation unavailable</strong>
          <span>Showing original English content.</span>
        </div>
      </div>
      <div className={styles.errorToastActions}>
        <button
          type="button"
          className={styles.retryButton}
          onClick={handleRetry}
        >
          Retry
        </button>
        <button
          type="button"
          className={styles.dismissButton}
          onClick={handleDismiss}
          aria-label="Dismiss"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        </button>
      </div>
    </div>
  );
}

export default TranslationErrorToast;

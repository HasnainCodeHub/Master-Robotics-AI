/**
 * BookAccessGuard - Blocks book content for unauthenticated users.
 *
 * SECURITY: This is a UI-level guard only.
 * Backend authentication is the source of truth.
 */

import React, { ReactNode, useCallback } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '../AuthContext';
import styles from './guards.module.css';

// Key for storing redirect URL
const REDIRECT_URL_KEY = 'auth_redirect_url';

interface BookAccessGuardProps {
  children: ReactNode;
}

/**
 * Store the current URL for redirect after login.
 */
function storeRedirectUrl(): void {
  if (typeof window !== 'undefined') {
    sessionStorage.setItem(REDIRECT_URL_KEY, window.location.href);
  }
}

/**
 * Get and clear the stored redirect URL.
 */
export function getAndClearRedirectUrl(): string | null {
  if (typeof window === 'undefined') return null;
  const url = sessionStorage.getItem(REDIRECT_URL_KEY);
  sessionStorage.removeItem(REDIRECT_URL_KEY);
  return url;
}

/**
 * BookAccessGuard component.
 * Shows login prompt instead of book content when not authenticated.
 */
export function BookAccessGuard({ children }: BookAccessGuardProps): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();

  const handleLoginClick = useCallback(() => {
    // Store current URL for redirect after login
    storeRedirectUrl();
  }, []);

  // Show loading state while checking auth
  if (isLoading) {
    return (
      <div className={styles.guardContainer}>
        <div className={styles.loadingState}>
          <div className={styles.spinner} />
          <p>Loading...</p>
        </div>
      </div>
    );
  }

  // Show login prompt if not authenticated
  if (!isAuthenticated) {
    return (
      <div className={styles.guardContainer}>
        <div className={styles.accessDenied}>
          <div className={styles.lockIcon}>
            <svg
              width="64"
              height="64"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="1.5"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <rect x="3" y="11" width="18" height="11" rx="2" ry="2" />
              <path d="M7 11V7a5 5 0 0 1 10 0v4" />
            </svg>
          </div>
          <h2 className={styles.title}>Authentication Required</h2>
          <p className={styles.message}>
            Please login to access the textbook content.
          </p>
          <p className={styles.subMessage}>
            Create a free account to get personalized learning and access to the AI tutor.
          </p>
          <div className={styles.actions}>
            <Link
              to="/login"
              className={styles.primaryButton}
              onClick={handleLoginClick}
            >
              Login / Sign Up
            </Link>
          </div>
        </div>
      </div>
    );
  }

  // User is authenticated - render children
  return <>{children}</>;
}

export default BookAccessGuard;

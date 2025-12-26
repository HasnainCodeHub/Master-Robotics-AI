/**
 * Login Page - Full-page authentication route
 *
 * Professional full-screen login page with centered card layout.
 * Replaces modal-based authentication for better UX.
 */

import React, { useState, useCallback, useEffect, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import Head from '@docusaurus/Head';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/components/Auth/AuthContext';
import { getAndClearRedirectUrl } from '@site/src/components/Auth/guards';
import styles from './auth.module.css';

export default function LoginPage(): JSX.Element {
  const history = useHistory();
  const { login, isAuthenticated, error, clearError, isLoading } = useAuth();
  const firstInputRef = useRef<HTMLInputElement>(null);

  // Form state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [rememberMe, setRememberMe] = useState(false);
  const [localError, setLocalError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  // Redirect if already authenticated
  useEffect(() => {
    if (isAuthenticated) {
      const redirectUrl = getAndClearRedirectUrl();
      let targetPath = '/robotics-book/textbook/intro'; // default

      if (redirectUrl) {
        try {
          const url = new URL(redirectUrl);
          // Security: only allow same-origin redirects
          if (url.origin === window.location.origin) {
            targetPath = url.pathname; // Extract pathname only
          } else {
            console.warn('Cross-origin redirect blocked:', redirectUrl);
          }
        } catch (err) {
          console.warn('Invalid redirect URL, using default:', err);
        }
      }

      history.push(targetPath);
    }
  }, [isAuthenticated, history]);

  // Focus first input on mount
  useEffect(() => {
    firstInputRef.current?.focus();
  }, []);

  // Clear errors when form changes
  useEffect(() => {
    clearError();
    setLocalError(null);
  }, [email, password, clearError]);

  /**
   * Handle form submission
   */
  const handleSubmit = useCallback(
    async (event: React.FormEvent) => {
      event.preventDefault();
      setLocalError(null);

      if (!email || !password) {
        setLocalError('Please fill in all fields');
        return;
      }

      try {
        await login(email, password, rememberMe);
        setSuccessMessage('Successfully signed in!');

        // Redirect after success
        setTimeout(() => {
          const redirectUrl = getAndClearRedirectUrl();
          let targetPath = '/robotics-book/textbook/intro'; // default

          if (redirectUrl) {
            try {
              const url = new URL(redirectUrl);
              // Security: only allow same-origin redirects
              if (url.origin === window.location.origin) {
                targetPath = url.pathname; // Extract pathname only
              } else {
                console.warn('Cross-origin redirect blocked:', redirectUrl);
              }
            } catch (err) {
              console.warn('Invalid redirect URL, using default:', err);
            }
          }

          console.log('Redirecting to:', targetPath);
          history.push(targetPath);
        }, 500);
      } catch {
        // Error is handled by AuthContext
      }
    },
    [email, password, rememberMe, login, history]
  );

  const displayError = localError || error;

  return (
    <Layout title="Sign In" description="Sign in to access the Physical AI & Humanoid Robotics textbook">
      <Head>
        <title>Sign In | Physical AI & Humanoid Robotics</title>
      </Head>

      <main className={styles.authPage}>
        {/* Background gradients */}
        <div className={styles.authBackground}>
          <div className={styles.authGradient1} />
          <div className={styles.authGradient2} />
          <div className={styles.authGrid} />
        </div>

        <div className={styles.authContainer}>
          {/* Branding */}
          <div className={styles.authBranding}>
            <Link to="/" className={styles.authLogo}>
              <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <circle cx="12" cy="12" r="3" />
                <path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42" />
              </svg>
              <span>Physical AI & Humanoid Robotics</span>
            </Link>
          </div>

          {/* Auth Card */}
          <div className={styles.authCard}>
            {/* Header */}
            <div className={styles.authHeader}>
              <h1 className={styles.authTitle}>Welcome Back</h1>
              <p className={styles.authSubtitle}>
                Sign in to continue your learning journey
              </p>
            </div>

            {/* Error/Success Messages */}
            {displayError && (
              <div className={styles.errorBanner} role="alert">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <circle cx="12" cy="12" r="10" />
                  <line x1="12" y1="8" x2="12" y2="12" />
                  <line x1="12" y1="16" x2="12.01" y2="16" />
                </svg>
                <span>{displayError}</span>
              </div>
            )}

            {successMessage && (
              <div className={styles.successBanner} role="status">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
                  <polyline points="22 4 12 14.01 9 11.01" />
                </svg>
                <span>{successMessage}</span>
              </div>
            )}

            {/* Login Form */}
            <form onSubmit={handleSubmit} className={styles.authForm}>
              <div className={styles.formGroup}>
                <label htmlFor="email" className={styles.label}>
                  Email
                </label>
                <input
                  ref={firstInputRef}
                  id="email"
                  type="email"
                  className={styles.input}
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  placeholder="you@example.com"
                  autoComplete="email"
                  disabled={isLoading}
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password" className={styles.label}>
                  Password
                </label>
                <input
                  id="password"
                  type="password"
                  className={styles.input}
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="Enter your password"
                  autoComplete="current-password"
                  disabled={isLoading}
                  required
                />
              </div>

              <div className={styles.formRow}>
                <div className={styles.checkboxGroup}>
                  <input
                    id="remember-me"
                    type="checkbox"
                    className={styles.checkbox}
                    checked={rememberMe}
                    onChange={(e) => setRememberMe(e.target.checked)}
                    disabled={isLoading}
                  />
                  <label htmlFor="remember-me" className={styles.checkboxLabel}>
                    Remember me
                  </label>
                </div>
              </div>

              <button
                type="submit"
                className={styles.submitButton}
                disabled={isLoading}
              >
                {isLoading ? (
                  <span className={styles.loadingSpinner} />
                ) : (
                  <>
                    <span>Sign In</span>
                    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                      <path d="M5 12h14M12 5l7 7-7 7" />
                    </svg>
                  </>
                )}
              </button>
            </form>

            {/* Footer Links */}
            <div className={styles.authFooter}>
              <p className={styles.authFooterText}>
                Don't have an account?{' '}
                <Link to="/signup" className={styles.authLink}>
                  Sign up
                </Link>
              </p>
              <Link to="/" className={styles.authBackLink}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M19 12H5M12 19l-7-7 7-7" />
                </svg>
                Back to homepage
              </Link>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

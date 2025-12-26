/**
 * Signup Page - Full-page registration route
 *
 * Professional full-screen signup page with centered card layout.
 * Includes profile fields for personalization.
 */

import React, { useState, useCallback, useEffect, useRef } from 'react';
import { useHistory } from '@docusaurus/router';
import Head from '@docusaurus/Head';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/components/Auth/AuthContext';
import type { ExperienceLevel, HardwareAccess } from '@site/src/components/Auth/types';
import styles from './auth.module.css';

/**
 * Dropdown options.
 */
const SOFTWARE_LEVELS: { value: ExperienceLevel; label: string }[] = [
  { value: 'beginner', label: 'Beginner' },
  { value: 'intermediate', label: 'Intermediate' },
  { value: 'advanced', label: 'Advanced' },
];

const ROBOTICS_LEVELS: { value: ExperienceLevel; label: string }[] = [
  { value: 'beginner', label: 'Beginner' },
  { value: 'intermediate', label: 'Intermediate' },
  { value: 'advanced', label: 'Advanced' },
];

const HARDWARE_OPTIONS: { value: HardwareAccess; label: string }[] = [
  { value: 'simulation_only', label: 'Simulation Only' },
  { value: 'jetson_device', label: 'Jetson Device' },
  { value: 'physical_robot', label: 'Physical Robot' },
];

export default function SignupPage(): JSX.Element {
  const history = useHistory();
  const { signup, isAuthenticated, error, clearError, isLoading } = useAuth();
  const firstInputRef = useRef<HTMLInputElement>(null);

  // Form state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState<ExperienceLevel>('beginner');
  const [roboticsLevel, setRoboticsLevel] = useState<ExperienceLevel>('beginner');
  const [hardwareAccess, setHardwareAccess] = useState<HardwareAccess>('simulation_only');

  // Local messages
  const [localError, setLocalError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  // Redirect if already authenticated
  useEffect(() => {
    if (isAuthenticated) {
      const redirectUrl = getAndClearRedirectUrl();
      let targetPath = 'robotics-book/textbook/intro'; // default

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
  }, [email, password, confirmPassword, clearError]);

  /**
   * Handle form submission
   */
  const handleSubmit = useCallback(
    async (event: React.FormEvent) => {
      event.preventDefault();
      setLocalError(null);

      if (!email || !password || !confirmPassword) {
        setLocalError('Please fill in all required fields');
        return;
      }

      if (password.length < 8) {
        setLocalError('Password must be at least 8 characters');
        return;
      }

      // Validate password has at least one letter and one number
      if (!/[a-zA-Z]/.test(password)) {
        setLocalError('Password must contain at least one letter');
        return;
      }

      if (!/[0-9]/.test(password)) {
        setLocalError('Password must contain at least one number');
        return;
      }

      if (password !== confirmPassword) {
        setLocalError('Passwords do not match');
        return;
      }

      try {
        await signup(email, password, {
          software_level: softwareLevel,
          robotics_level: roboticsLevel,
          hardware_access: hardwareAccess,
        });

        // Success! Show message and redirect to final destination
        setSuccessMessage('Account created successfully! Redirecting...');
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
        }, 1500);
      } catch {
        // Error is handled by AuthContext
      }
    },
    [
      email,
      password,
      confirmPassword,
      softwareLevel,
      roboticsLevel,
      hardwareAccess,
      signup,
      history,
    ]
  );

  const displayError = localError || error;

  return (
    <Layout title="Sign Up" description="Create an account to access the Physical AI & Humanoid Robotics textbook">
      <Head>
        <title>Sign Up | Physical AI & Humanoid Robotics</title>
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
              <h1 className={styles.authTitle}>Create Your Account</h1>
              <p className={styles.authSubtitle}>
                Get personalized access to the interactive textbook
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

            {/* Signup Form */}
            <form onSubmit={handleSubmit} className={styles.authForm}>
              <div className={styles.formGroup}>
                <label htmlFor="email" className={styles.label}>
                  Email <span className={styles.required}>*</span>
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
                  Password <span className={styles.required}>*</span>
                </label>
                <input
                  id="password"
                  type="password"
                  className={styles.input}
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="At least 8 characters"
                  autoComplete="new-password"
                  disabled={isLoading}
                  minLength={8}
                  required
                />
                <p className={styles.hint}>Must contain at least one letter and one number</p>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="confirm-password" className={styles.label}>
                  Confirm Password <span className={styles.required}>*</span>
                </label>
                <input
                  id="confirm-password"
                  type="password"
                  className={styles.input}
                  value={confirmPassword}
                  onChange={(e) => setConfirmPassword(e.target.value)}
                  placeholder="Re-enter your password"
                  autoComplete="new-password"
                  disabled={isLoading}
                  required
                />
              </div>

              <div className={styles.formDivider}>
                <span>Your Background</span>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="software-level" className={styles.label}>
                  Software Experience
                </label>
                <select
                  id="software-level"
                  className={styles.select}
                  value={softwareLevel}
                  onChange={(e) => setSoftwareLevel(e.target.value as ExperienceLevel)}
                  disabled={isLoading}
                >
                  {SOFTWARE_LEVELS.map((option) => (
                    <option key={option.value} value={option.value}>
                      {option.label}
                    </option>
                  ))}
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="robotics-level" className={styles.label}>
                  Robotics Experience
                </label>
                <select
                  id="robotics-level"
                  className={styles.select}
                  value={roboticsLevel}
                  onChange={(e) => setRoboticsLevel(e.target.value as ExperienceLevel)}
                  disabled={isLoading}
                >
                  {ROBOTICS_LEVELS.map((option) => (
                    <option key={option.value} value={option.value}>
                      {option.label}
                    </option>
                  ))}
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="hardware-access" className={styles.label}>
                  Hardware Access
                </label>
                <select
                  id="hardware-access"
                  className={styles.select}
                  value={hardwareAccess}
                  onChange={(e) => setHardwareAccess(e.target.value as HardwareAccess)}
                  disabled={isLoading}
                >
                  {HARDWARE_OPTIONS.map((option) => (
                    <option key={option.value} value={option.value}>
                      {option.label}
                    </option>
                  ))}
                </select>
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
                    <span>Create Account</span>
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
                Already have an account?{' '}
                <Link to="/login" className={styles.authLink}>
                  Sign in
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

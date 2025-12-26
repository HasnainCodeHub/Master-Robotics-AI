/**
 * AuthModal - Combined signup/signin modal with tab interface.
 *
 * Features:
 * - Tab-based interface (Sign In / Sign Up)
 * - Form validation
 * - Loading states during API calls
 * - Error/success messaging
 * - Profile fields for signup (software level, robotics level, hardware access)
 * - Keyboard accessible (Tab, Enter, ESC to close)
 */

import React, { useState, useCallback, useEffect, useRef } from 'react';
import { useAuth } from './AuthContext';
import { getAndClearRedirectUrl } from './guards';
import type { ExperienceLevel, HardwareAccess } from './types';
import styles from './styles.module.css';

/**
 * Tab types.
 */
type AuthTab = 'signin' | 'signup';

/**
 * Props for AuthModal.
 */
interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  initialTab?: AuthTab;
}

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

/**
 * AuthModal component.
 */
export function AuthModal({
  isOpen,
  onClose,
  initialTab = 'signin',
}: AuthModalProps): JSX.Element | null {
  const { login, signup, error, clearError, isLoading } = useAuth();
  const modalRef = useRef<HTMLDivElement>(null);
  const firstInputRef = useRef<HTMLInputElement>(null);

  // Tab state
  const [activeTab, setActiveTab] = useState<AuthTab>(initialTab);

  // Sign In form state
  const [signinEmail, setSigninEmail] = useState('');
  const [signinPassword, setSigninPassword] = useState('');
  const [rememberMe, setRememberMe] = useState(false);

  // Sign Up form state
  const [signupEmail, setSignupEmail] = useState('');
  const [signupPassword, setSignupPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState<ExperienceLevel>('beginner');
  const [roboticsLevel, setRoboticsLevel] = useState<ExperienceLevel>('beginner');
  const [hardwareAccess, setHardwareAccess] = useState<HardwareAccess>('simulation_only');

  // Local error for password mismatch
  const [localError, setLocalError] = useState<string | null>(null);

  // Success message
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  /**
   * Reset form state when modal opens/closes or tab changes.
   */
  useEffect(() => {
    if (isOpen) {
      clearError();
      setLocalError(null);
      setSuccessMessage(null);
      // Focus first input when modal opens
      setTimeout(() => {
        firstInputRef.current?.focus();
      }, 100);
    }
  }, [isOpen, clearError]);

  useEffect(() => {
    clearError();
    setLocalError(null);
    setSuccessMessage(null);
  }, [activeTab, clearError]);

  /**
   * Handle ESC key to close modal.
   */
  useEffect(() => {
    function handleKeyDown(event: KeyboardEvent) {
      if (event.key === 'Escape' && isOpen) {
        onClose();
      }
    }

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  /**
   * Trap focus within modal.
   */
  useEffect(() => {
    if (!isOpen || !modalRef.current) return;

    const modal = modalRef.current;
    const focusableElements = modal.querySelectorAll<HTMLElement>(
      'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
    );
    const firstElement = focusableElements[0];
    const lastElement = focusableElements[focusableElements.length - 1];

    function handleTabKey(event: KeyboardEvent) {
      if (event.key !== 'Tab') return;

      if (event.shiftKey) {
        if (document.activeElement === firstElement) {
          event.preventDefault();
          lastElement?.focus();
        }
      } else {
        if (document.activeElement === lastElement) {
          event.preventDefault();
          firstElement?.focus();
        }
      }
    }

    document.addEventListener('keydown', handleTabKey);
    return () => document.removeEventListener('keydown', handleTabKey);
  }, [isOpen]);

  /**
   * Handle backdrop click.
   */
  const handleBackdropClick = useCallback(
    (event: React.MouseEvent) => {
      if (event.target === event.currentTarget) {
        onClose();
      }
    },
    [onClose]
  );

  /**
   * Handle redirect after successful authentication.
   */
  const handleAuthSuccess = useCallback(() => {
    const redirectUrl = getAndClearRedirectUrl();
    onClose();

    // If there's a stored redirect URL and it's on the same origin, navigate to it
    if (redirectUrl && typeof window !== 'undefined') {
      try {
        const url = new URL(redirectUrl);
        if (url.origin === window.location.origin) {
          // Navigate to the stored URL
          window.location.href = redirectUrl;
          return;
        }
      } catch {
        // Invalid URL, ignore
      }
    }
  }, [onClose]);

  /**
   * Handle sign in form submission.
   */
  const handleSignIn = useCallback(
    async (event: React.FormEvent) => {
      event.preventDefault();
      setLocalError(null);

      if (!signinEmail || !signinPassword) {
        setLocalError('Please fill in all fields');
        return;
      }

      try {
        await login(signinEmail, signinPassword, rememberMe);
        setSuccessMessage('Successfully signed in!');
        setTimeout(() => {
          handleAuthSuccess();
        }, 500);
      } catch {
        // Error is handled by AuthContext
      }
    },
    [signinEmail, signinPassword, rememberMe, login, handleAuthSuccess]
  );

  /**
   * Handle sign up form submission.
   */
  const handleSignUp = useCallback(
    async (event: React.FormEvent) => {
      event.preventDefault();
      setLocalError(null);

      if (!signupEmail || !signupPassword || !confirmPassword) {
        setLocalError('Please fill in all required fields');
        return;
      }

      if (signupPassword.length < 8) {
        setLocalError('Password must be at least 8 characters');
        return;
      }

      // Validate password has at least one letter and one number
      if (!/[a-zA-Z]/.test(signupPassword)) {
        setLocalError('Password must contain at least one letter');
        return;
      }

      if (!/[0-9]/.test(signupPassword)) {
        setLocalError('Password must contain at least one number');
        return;
      }

      if (signupPassword !== confirmPassword) {
        setLocalError('Passwords do not match');
        return;
      }

      try {
        await signup(signupEmail, signupPassword, {
          software_level: softwareLevel,
          robotics_level: roboticsLevel,
          hardware_access: hardwareAccess,
        });
        // Account created - switch to signin tab
        setSuccessMessage('Account created! Please sign in with your credentials.');
        // Pre-fill signin email with the signup email
        setSigninEmail(signupEmail);
        // Clear signup form
        setSignupEmail('');
        setSignupPassword('');
        setConfirmPassword('');
        // Switch to signin tab after brief delay
        setTimeout(() => {
          setActiveTab('signin');
        }, 1000);
      } catch {
        // Error is handled by AuthContext
      }
    },
    [
      signupEmail,
      signupPassword,
      confirmPassword,
      softwareLevel,
      roboticsLevel,
      hardwareAccess,
      signup,
    ]
  );

  // Don't render if not open
  if (!isOpen) {
    return null;
  }

  const displayError = localError || error;

  return (
    <div
      className={styles.modalOverlay}
      onClick={handleBackdropClick}
      role="dialog"
      aria-modal="true"
      aria-labelledby="auth-modal-title"
    >
      <div className={styles.modalContainer} ref={modalRef}>
        {/* Header */}
        <div className={styles.modalHeader}>
          <h2 id="auth-modal-title" className={styles.modalTitle}>
            {activeTab === 'signin' ? 'Sign In' : 'Create Account'}
          </h2>
          <button
            type="button"
            className={styles.modalCloseButton}
            onClick={onClose}
            aria-label="Close modal"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>

        {/* Tabs */}
        <div className={styles.tabContainer} role="tablist">
          <button
            type="button"
            role="tab"
            aria-selected={activeTab === 'signin'}
            aria-controls="signin-panel"
            className={`${styles.tab} ${activeTab === 'signin' ? styles.tabActive : ''}`}
            onClick={() => setActiveTab('signin')}
          >
            Sign In
          </button>
          <button
            type="button"
            role="tab"
            aria-selected={activeTab === 'signup'}
            aria-controls="signup-panel"
            className={`${styles.tab} ${activeTab === 'signup' ? styles.tabActive : ''}`}
            onClick={() => setActiveTab('signup')}
          >
            Sign Up
          </button>
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

        {/* Sign In Panel */}
        {activeTab === 'signin' && (
          <form
            id="signin-panel"
            role="tabpanel"
            aria-labelledby="signin-tab"
            className={styles.formPanel}
            onSubmit={handleSignIn}
          >
            <div className={styles.formGroup}>
              <label htmlFor="signin-email" className={styles.label}>
                Email
              </label>
              <input
                ref={firstInputRef}
                id="signin-email"
                type="email"
                className={styles.input}
                value={signinEmail}
                onChange={(e) => setSigninEmail(e.target.value)}
                placeholder="you@example.com"
                autoComplete="email"
                disabled={isLoading}
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="signin-password" className={styles.label}>
                Password
              </label>
              <input
                id="signin-password"
                type="password"
                className={styles.input}
                value={signinPassword}
                onChange={(e) => setSigninPassword(e.target.value)}
                placeholder="Enter your password"
                autoComplete="current-password"
                disabled={isLoading}
                required
              />
            </div>

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

            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading ? (
                <span className={styles.loadingSpinner} />
              ) : (
                'Sign In'
              )}
            </button>
          </form>
        )}

        {/* Sign Up Panel */}
        {activeTab === 'signup' && (
          <form
            id="signup-panel"
            role="tabpanel"
            aria-labelledby="signup-tab"
            className={styles.formPanel}
            onSubmit={handleSignUp}
          >
            <div className={styles.formGroup}>
              <label htmlFor="signup-email" className={styles.label}>
                Email <span className={styles.required}>*</span>
              </label>
              <input
                ref={activeTab === 'signup' ? firstInputRef : undefined}
                id="signup-email"
                type="email"
                className={styles.input}
                value={signupEmail}
                onChange={(e) => setSignupEmail(e.target.value)}
                placeholder="you@example.com"
                autoComplete="email"
                disabled={isLoading}
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="signup-password" className={styles.label}>
                Password <span className={styles.required}>*</span>
              </label>
              <input
                id="signup-password"
                type="password"
                className={styles.input}
                value={signupPassword}
                onChange={(e) => setSignupPassword(e.target.value)}
                placeholder="At least 8 characters"
                autoComplete="new-password"
                disabled={isLoading}
                minLength={8}
                required
              />
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
                'Create Account'
              )}
            </button>
          </form>
        )}
      </div>
    </div>
  );
}

export default AuthModal;

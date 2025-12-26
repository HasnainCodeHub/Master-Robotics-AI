/**
 * UserMenu - User dropdown in navigation.
 *
 * Features:
 * - Shows "Sign In" button when not authenticated
 * - Shows user email dropdown when authenticated
 * - Dropdown options: Profile Settings, Sign Out
 * - Clicking "Sign In" navigates to /login page
 * - Keyboard accessible
 */

import React, { useState, useCallback, useRef, useEffect } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from './AuthContext';
import { ProfileSettings } from './ProfileSettings';
import { ConfirmDialog } from './ConfirmDialog';
import { Toast } from './Toast';
import styles from './styles.module.css';

/**
 * UserMenu component.
 */
export function UserMenu(): JSX.Element {
  const { user, isAuthenticated, isLoading, logout } = useAuth();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [isProfileOpen, setIsProfileOpen] = useState(false);
  const [isConfirmSignoutOpen, setIsConfirmSignoutOpen] = useState(false);
  const [isSigningOut, setIsSigningOut] = useState(false);
  const [showSuccessToast, setShowSuccessToast] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  /**
   * Close dropdown when clicking outside.
   */
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsDropdownOpen(false);
      }
    }

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  /**
   * Handle keyboard navigation.
   */
  useEffect(() => {
    function handleKeyDown(event: KeyboardEvent) {
      if (!isDropdownOpen) return;

      if (event.key === 'Escape') {
        setIsDropdownOpen(false);
      }
    }

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isDropdownOpen]);

  /**
   * Toggle dropdown.
   */
  const toggleDropdown = useCallback(() => {
    setIsDropdownOpen((prev) => !prev);
  }, []);

  /**
   * Open profile settings.
   */
  const openProfile = useCallback(() => {
    setIsProfileOpen(true);
    setIsDropdownOpen(false);
  }, []);

  /**
   * Close profile settings.
   */
  const closeProfile = useCallback(() => {
    setIsProfileOpen(false);
  }, []);

  /**
   * Open sign-out confirmation dialog.
   */
  const handleSignOutClick = useCallback(() => {
    setIsDropdownOpen(false);
    setIsConfirmSignoutOpen(true);
  }, []);

  /**
   * Cancel sign-out.
   */
  const handleCancelSignout = useCallback(() => {
    setIsConfirmSignoutOpen(false);
  }, []);

  /**
   * Confirm and execute sign-out.
   */
  const handleConfirmSignout = useCallback(async () => {
    setIsSigningOut(true);
    try {
      await logout();
      setIsConfirmSignoutOpen(false);
      setIsSigningOut(false);
      // Show success toast
      setShowSuccessToast(true);
    } catch (error) {
      console.error('Sign out failed:', error);
      setIsSigningOut(false);
      setIsConfirmSignoutOpen(false);
    }
  }, [logout]);

  /**
   * Close success toast.
   */
  const handleCloseToast = useCallback(() => {
    setShowSuccessToast(false);
  }, []);

  /**
   * Get user initials for avatar.
   */
  const getUserInitials = (): string => {
    if (!user?.email) return '?';
    return user.email.charAt(0).toUpperCase();
  };

  /**
   * Truncate email for display.
   */
  const getDisplayEmail = (): string => {
    if (!user?.email) return '';
    if (user.email.length <= 24) return user.email;
    return user.email.substring(0, 21) + '...';
  };

  // Show loading state
  if (isLoading) {
    return (
      <div className={styles.userMenuContainer}>
        <div className={styles.userMenuLoading}>
          <span className={styles.loadingSpinner} />
        </div>
      </div>
    );
  }

  // Not authenticated - show sign in button
  if (!isAuthenticated) {
    return (
      <div className={styles.userMenuContainer}>
        <Link
          to="/login"
          className={styles.signInButton}
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M15 3h4a2 2 0 0 1 2 2v14a2 2 0 0 1-2 2h-4" />
            <polyline points="10 17 15 12 10 7" />
            <line x1="15" y1="12" x2="3" y2="12" />
          </svg>
          Sign In
        </Link>
      </div>
    );
  }

  // Authenticated - show user dropdown
  return (
    <div className={styles.userMenuContainer} ref={dropdownRef}>
      <button
        type="button"
        className={styles.userMenuButton}
        onClick={toggleDropdown}
        aria-expanded={isDropdownOpen}
        aria-haspopup="true"
      >
        <span className={styles.userAvatar}>{getUserInitials()}</span>
        <span className={styles.userEmail}>{getDisplayEmail()}</span>
        <svg
          className={`${styles.dropdownArrow} ${isDropdownOpen ? styles.dropdownArrowOpen : ''}`}
          width="12"
          height="12"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <polyline points="6 9 12 15 18 9" />
        </svg>
      </button>

      {isDropdownOpen && (
        <div className={styles.dropdown} role="menu">
          <button
            type="button"
            className={styles.dropdownItem}
            onClick={openProfile}
            role="menuitem"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
              <circle cx="12" cy="7" r="4" />
            </svg>
            Profile Settings
          </button>
          <div className={styles.dropdownDivider} />
          <button
            type="button"
            className={styles.dropdownItem}
            onClick={handleSignOutClick}
            role="menuitem"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
              <polyline points="16 17 21 12 16 7" />
              <line x1="21" y1="12" x2="9" y2="12" />
            </svg>
            Sign Out
          </button>
        </div>
      )}

      {/* Profile Settings Modal */}
      <ProfileSettings isOpen={isProfileOpen} onClose={closeProfile} />

      {/* Sign Out Confirmation Dialog */}
      <ConfirmDialog
        isOpen={isConfirmSignoutOpen}
        title="Sign Out?"
        message="Are you sure you want to sign out? You'll need to sign in again to access your personalized content."
        confirmLabel="Yes, Sign Out"
        cancelLabel="No, Stay Signed In"
        confirmVariant="danger"
        onConfirm={handleConfirmSignout}
        onCancel={handleCancelSignout}
        isLoading={isSigningOut}
      />

      {/* Success Toast */}
      {showSuccessToast && (
        <Toast
          message="You've been successfully signed out. See you next time!"
          type="success"
          onClose={handleCloseToast}
        />
      )}
    </div>
  );
}

export default UserMenu;

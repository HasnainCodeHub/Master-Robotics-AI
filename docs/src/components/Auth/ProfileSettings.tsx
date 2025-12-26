/**
 * ProfileSettings - Profile editing form modal.
 *
 * Features:
 * - Edit software_level, robotics_level, hardware_access
 * - Toggle personalization_enabled
 * - Save button with loading state
 * - Success toast on save
 * - Keyboard accessible
 */

import React, { useState, useCallback, useEffect, useRef } from 'react';
import { useAuth } from './AuthContext';
import type { ExperienceLevel, HardwareAccess, ProfileUpdateRequest } from './types';
import styles from './styles.module.css';

/**
 * Props for ProfileSettings.
 */
interface ProfileSettingsProps {
  isOpen: boolean;
  onClose: () => void;
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
 * ProfileSettings component.
 */
export function ProfileSettings({ isOpen, onClose }: ProfileSettingsProps): JSX.Element | null {
  const { user, profile, updateProfile, error, clearError } = useAuth();
  const modalRef = useRef<HTMLDivElement>(null);

  // Form state
  const [softwareLevel, setSoftwareLevel] = useState<ExperienceLevel>(
    profile?.software_level || 'beginner'
  );
  const [roboticsLevel, setRoboticsLevel] = useState<ExperienceLevel>(
    profile?.robotics_level || 'beginner'
  );
  const [hardwareAccess, setHardwareAccess] = useState<HardwareAccess>(
    profile?.hardware_access || 'simulation_only'
  );
  const [personalizationEnabled, setPersonalizationEnabled] = useState(
    profile?.personalization_enabled ?? true
  );

  // UI state
  const [isSaving, setIsSaving] = useState(false);
  const [showSuccess, setShowSuccess] = useState(false);
  const [localError, setLocalError] = useState<string | null>(null);

  /**
   * Sync form state when profile changes.
   */
  useEffect(() => {
    if (profile) {
      setSoftwareLevel(profile.software_level);
      setRoboticsLevel(profile.robotics_level);
      setHardwareAccess(profile.hardware_access);
      setPersonalizationEnabled(profile.personalization_enabled);
    }
  }, [profile]);

  /**
   * Reset state when modal opens.
   */
  useEffect(() => {
    if (isOpen) {
      clearError();
      setLocalError(null);
      setShowSuccess(false);
    }
  }, [isOpen, clearError]);

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
   * Handle save.
   */
  const handleSave = useCallback(async () => {
    setIsSaving(true);
    setLocalError(null);
    setShowSuccess(false);

    try {
      const updates: ProfileUpdateRequest = {
        software_level: softwareLevel,
        robotics_level: roboticsLevel,
        hardware_access: hardwareAccess,
        personalization_enabled: personalizationEnabled,
      };

      await updateProfile(updates);
      setShowSuccess(true);

      // Auto-hide success message after 3 seconds
      setTimeout(() => {
        setShowSuccess(false);
      }, 3000);
    } catch {
      setLocalError('Failed to save profile. Please try again.');
    } finally {
      setIsSaving(false);
    }
  }, [softwareLevel, roboticsLevel, hardwareAccess, personalizationEnabled, updateProfile]);

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
      aria-labelledby="profile-modal-title"
    >
      <div className={styles.modalContainer} ref={modalRef}>
        {/* Header */}
        <div className={styles.modalHeader}>
          <h2 id="profile-modal-title" className={styles.modalTitle}>
            Profile Settings
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

        {/* User Info */}
        <div className={styles.profileUserInfo}>
          <div className={styles.profileAvatar}>
            {user?.email?.charAt(0).toUpperCase() || '?'}
          </div>
          <div className={styles.profileEmail}>{user?.email}</div>
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

        {showSuccess && (
          <div className={styles.successBanner} role="status">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14" />
              <polyline points="22 4 12 14.01 9 11.01" />
            </svg>
            <span>Profile saved successfully!</span>
          </div>
        )}

        {/* Form */}
        <div className={styles.formPanel}>
          <div className={styles.formDivider}>
            <span>Experience Level</span>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="profile-software-level" className={styles.label}>
              Software Experience
            </label>
            <select
              id="profile-software-level"
              className={styles.select}
              value={softwareLevel}
              onChange={(e) => setSoftwareLevel(e.target.value as ExperienceLevel)}
              disabled={isSaving}
            >
              {SOFTWARE_LEVELS.map((option) => (
                <option key={option.value} value={option.value}>
                  {option.label}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="profile-robotics-level" className={styles.label}>
              Robotics Experience
            </label>
            <select
              id="profile-robotics-level"
              className={styles.select}
              value={roboticsLevel}
              onChange={(e) => setRoboticsLevel(e.target.value as ExperienceLevel)}
              disabled={isSaving}
            >
              {ROBOTICS_LEVELS.map((option) => (
                <option key={option.value} value={option.value}>
                  {option.label}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="profile-hardware-access" className={styles.label}>
              Hardware Access
            </label>
            <select
              id="profile-hardware-access"
              className={styles.select}
              value={hardwareAccess}
              onChange={(e) => setHardwareAccess(e.target.value as HardwareAccess)}
              disabled={isSaving}
            >
              {HARDWARE_OPTIONS.map((option) => (
                <option key={option.value} value={option.value}>
                  {option.label}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formDivider}>
            <span>AI Features</span>
          </div>

          <div className={styles.toggleGroup}>
            <div className={styles.toggleInfo}>
              <label htmlFor="personalization-toggle" className={styles.toggleLabel}>
                Enable Personalization
              </label>
              <p className={styles.toggleDescription}>
                When enabled, AI responses will be tailored to your experience level
              </p>
            </div>
            <label className={styles.toggleSwitch}>
              <input
                id="personalization-toggle"
                type="checkbox"
                checked={personalizationEnabled}
                onChange={(e) => setPersonalizationEnabled(e.target.checked)}
                disabled={isSaving}
              />
              <span className={styles.toggleSlider} />
            </label>
          </div>

          <button
            type="button"
            className={styles.submitButton}
            onClick={handleSave}
            disabled={isSaving}
          >
            {isSaving ? (
              <span className={styles.loadingSpinner} />
            ) : (
              'Save Changes'
            )}
          </button>
        </div>
      </div>
    </div>
  );
}

export default ProfileSettings;

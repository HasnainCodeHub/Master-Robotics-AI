/**
 * PersonalizationToggle - Quick toggle for personalization_enabled.
 *
 * Allows users to quickly enable/disable personalization.
 * Calls updateProfile when toggled to persist the preference.
 *
 * @see specs/auth/personalization-rules.md Section 4.3
 */

import React, { useState, useCallback } from 'react';
import { useAuth } from '../Auth/AuthContext';
import { usePersonalization } from './PersonalizationContext';

/**
 * Props for PersonalizationToggle component.
 */
interface PersonalizationToggleProps {
  /** Optional custom class name */
  className?: string;
  /** Optional label text */
  label?: string;
  /** Show label inline or screen-reader only */
  showLabel?: boolean;
}

/**
 * PersonalizationToggle component.
 * Renders a toggle switch to enable/disable personalization.
 *
 * @example
 * ```tsx
 * // In user menu
 * <PersonalizationToggle showLabel />
 * ```
 */
export function PersonalizationToggle({
  className = '',
  label = 'Personalization',
  showLabel = true,
}: PersonalizationToggleProps): JSX.Element | null {
  const { profile, updateProfile, isAuthenticated } = useAuth();
  const { isEnabled } = usePersonalization();
  const [isUpdating, setIsUpdating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Only show for authenticated users
  if (!isAuthenticated || !profile) {
    return null;
  }

  const handleToggle = useCallback(async () => {
    if (isUpdating) return;

    setIsUpdating(true);
    setError(null);

    try {
      await updateProfile({
        personalization_enabled: !profile.personalization_enabled,
      });
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to update preference';
      setError(message);
      // Error will clear on next successful toggle
    } finally {
      setIsUpdating(false);
    }
  }, [profile, updateProfile, isUpdating]);

  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      handleToggle();
    }
  };

  return (
    <div
      className={`personalization-toggle ${isEnabled ? 'enabled' : 'disabled'} ${isUpdating ? 'updating' : ''} ${className}`}
      data-personalization-component="toggle"
    >
      <label className="personalization-toggle__label">
        <span className={showLabel ? 'personalization-toggle__label-text' : 'sr-only'}>
          {label}
        </span>

        <button
          type="button"
          role="switch"
          aria-checked={isEnabled}
          aria-label={`${label}: ${isEnabled ? 'On' : 'Off'}`}
          className="personalization-toggle__switch"
          onClick={handleToggle}
          onKeyDown={handleKeyDown}
          disabled={isUpdating}
        >
          <span className="personalization-toggle__track">
            <span className="personalization-toggle__thumb" />
          </span>
        </button>

        <span className="personalization-toggle__status" aria-live="polite">
          {isUpdating ? 'Updating...' : isEnabled ? 'On' : 'Off'}
        </span>
      </label>

      {error && (
        <div className="personalization-toggle__error" role="alert">
          {error}
        </div>
      )}
    </div>
  );
}

export default PersonalizationToggle;

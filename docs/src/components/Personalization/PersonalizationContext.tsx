/**
 * PersonalizationContext - Presentation-level personalization state.
 *
 * CRITICAL: This is PRESENTATION ONLY.
 * - RAG outputs are IDENTICAL for all users
 * - Personalization controls UI display, not content meaning
 * - Profile data NEVER leaves the frontend to RAG/LLM
 *
 * @see specs/auth/personalization-rules.md for full specification
 */

import React, {
  createContext,
  useContext,
  useEffect,
  useMemo,
  ReactNode,
} from 'react';
import { useAuth } from '../Auth/AuthContext';
import type { ExperienceLevel, HardwareAccess } from '../Auth/types';

/**
 * Personalization settings derived from user profile.
 */
export interface PersonalizationSettings {
  /** Software experience level (beginner/intermediate/advanced) */
  softwareLevel: ExperienceLevel;
  /** Robotics experience level (beginner/intermediate/advanced) */
  roboticsLevel: ExperienceLevel;
  /** User's hardware access type */
  hardwareAccess: HardwareAccess;
  /** Whether personalization is enabled */
  isEnabled: boolean;
  /** Whether the user is authenticated */
  isAuthenticated: boolean;
  /** Combined level for general content adjustments */
  effectiveLevel: ExperienceLevel;
}

/**
 * Default settings when not authenticated or personalization disabled.
 * Per spec: "When disabled: Show content as if user is 'intermediate' level"
 */
const DEFAULT_SETTINGS: PersonalizationSettings = {
  softwareLevel: 'intermediate',
  roboticsLevel: 'intermediate',
  hardwareAccess: 'simulation_only',
  isEnabled: false,
  isAuthenticated: false,
  effectiveLevel: 'intermediate',
};

/**
 * Personalization context value.
 */
const PersonalizationContext = createContext<PersonalizationSettings>(DEFAULT_SETTINGS);

/**
 * Props for PersonalizationProvider.
 */
interface PersonalizationProviderProps {
  children: ReactNode;
}

/**
 * Calculate effective level from software and robotics levels.
 * Per spec:
 * - Both Beginner = beginner (expanded explanations)
 * - Mixed = intermediate (standard explanations)
 * - Both Advanced = advanced (concise explanations)
 */
function calculateEffectiveLevel(
  softwareLevel: ExperienceLevel,
  roboticsLevel: ExperienceLevel
): ExperienceLevel {
  if (softwareLevel === 'beginner' && roboticsLevel === 'beginner') {
    return 'beginner';
  }
  if (softwareLevel === 'advanced' && roboticsLevel === 'advanced') {
    return 'advanced';
  }
  return 'intermediate';
}

/**
 * PersonalizationProvider component.
 * Wraps the application and provides personalization settings to all children.
 * Sets data attributes on document.documentElement for CSS-based personalization.
 */
export function PersonalizationProvider({ children }: PersonalizationProviderProps): JSX.Element {
  const { profile, isAuthenticated } = useAuth();

  /**
   * Calculate personalization settings from profile.
   */
  const settings = useMemo<PersonalizationSettings>(() => {
    // Not authenticated or no profile: use defaults
    if (!isAuthenticated || !profile) {
      return DEFAULT_SETTINGS;
    }

    // Personalization disabled: use intermediate defaults
    if (!profile.personalization_enabled) {
      return {
        ...DEFAULT_SETTINGS,
        isAuthenticated: true,
        isEnabled: false,
      };
    }

    // Personalization enabled: use profile values
    const effectiveLevel = calculateEffectiveLevel(
      profile.software_level,
      profile.robotics_level
    );

    return {
      softwareLevel: profile.software_level,
      roboticsLevel: profile.robotics_level,
      hardwareAccess: profile.hardware_access,
      isEnabled: true,
      isAuthenticated: true,
      effectiveLevel,
    };
  }, [profile, isAuthenticated]);

  /**
   * Set data attributes on document.documentElement for CSS-based personalization.
   * This allows CSS rules to style based on user levels without JavaScript.
   */
  useEffect(() => {
    const root = document.documentElement;

    // Set personalization enabled state
    root.setAttribute(
      'data-personalization',
      settings.isEnabled ? 'enabled' : 'disabled'
    );

    // Set level attributes (for CSS selectors)
    root.setAttribute('data-software-level', settings.softwareLevel);
    root.setAttribute('data-robotics-level', settings.roboticsLevel);
    root.setAttribute('data-hardware', settings.hardwareAccess);
    root.setAttribute('data-level', settings.effectiveLevel);

    // Cleanup on unmount
    return () => {
      root.removeAttribute('data-personalization');
      root.removeAttribute('data-software-level');
      root.removeAttribute('data-robotics-level');
      root.removeAttribute('data-hardware');
      root.removeAttribute('data-level');
    };
  }, [settings]);

  return (
    <PersonalizationContext.Provider value={settings}>
      {children}
    </PersonalizationContext.Provider>
  );
}

/**
 * usePersonalization hook.
 * Access personalization settings from any component.
 *
 * @example
 * ```tsx
 * const { effectiveLevel, isEnabled } = usePersonalization();
 * ```
 */
export function usePersonalization(): PersonalizationSettings {
  return useContext(PersonalizationContext);
}

export default PersonalizationContext;

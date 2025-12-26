/**
 * HardwareRecommendation - Hardware-aware content highlighting.
 *
 * PRESENTATION ONLY:
 * - Shows "Recommended for your hardware" badge
 * - Based on user's hardware_access
 * - ALL examples remain VISIBLE
 * - Only highlights relevant ones, NEVER hides content
 *
 * @see specs/auth/personalization-rules.md Section 1.5
 */

import React, { ReactNode } from 'react';
import { usePersonalization } from './PersonalizationContext';
import type { HardwareAccess } from '../Auth/types';

/**
 * Hardware type that this content is relevant for.
 */
export type HardwareType = 'simulation' | 'jetson' | 'physical_robot';

/**
 * Map from HardwareAccess profile value to HardwareType.
 */
const HARDWARE_ACCESS_MAP: Record<HardwareAccess, HardwareType> = {
  simulation_only: 'simulation',
  jetson_device: 'jetson',
  physical_robot: 'physical_robot',
};

/**
 * Props for HardwareRecommendation component.
 */
interface HardwareRecommendationProps {
  /** Content to show (always visible) */
  children: ReactNode;
  /** Hardware type this content is relevant for */
  hardware: HardwareType;
  /** Optional custom class name */
  className?: string;
  /** Optional custom badge text */
  badgeText?: string;
}

/**
 * HardwareRecommendation component.
 * Wraps content and shows a "Recommended" badge if it matches user's hardware.
 *
 * CRITICAL: ALL content is ALWAYS visible. Only the badge visibility changes.
 *
 * @example
 * ```tsx
 * <HardwareRecommendation hardware="jetson">
 *   <h3>Jetson Deployment</h3>
 *   <p>When deploying to Jetson devices...</p>
 * </HardwareRecommendation>
 *
 * <HardwareRecommendation hardware="simulation">
 *   <h3>Gazebo Simulation</h3>
 *   <p>For testing in simulation...</p>
 * </HardwareRecommendation>
 * ```
 */
export function HardwareRecommendation({
  children,
  hardware,
  className = '',
  badgeText = 'Recommended for your hardware',
}: HardwareRecommendationProps): JSX.Element {
  const { hardwareAccess, isEnabled, isAuthenticated } = usePersonalization();

  // Determine if this content matches user's hardware
  const userHardwareType = HARDWARE_ACCESS_MAP[hardwareAccess];
  const isRecommended = isEnabled && isAuthenticated && userHardwareType === hardware;

  // CSS class for hardware-specific styling
  const hardwareClass = `${hardware}-example`;

  return (
    <div
      className={`personalization-hardware-recommendation ${hardwareClass} ${isRecommended ? 'recommended' : ''} ${className}`}
      data-personalization-component="hardware-recommendation"
      data-hardware-type={hardware}
      data-recommended={isRecommended ? 'true' : 'false'}
    >
      {isRecommended && (
        <div className="personalization-hardware-recommendation__badge">
          <span className="personalization-hardware-recommendation__badge-icon" aria-hidden="true">
            ★
          </span>
          <span className="personalization-hardware-recommendation__badge-text">
            {badgeText}
          </span>
        </div>
      )}

      <div className="personalization-hardware-recommendation__content">
        {children}
      </div>
    </div>
  );
}

export default HardwareRecommendation;

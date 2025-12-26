/**
 * AdvancedDeepDive - Collapsible advanced content section.
 *
 * PRESENTATION ONLY:
 * - Hidden (collapsed) by default for beginners (but expandable)
 * - Expanded by default for advanced users
 * - Collapsible section with expand/collapse toggle
 * - ALL content remains accessible to ALL users
 *
 * @see specs/auth/personalization-rules.md Section 1.1
 */

import React, { useState, useEffect, ReactNode } from 'react';
import { usePersonalization } from './PersonalizationContext';

/**
 * Props for AdvancedDeepDive component.
 */
interface AdvancedDeepDiveProps {
  /** Content to show in the deep dive section */
  children: ReactNode;
  /** Optional title for the section */
  title?: string;
  /** Optional custom class name */
  className?: string;
}

/**
 * AdvancedDeepDive component.
 * Renders a collapsible section for advanced content.
 * Expanded by default for advanced users, collapsed for others.
 *
 * @example
 * ```tsx
 * <AdvancedDeepDive title="Implementation Details">
 *   Under the hood, this uses DDS for message transport...
 * </AdvancedDeepDive>
 * ```
 */
export function AdvancedDeepDive({
  children,
  title = 'Advanced Deep Dive',
  className = '',
}: AdvancedDeepDiveProps): JSX.Element {
  const { effectiveLevel, isEnabled } = usePersonalization();

  // Determine default expanded state based on user level
  // Advanced: expanded by default
  // Others: collapsed by default (but always expandable)
  const defaultExpanded = isEnabled && effectiveLevel === 'advanced';

  const [isExpanded, setIsExpanded] = useState(defaultExpanded);

  // Update expanded state when personalization settings change
  useEffect(() => {
    setIsExpanded(isEnabled && effectiveLevel === 'advanced');
  }, [effectiveLevel, isEnabled]);

  const toggleExpanded = () => {
    setIsExpanded((prev) => !prev);
  };

  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      toggleExpanded();
    }
  };

  return (
    <div
      className={`personalization-deep-dive ${isExpanded ? 'expanded' : 'collapsed'} ${className}`}
      data-personalization-component="advanced-deep-dive"
    >
      <button
        type="button"
        className="personalization-deep-dive__toggle"
        onClick={toggleExpanded}
        onKeyDown={handleKeyDown}
        aria-expanded={isExpanded}
        aria-controls="deep-dive-content"
      >
        <span className="personalization-deep-dive__icon" aria-hidden="true">
          {isExpanded ? '▼' : '▶'}
        </span>
        <span className="personalization-deep-dive__title">{title}</span>
        <span className="personalization-deep-dive__badge">Advanced</span>
      </button>

      <div
        id="deep-dive-content"
        className="personalization-deep-dive__content"
        role="region"
        aria-labelledby="deep-dive-toggle"
        hidden={!isExpanded}
      >
        {children}
      </div>
    </div>
  );
}

export default AdvancedDeepDive;

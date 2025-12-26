/**
 * BeginnerHint - Collapsible extra explanations for beginners.
 *
 * PRESENTATION ONLY:
 * - Shows expanded by default for beginners
 * - Collapsed by default for others
 * - Can ALWAYS be expanded/collapsed by click
 * - NEVER hides content, only controls default state
 *
 * @see specs/auth/personalization-rules.md Section 1.1
 */

import React, { useState, useEffect, ReactNode } from 'react';
import { usePersonalization } from './PersonalizationContext';

/**
 * Props for BeginnerHint component.
 */
interface BeginnerHintProps {
  /** Content to show in the hint */
  children: ReactNode;
  /** Optional title for the hint section */
  title?: string;
  /** Optional custom class name */
  className?: string;
}

/**
 * BeginnerHint component.
 * Renders a collapsible hint section that is expanded by default for beginners.
 *
 * @example
 * ```tsx
 * <BeginnerHint title="What is a node?">
 *   A node is a process that performs computation in ROS 2.
 *   Think of it like a small program that does one specific job.
 * </BeginnerHint>
 * ```
 */
export function BeginnerHint({
  children,
  title = 'Beginner Hint',
  className = '',
}: BeginnerHintProps): JSX.Element {
  const { effectiveLevel, isEnabled } = usePersonalization();

  // Determine default expanded state based on user level
  // Beginners: expanded by default
  // Others: collapsed by default
  const defaultExpanded = isEnabled && effectiveLevel === 'beginner';

  const [isExpanded, setIsExpanded] = useState(defaultExpanded);

  // Update expanded state when personalization settings change
  useEffect(() => {
    setIsExpanded(isEnabled && effectiveLevel === 'beginner');
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
      className={`personalization-beginner-hint ${isExpanded ? 'expanded' : 'collapsed'} ${className}`}
      data-personalization-component="beginner-hint"
    >
      <button
        type="button"
        className="personalization-beginner-hint__toggle"
        onClick={toggleExpanded}
        onKeyDown={handleKeyDown}
        aria-expanded={isExpanded}
        aria-controls="beginner-hint-content"
      >
        <span className="personalization-beginner-hint__icon" aria-hidden="true">
          {isExpanded ? '▼' : '▶'}
        </span>
        <span className="personalization-beginner-hint__title">{title}</span>
      </button>

      <div
        id="beginner-hint-content"
        className="personalization-beginner-hint__content"
        role="region"
        aria-labelledby="beginner-hint-toggle"
        hidden={!isExpanded}
      >
        {children}
      </div>
    </div>
  );
}

export default BeginnerHint;

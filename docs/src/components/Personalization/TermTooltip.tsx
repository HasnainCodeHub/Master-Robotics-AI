/**
 * TermTooltip - Terminology tooltips with level-aware behavior.
 *
 * PRESENTATION ONLY:
 * - Wraps terms with hover tooltip definitions
 * - Auto-show on hover for beginners
 * - Click-to-show for advanced users
 * - Tooltip contains definition text
 * - NEVER modifies the term itself
 *
 * @see specs/auth/personalization-rules.md Section 1.4
 */

import React, { useState, useRef, useEffect, ReactNode } from 'react';
import { usePersonalization } from './PersonalizationContext';

/**
 * Props for TermTooltip component.
 */
interface TermTooltipProps {
  /** The term to wrap (visible text) */
  children: ReactNode;
  /** Definition to show in tooltip */
  definition: string;
  /** Optional custom class name */
  className?: string;
}

/**
 * TermTooltip component.
 * Renders a term with a tooltip showing its definition.
 * Behavior varies by user level:
 * - Beginner: auto-show on hover
 * - Intermediate: show on hover
 * - Advanced: click to show
 *
 * @example
 * ```tsx
 * <p>
 *   The <TermTooltip definition="A named channel for sending messages between nodes">
 *     topic
 *   </TermTooltip> allows nodes to communicate.
 * </p>
 * ```
 */
export function TermTooltip({
  children,
  definition,
  className = '',
}: TermTooltipProps): JSX.Element {
  const { effectiveLevel, roboticsLevel, isEnabled } = usePersonalization();
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState<{ top: number; left: number }>({ top: 0, left: 0 });
  const termRef = useRef<HTMLSpanElement>(null);
  const tooltipRef = useRef<HTMLDivElement>(null);

  // Determine interaction mode based on user level
  // Per spec: Beginner = auto-show on hover, Advanced = click only
  const isClickMode = isEnabled && roboticsLevel === 'advanced';
  const isAutoShow = isEnabled && roboticsLevel === 'beginner';

  // Calculate tooltip position
  const updatePosition = () => {
    if (termRef.current && tooltipRef.current) {
      const rect = termRef.current.getBoundingClientRect();
      const tooltipRect = tooltipRef.current.getBoundingClientRect();

      // Position above the term, centered
      let left = rect.left + rect.width / 2 - tooltipRect.width / 2;
      let top = rect.top - tooltipRect.height - 8;

      // Keep tooltip within viewport
      if (left < 8) left = 8;
      if (left + tooltipRect.width > window.innerWidth - 8) {
        left = window.innerWidth - tooltipRect.width - 8;
      }
      if (top < 8) {
        // Show below if not enough space above
        top = rect.bottom + 8;
      }

      setPosition({ top, left });
    }
  };

  useEffect(() => {
    if (isVisible) {
      updatePosition();
      window.addEventListener('scroll', updatePosition, true);
      window.addEventListener('resize', updatePosition);
      return () => {
        window.removeEventListener('scroll', updatePosition, true);
        window.removeEventListener('resize', updatePosition);
      };
    }
  }, [isVisible]);

  // Handle mouse enter
  const handleMouseEnter = () => {
    if (!isClickMode) {
      setIsVisible(true);
    }
  };

  // Handle mouse leave
  const handleMouseLeave = () => {
    if (!isClickMode) {
      setIsVisible(false);
    }
  };

  // Handle click
  const handleClick = () => {
    if (isClickMode) {
      setIsVisible((prev) => !prev);
    }
  };

  // Handle keyboard
  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Escape') {
      setIsVisible(false);
    }
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      setIsVisible((prev) => !prev);
    }
  };

  // Close tooltip when clicking outside
  useEffect(() => {
    if (isClickMode && isVisible) {
      const handleClickOutside = (event: MouseEvent) => {
        if (
          termRef.current &&
          !termRef.current.contains(event.target as Node) &&
          tooltipRef.current &&
          !tooltipRef.current.contains(event.target as Node)
        ) {
          setIsVisible(false);
        }
      };

      document.addEventListener('mousedown', handleClickOutside);
      return () => {
        document.removeEventListener('mousedown', handleClickOutside);
      };
    }
  }, [isClickMode, isVisible]);

  return (
    <>
      <span
        ref={termRef}
        className={`personalization-term ${isClickMode ? 'click-mode' : 'hover-mode'} ${isAutoShow ? 'auto-show' : ''} ${className}`}
        data-personalization-component="term-tooltip"
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
        onClick={handleClick}
        onKeyDown={handleKeyDown}
        tabIndex={0}
        role="button"
        aria-describedby={isVisible ? 'term-tooltip' : undefined}
      >
        {children}
      </span>

      {isVisible && (
        <div
          ref={tooltipRef}
          id="term-tooltip"
          className="personalization-tooltip"
          role="tooltip"
          style={{
            position: 'fixed',
            top: position.top,
            left: position.left,
          }}
        >
          <div className="personalization-tooltip__content">
            {definition}
          </div>
          <div className="personalization-tooltip__arrow" aria-hidden="true" />
        </div>
      )}
    </>
  );
}

export default TermTooltip;

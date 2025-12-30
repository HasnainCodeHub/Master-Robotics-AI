/**
 * ConfirmDialog - Reusable confirmation dialog component.
 *
 * Features:
 * - Customizable title, message, and button labels
 * - Keyboard accessible (ESC to cancel, Enter to confirm)
 * - Focus trapping
 * - Body scroll locking when open
 * - Professional centered modal styling
 * - Uses React Portal to render at document root (avoids z-index issues)
 */

import React, { useCallback, useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import styles from './styles.module.css';

interface ConfirmDialogProps {
  isOpen: boolean;
  title: string;
  message: string;
  confirmLabel?: string;
  cancelLabel?: string;
  confirmVariant?: 'primary' | 'danger';
  onConfirm: () => void;
  onCancel: () => void;
  isLoading?: boolean;
}

export function ConfirmDialog({
  isOpen,
  title,
  message,
  confirmLabel = 'Confirm',
  cancelLabel = 'Cancel',
  confirmVariant = 'primary',
  onConfirm,
  onCancel,
  isLoading = false,
}: ConfirmDialogProps): JSX.Element | null {
  const modalRef = useRef<HTMLDivElement>(null);
  const confirmButtonRef = useRef<HTMLButtonElement>(null);
  const scrollPositionRef = useRef<number>(0);
  const [portalContainer, setPortalContainer] = useState<HTMLElement | null>(null);

  /**
   * Create portal container on mount (client-side only for SSR compatibility).
   */
  useEffect(() => {
    // Only run on client side
    if (typeof window !== 'undefined') {
      setPortalContainer(document.body);
    }
  }, []);

  /**
   * Lock body scroll when modal is open.
   */
  useEffect(() => {
    if (isOpen) {
      // Store current scroll position
      scrollPositionRef.current = window.scrollY;
      // Lock body scroll
      document.body.classList.add('body-scroll-locked');
      document.body.style.top = `-${scrollPositionRef.current}px`;
    } else {
      // Unlock body scroll
      document.body.classList.remove('body-scroll-locked');
      document.body.style.top = '';
      // Restore scroll position
      window.scrollTo(0, scrollPositionRef.current);
    }

    return () => {
      // Cleanup on unmount
      document.body.classList.remove('body-scroll-locked');
      document.body.style.top = '';
    };
  }, [isOpen]);

  /**
   * Handle ESC key to cancel.
   */
  useEffect(() => {
    function handleKeyDown(event: KeyboardEvent) {
      if (event.key === 'Escape' && isOpen && !isLoading) {
        onCancel();
      }
    }

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, isLoading, onCancel]);

  /**
   * Focus confirm button when dialog opens.
   */
  useEffect(() => {
    if (isOpen) {
      setTimeout(() => {
        confirmButtonRef.current?.focus();
      }, 100);
    }
  }, [isOpen]);

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
      if (event.target === event.currentTarget && !isLoading) {
        onCancel();
      }
    },
    [isLoading, onCancel]
  );

  // Don't render if not open or portal not ready
  if (!isOpen || !portalContainer) {
    return null;
  }

  const dialogContent = (
    <div
      className={styles.confirmOverlay}
      onClick={handleBackdropClick}
      role="dialog"
      aria-modal="true"
      aria-labelledby="confirm-dialog-title"
    >
      <div className={styles.confirmDialog} ref={modalRef}>
        {/* Icon */}
        <div className={styles.confirmIcon}>
          <svg
            width="48"
            height="48"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <circle cx="12" cy="12" r="10" />
            <line x1="12" y1="8" x2="12" y2="12" />
            <line x1="12" y1="16" x2="12.01" y2="16" />
          </svg>
        </div>

        {/* Content */}
        <div className={styles.confirmContent}>
          <h2 id="confirm-dialog-title" className={styles.confirmTitle}>
            {title}
          </h2>
          <p className={styles.confirmMessage}>{message}</p>
        </div>

        {/* Actions */}
        <div className={styles.confirmActions}>
          <button
            type="button"
            className={styles.confirmCancelButton}
            onClick={onCancel}
            disabled={isLoading}
          >
            {cancelLabel}
          </button>
          <button
            ref={confirmButtonRef}
            type="button"
            className={
              confirmVariant === 'danger'
                ? styles.confirmDangerButton
                : styles.confirmPrimaryButton
            }
            onClick={onConfirm}
            disabled={isLoading}
          >
            {isLoading ? (
              <span className={styles.loadingSpinner} />
            ) : (
              confirmLabel
            )}
          </button>
        </div>
      </div>
    </div>
  );

  // Use portal to render at document root, avoiding z-index issues
  return createPortal(dialogContent, portalContainer);
}

export default ConfirmDialog;

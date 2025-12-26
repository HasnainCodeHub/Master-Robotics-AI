/**
 * Search Bar Component
 * Displayed in the navbar, opens search modal on click.
 */

import React, { useCallback, useEffect, useState } from 'react';
import { useSearch } from './SearchContext';
import styles from './styles.module.css';

export function SearchBar() {
  const { openSearch } = useSearch();
  const [isMac, setIsMac] = useState(false);

  // Detect macOS for showing correct shortcut
  useEffect(() => {
    if (typeof navigator !== 'undefined') {
      setIsMac(navigator.platform.toUpperCase().indexOf('MAC') >= 0);
    }
  }, []);

  const handleClick = useCallback(() => {
    openSearch();
  }, [openSearch]);

  return (
    <button
      className={styles.searchBar}
      onClick={handleClick}
      aria-label="Search textbook"
      type="button"
    >
      <svg
        className={styles.searchBarIcon}
        width="16"
        height="16"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <circle cx="11" cy="11" r="8" />
        <line x1="21" y1="21" x2="16.65" y2="16.65" />
      </svg>
      <span className={styles.searchBarText}>Search docs...</span>
      <span className={styles.searchBarShortcut}>
        <kbd>{isMac ? '⌘' : 'Ctrl'}</kbd>
        <kbd>K</kbd>
      </span>
    </button>
  );
}

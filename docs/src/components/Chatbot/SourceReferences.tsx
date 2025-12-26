/**
 * Source References Component
 * Displays the sources used to generate an answer with transparency.
 */

import React, { useState } from 'react';
import type { SourceReference } from './types';
import styles from './styles.module.css';

interface SourceReferencesProps {
  sources: SourceReference[];
}

/**
 * Render confidence score as filled/empty circles.
 */
function renderScore(score: number): string {
  const filled = Math.round(score * 5);
  const empty = 5 - filled;
  return '●'.repeat(filled) + '○'.repeat(empty);
}

/**
 * Format module ID for display.
 */
function formatModuleId(moduleId: string): string {
  // Convert "module-1" to "Module 1"
  return moduleId
    .replace(/-/g, ' ')
    .replace(/\b\w/g, (l) => l.toUpperCase());
}

/**
 * Format chapter ID for display.
 */
function formatChapterId(chapterId: string): string {
  // Convert "01-embodied-intelligence" to "Chapter 1: Embodied Intelligence"
  const match = chapterId.match(/^(\d+)-(.+)$/);
  if (match) {
    const num = parseInt(match[1], 10);
    const name = match[2]
      .replace(/-/g, ' ')
      .replace(/\b\w/g, (l) => l.toUpperCase());
    return `Chapter ${num}: ${name}`;
  }
  return chapterId;
}

export function SourceReferences({ sources }: SourceReferencesProps) {
  const [isExpanded, setIsExpanded] = useState(false);

  if (sources.length === 0) {
    return null;
  }

  return (
    <div className={styles.sourcesSection}>
      <button
        className={styles.sourcesToggle}
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
        aria-controls="source-list"
      >
        📚 Sources ({sources.length})
        <span aria-hidden="true">{isExpanded ? ' ▼' : ' ▶'}</span>
      </button>

      {isExpanded && (
        <div id="source-list" className={styles.sourcesList}>
          {sources.map((source, index) => (
            <div key={source.chunk_id} className={styles.sourceItem}>
              <span className={styles.sourceIcon} aria-hidden="true">
                {index === sources.length - 1 ? '└─' : '├─'}
              </span>
              <div className={styles.sourceInfo}>
                <div className={styles.sourceModule}>
                  {formatModuleId(source.module_id)}
                </div>
                <div className={styles.sourceChapter}>
                  └─ {formatChapterId(source.chapter_id)} § {source.section_heading}
                </div>
                <div className={styles.sourceScore}>
                  Relevance: {renderScore(source.score)} ({Math.round(source.score * 100)}%)
                </div>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}

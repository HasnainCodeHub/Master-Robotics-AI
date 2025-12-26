/**
 * Refusal Message Component
 * Displays refusal messages with distinct styling and helpful suggestions.
 * Refusals are NOT errors - they are the chatbot being transparent about limits.
 */

import React from 'react';
import type { RefusalReason } from './types';
import styles from './styles.module.css';

interface RefusalMessageProps {
  reason: RefusalReason | null;
  content: string;
}

interface RefusalConfig {
  title: string;
  hint: string;
}

const REFUSAL_CONFIG: Record<RefusalReason, RefusalConfig> = {
  empty_retrieval: {
    title: 'Topic Not Covered',
    hint: 'Try asking about: Physical AI, ROS 2, Simulation, Isaac, or VLA systems.',
  },
  insufficient_context: {
    title: 'Need More Context',
    hint: 'Try rephrasing your question more specifically, or selecting text from the chapter.',
  },
  out_of_scope: {
    title: 'Outside Course Scope',
    hint: 'This topic is not covered in the Physical AI & Humanoid Robotics textbook.',
  },
  selected_text_insufficient: {
    title: 'Selection Too Limited',
    hint: 'Try selecting a different section, or ask without selection to search the full textbook.',
  },
};

export function RefusalMessage({ reason, content }: RefusalMessageProps) {
  const config = reason ? REFUSAL_CONFIG[reason] : {
    title: 'Cannot Answer',
    hint: 'Please try a different question.',
  };

  return (
    <div className={styles.refusalMessage} role="status" aria-live="polite">
      <div>
        <span className={styles.refusalIcon} aria-hidden="true">ℹ️</span>
        <span className={styles.refusalTitle}>{config.title}</span>
      </div>
      <div>{content}</div>
      <div className={styles.refusalHint}>
        💡 {config.hint}
      </div>
    </div>
  );
}

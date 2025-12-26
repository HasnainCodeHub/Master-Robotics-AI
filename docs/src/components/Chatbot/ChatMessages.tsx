/**
 * Chat Messages Component
 * Renders the message history with proper styling for different message types.
 */

import React, { useRef, useEffect } from 'react';
import type { Message } from './types';
import { SourceReferences } from './SourceReferences';
import { RefusalMessage } from './RefusalMessage';
import styles from './styles.module.css';

interface ChatMessagesProps {
  messages: Message[];
  isLoading: boolean;
}

function WelcomeMessage() {
  return (
    <div className={styles.welcomeMessage}>
      <p>
        <strong>Ask me anything about the Physical AI & Humanoid Robotics textbook.</strong>
      </p>
      <p>
        I can only answer from the course content. Select text from a chapter for focused questions.
      </p>
    </div>
  );
}

function LoadingIndicator() {
  return (
    <div className={styles.loadingIndicator} aria-label="Loading response">
      <span className={styles.loadingDot}></span>
      <span className={styles.loadingDot}></span>
      <span className={styles.loadingDot}></span>
    </div>
  );
}

function UserMessageComponent({ message }: { message: Message }) {
  return (
    <div className={`${styles.message} ${styles.userMessage}`}>
      <div>{message.content}</div>
      {message.selectedText && (
        <div className={styles.selectedTextIndicator}>
          <div className={styles.selectedTextLabel}>
            📌 Context ({message.selectedText.length} chars)
          </div>
          <div className={styles.selectedTextContent}>
            "{message.selectedText.text.slice(0, 100)}..."
          </div>
        </div>
      )}
    </div>
  );
}

function AssistantMessageComponent({ message }: { message: Message }) {
  if (message.isRefusal) {
    return (
      <RefusalMessage
        reason={message.refusalReason as any}
        content={message.content}
      />
    );
  }

  return (
    <div className={`${styles.message} ${styles.assistantMessage}`}>
      <div>{message.content}</div>
      {message.sources && message.sources.length > 0 && (
        <SourceReferences sources={message.sources} />
      )}
    </div>
  );
}

export function ChatMessages({ messages, isLoading }: ChatMessagesProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  return (
    <div className={styles.messagesArea} role="log" aria-live="polite">
      {messages.length === 0 && !isLoading && <WelcomeMessage />}

      {messages.map((message) => (
        message.role === 'user' ? (
          <UserMessageComponent key={message.id} message={message} />
        ) : (
          <AssistantMessageComponent key={message.id} message={message} />
        )
      ))}

      {isLoading && <LoadingIndicator />}

      <div ref={messagesEndRef} />
    </div>
  );
}

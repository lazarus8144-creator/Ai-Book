import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { Message, QueryResponse } from './types';

// Use globalThis to safely access environment variables in browser
const API_URL = (typeof window !== 'undefined' && (window as any).REACT_APP_API_URL) || 'http://localhost:8000';

const WELCOME_MESSAGE: Message = {
  role: 'assistant',
  content: 'Hello! I\'m your AI teaching assistant for the Physical AI & Humanoid Robotics textbook. Ask me anything about ROS 2, digital twins, NVIDIA Isaac, or Vision-Language-Action models!',
  timestamp: new Date(),
};

export default function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([WELCOME_MESSAGE]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: input,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/api/v1/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: input }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data: QueryResponse = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Chatbot error:', err);
      const errorMsg = err instanceof Error ? err.message : 'Unknown error';
      setError(errorMsg);

      const errorMessage: Message = {
        role: 'assistant',
        content: `Sorry, I encountered an error: ${errorMsg}\n\nPlease check:\n- Backend is running on http://localhost:8000\n- Your internet connection\n- Browser console for details`,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleOpen = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Floating Button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={toggleOpen}
          aria-label="Open chatbot"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.header}>
            <h3 className={styles.title}>AI Teaching Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={toggleOpen}
              aria-label="Close chatbot"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="18" y1="6" x2="6" y2="18" />
                <line x1="6" y1="6" x2="18" y2="18" />
              </svg>
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${
                  message.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageContent}>{message.content}</div>

                {/* Sources */}
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <div className={styles.sourcesLabel}>Sources:</div>
                    {message.sources.map((source, idx) => (
                      <a
                        key={idx}
                        href={source.url}
                        className={styles.sourceLink}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        {source.title} ({source.module})
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {/* Loading Indicator */}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            {/* Error Message */}
            {error && (
              <div className={styles.errorBanner}>
                {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form className={styles.inputForm} onSubmit={handleSubmit}>
            <input
              type="text"
              className={styles.input}
              placeholder="Ask a question..."
              value={input}
              onChange={(e) => setInput(e.target.value)}
              disabled={isLoading}
              maxLength={500}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={!input.trim() || isLoading}
              aria-label="Send message"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="22" y1="2" x2="11" y2="13" />
                <polygon points="22 2 15 22 11 13 2 9 22 2" />
              </svg>
            </button>
          </form>
        </div>
      )}
    </>
  );
}

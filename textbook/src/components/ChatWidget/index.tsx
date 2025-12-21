import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

interface Citation {
  text: string;
  url: string;
  module: string;
  chapter: string;
  heading?: string;
  score: number;
}

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  timestamp: Date;
}

interface ChatWidgetProps {
  apiBaseUrl?: string;
}

export default function ChatWidget({ apiBaseUrl = 'http://localhost:8000' }: ChatWidgetProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [mode, setMode] = useState<'full_book' | 'selected_text'>('full_book');
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 10) {
        setSelectedText(text);
        setMode('selected_text');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const handleSendMessage = async () => {
    if (!inputValue.trim()) return;

    const userMessage: ChatMessage = {
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(`${apiBaseUrl}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          mode: mode,
          selected_text: mode === 'selected_text' ? selectedText : null,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();

      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: ChatMessage = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again later.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const clearSelection = () => {
    setSelectedText('');
    setMode('full_book');
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Widget */}
      {isOpen && (
        <div className={styles.chatWidget}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>📚 Textbook Assistant</h3>
            <div className={styles.modeSwitch}>
              <button
                className={mode === 'full_book' ? styles.active : ''}
                onClick={() => {
                  setMode('full_book');
                  clearSelection();
                }}
              >
                Full Book
              </button>
              <button
                className={mode === 'selected_text' ? styles.active : ''}
                onClick={() => setMode('selected_text')}
                disabled={!selectedText}
                title={selectedText ? 'Ask about selected text' : 'Select text on the page first'}
              >
                Selected Text
              </button>
            </div>
          </div>

          {/* Selected Text Banner */}
          {selectedText && mode === 'selected_text' && (
            <div className={styles.selectedBanner}>
              <span>📝 {selectedText.substring(0, 50)}...</span>
              <button onClick={clearSelection}>✕</button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I'm your textbook assistant.</p>
                <p>Ask me anything about Physical AI & Humanoid Robotics!</p>
              </div>
            )}

            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>

                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>Sources:</strong>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx}>
                          <a
                            href={source.url}
                            target="_blank"
                            rel="noopener noreferrer"
                          >
                            [{idx + 1}] {source.module} › {source.chapter}
                            {source.heading && ` › ${source.heading}`}
                          </a>
                          <span className={styles.score}>
                            ({Math.round(source.score * 100)}% match)
                          </span>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}

                <div className={styles.timestamp}>
                  {message.timestamp.toLocaleTimeString()}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loader}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.chatInput}>
            <textarea
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={
                mode === 'selected_text'
                  ? 'Ask about the selected text...'
                  : 'Ask a question about the textbook...'
              }
              rows={2}
              disabled={isLoading}
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className={styles.sendButton}
            >
              {isLoading ? '⏳' : '📤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
}

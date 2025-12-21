import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

// Docusaurus Root component wrapper
// This wraps the entire app and allows us to add global components
export default function Root({ children }) {
  // For production, set this to your deployed backend URL
  // For local development, it defaults to localhost:8000
  const apiBaseUrl = typeof window !== 'undefined'
    ? (window as any).RAG_API_URL || 'http://localhost:8000'
    : 'http://localhost:8000';

  return (
    <>
      {children}
      <ChatWidget apiBaseUrl={apiBaseUrl} />
    </>
  );
}

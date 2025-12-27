import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';

// Root wrapper component that wraps the entire Docusaurus application
// This allows us to add global components that persist across all pages
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}

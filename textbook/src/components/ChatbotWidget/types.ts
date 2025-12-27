/**
 * TypeScript type definitions for ChatbotWidget
 */

export interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: Date;
}

export interface Source {
  module: string;
  title: string;
  url: string;
  score: number;
}

export interface QueryResponse {
  answer: string;
  sources: Source[];
  response_time_ms: number;
  tokens_used: number;
}

export interface QueryRequest {
  question: string;
  max_results?: number;
}

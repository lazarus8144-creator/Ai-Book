// RAG API Configuration
// This file is loaded before the app starts
// Edit window.RAG_API_URL to point to your deployed backend

window.RAG_API_URL = window.RAG_API_URL || 'http://localhost:8000';

// For production deployments, you can override this by setting it in your deployment platform:
// Vercel: Add this as an environment variable and inject it during build
// Or manually edit this file before deployment

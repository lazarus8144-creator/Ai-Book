#!/bin/bash

# Quick Start Script for RAG Chatbot + Docusaurus Textbook
# Run this after setting up .env file

set -e

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║              RAG Chatbot Quick Start Verification                     ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""

# Check if we're in the right directory
if [ ! -d "rag-backend" ] || [ ! -d "textbook" ]; then
    echo "❌ Error: Must run from project root directory"
    exit 1
fi

# Step 1: Check .env exists
echo "📋 Step 1: Checking environment configuration..."
if [ ! -f "rag-backend/.env" ]; then
    echo "❌ Error: rag-backend/.env not found"
    echo "   Please copy .env.example to .env and configure API keys:"
    echo "   cd rag-backend && cp .env.example .env"
    exit 1
fi
echo "✅ Environment file found"
echo ""

# Step 2: Check Python virtual environment
echo "📋 Step 2: Checking Python environment..."
if [ ! -d "rag-backend/venv" ]; then
    echo "⚠️  Virtual environment not found. Creating..."
    cd rag-backend
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    cd ..
    echo "✅ Virtual environment created and dependencies installed"
else
    echo "✅ Virtual environment exists"
fi
echo ""

# Step 3: Check Node modules
echo "📋 Step 3: Checking Node.js dependencies..."
if [ ! -d "textbook/node_modules" ]; then
    echo "⚠️  Node modules not found. Installing..."
    cd textbook
    npm install
    cd ..
    echo "✅ Node modules installed"
else
    echo "✅ Node modules exist"
fi
echo ""

# Step 4: Instructions for starting
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 READY TO START!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Open 3 terminal windows and run:"
echo ""
echo "Terminal 1 (Backend):"
echo "  cd rag-backend"
echo "  source venv/bin/activate"
echo "  python app/main.py"
echo ""
echo "Terminal 2 (Ingestion - run once):"
echo "  cd rag-backend"
echo "  source venv/bin/activate"
echo "  python scripts/ingest_docs.py --docs-path ../textbook/docs"
echo ""
echo "Terminal 3 (Frontend):"
echo "  cd textbook"
echo "  npm start"
echo ""
echo "Then open: http://localhost:3000"
echo "Click the 💬 button and ask: \"What is ROS 2?\""
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📖 For detailed instructions, see: SETUP.md"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

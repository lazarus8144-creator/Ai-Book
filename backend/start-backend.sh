#!/bin/bash
# Backend Startup Script with Dependency Check

echo "🔧 Starting Backend Setup..."

# Navigate to backend directory
cd "$(dirname "$0")"

# Check and install dependencies
echo "📦 Checking Python dependencies..."

python3 -c "import sentence_transformers" 2>/dev/null || {
    echo "⚠️  Installing sentence-transformers (this takes 2-3 minutes)..."
    pip3 install --break-system-packages sentence-transformers torch
}

python3 -c "import groq" 2>/dev/null || {
    echo "⚠️  Installing groq..."
    pip3 install --break-system-packages groq
}

python3 -c "import qdrant_client" 2>/dev/null || {
    echo "⚠️  Installing qdrant-client..."
    pip3 install --break-system-packages qdrant-client
}

python3 -c "import pydantic_settings" 2>/dev/null || {
    echo "⚠️  Installing pydantic-settings..."
    pip3 install --break-system-packages pydantic-settings
}

echo "✅ All dependencies installed!"
echo ""
echo "🚀 Starting Backend Server..."
echo "Press Ctrl+C to stop"
echo ""

# Start the backend
python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

#!/bin/bash

echo "🧪 Testing RAG Backend Setup..."
echo ""

# Activate virtual environment
source venv/bin/activate

# Check if FastAPI is installed
echo "✅ Checking FastAPI installation..."
python -c "import fastapi; print(f'FastAPI version: {fastapi.__version__}')" || { echo "❌ FastAPI not installed"; exit 1; }

# Check if dependencies are installed
echo "✅ Checking key dependencies..."
python -c "import openai; print(f'OpenAI version: {openai.__version__}')" || { echo "❌ OpenAI SDK not installed"; exit 1; }
python -c "import qdrant_client; print(f'Qdrant Client version: {qdrant_client.__version__}')" || { echo "❌ Qdrant Client not installed"; exit 1; }

# Check if .env file exists
if [ -f ".env" ]; then
    echo "✅ .env file found"
else
    echo "⚠️  .env file not found. Creating from .env.example..."
    cp .env.example .env
fi

echo ""
echo "🎉 All checks passed!"
echo ""
echo "To start the server, run:"
echo "  cd rag-backend"
echo "  source venv/bin/activate"
echo "  uvicorn app.main:app --reload"
echo ""
echo "Then visit: http://localhost:8000/docs"

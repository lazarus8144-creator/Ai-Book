# RAG Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics textbook chatbot, powered by OpenAI and Qdrant.

## Features

- **Retrieval-Augmented Generation (RAG)**: Answer questions about the textbook using semantic search
- **Modular Agent System**: BookSearchAgent, SelectedTextAgent, CitationAgent
- **Vector Search**: Qdrant Cloud integration for fast semantic search
- **OpenAI Integration**: GPT-4o-mini for answer generation, text-embedding-3-small for embeddings
- **REST API**: FastAPI endpoints for chat queries and document ingestion

## Architecture

```
┌─────────────────────────────────────────┐
│         FastAPI Backend                 │
│  ┌───────────────────────────────────┐ │
│  │  AgentOrchestrator                 │ │
│  │  ├─ BookSearchAgent                │ │
│  │  ├─ SelectedTextAgent              │ │
│  │  └─ CitationAgent                  │ │
│  └───────────────────────────────────┘ │
│  ┌───────────────────────────────────┐ │
│  │  RAG Pipeline                      │ │
│  │  • Ingestion: MD → Embeddings     │ │
│  │  • Retrieval: Query → Search      │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
                │
                ▼
        Qdrant Cloud (Vector DB)
```

## Prerequisites

- Python 3.11+
- OpenAI API key
- Qdrant Cloud account (free tier)
- Docker (optional, for containerized deployment)

## Quick Start

### 1. Clone and Navigate

```bash
cd rag-backend
```

### 2. Create Virtual Environment

```bash
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Configure Environment

```bash
cp .env.example .env
# Edit .env and add your API keys:
# - OPENAI_API_KEY
# - QDRANT_URL
# - QDRANT_API_KEY
```

### 5. Run the Server

```bash
uvicorn app.main:app --reload
```

Server will start at: `http://localhost:8000`

### 6. Verify Installation

Open your browser: `http://localhost:8000/docs` (FastAPI auto-generated docs)

Or test the health endpoint:

```bash
curl http://localhost:8000/api/v1/health
```

## Docker Setup (Alternative)

### Build and Run with Docker Compose

```bash
# Copy environment file
cp .env.example .env
# Edit .env with your credentials

# Build and start
docker-compose up --build

# Run in background
docker-compose up -d

# View logs
docker-compose logs -f backend

# Stop
docker-compose down
```

## API Endpoints

### Chat Query

```bash
POST /api/v1/chat/query
```

**Request:**
```json
{
  "query": "What is ROS 2?",
  "mode": "full_book",
  "selected_text": null,
  "filters": {}
}
```

**Response:**
```json
{
  "answer": "ROS 2 is a robot operating system...",
  "sources": [
    {
      "text": "...",
      "url": "/module-1-ros2/01-introduction#what-is-ros-2",
      "citation": "Module 1: ROS 2 > Introduction > What is ROS 2?",
      "relevance_score": 0.92
    }
  ],
  "agent_used": "book_search",
  "response_time_ms": 1847
}
```

### Document Ingestion

```bash
POST /api/v1/ingest/documents
```

**Request:**
```json
{
  "source_directory": "../textbook/docs",
  "force_reindex": false
}
```

Or use the CLI script:

```bash
python scripts/ingest_docs.py --source ../textbook/docs
```

### Health Check

```bash
GET /api/v1/health
```

**Response:**
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "openai": "available"
  }
}
```

### Collection Stats

```bash
GET /api/v1/collections/stats
```

**Response:**
```json
{
  "collection": "robotics_textbook",
  "vectors_count": 287,
  "indexed": true
}
```

## Project Structure

```
rag-backend/
├── app/
│   ├── main.py                 # FastAPI app entry point
│   ├── config.py               # Configuration (env vars)
│   ├── api/
│   │   └── routes/
│   │       ├── chat.py         # Chat endpoints
│   │       ├── ingest.py       # Ingestion endpoints
│   │       └── health.py       # Health check
│   ├── agents/
│   │   ├── base.py             # BaseAgent interface
│   │   ├── orchestrator.py     # Agent router
│   │   ├── book_search.py      # BookSearchAgent
│   │   ├── selected_text.py    # SelectedTextAgent
│   │   └── citation.py         # CitationAgent
│   ├── rag/
│   │   ├── ingestion.py        # MD → embeddings → Qdrant
│   │   ├── retrieval.py        # Query → search → answer
│   │   ├── chunking.py         # Text splitting
│   │   └── embeddings.py       # OpenAI embeddings
│   ├── vector_store/
│   │   ├── client.py           # Qdrant client
│   │   ├── schema.py           # Collection schema
│   │   └── operations.py       # CRUD operations
│   ├── models/
│   │   ├── chat.py             # Chat models
│   │   ├── documents.py        # Document models
│   │   └── agents.py           # Agent models
│   └── utils/
│       ├── markdown_parser.py  # Parse markdown
│       └── logging_config.py   # Logging setup
├── scripts/
│   └── ingest_docs.py          # Ingestion CLI
├── tests/
│   ├── test_agents.py          # Agent tests
│   ├── test_rag_pipeline.py    # RAG tests
│   └── test_api.py             # API tests
├── requirements.txt
├── Dockerfile
├── docker-compose.yml
├── .env.example
└── README.md
```

## Development

### Run Tests

```bash
# All tests
pytest

# With coverage
pytest --cov=app tests/

# Specific test file
pytest tests/test_agents.py -v
```

### Code Formatting

```bash
# Format with black
black app/ tests/

# Lint with ruff
ruff check app/ tests/
```

### Interactive API Docs

FastAPI auto-generates interactive documentation:

- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Deployment

### Railway (Recommended)

1. Create Railway account: https://railway.app
2. Connect GitHub repository
3. Add environment variables (OPENAI_API_KEY, QDRANT_URL, etc.)
4. Deploy automatically on push to main

### Render

1. Create Render account: https://render.com
2. Create new Web Service
3. Connect repository
4. Set build command: `pip install -r requirements.txt`
5. Set start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### Fly.io

1. Install flyctl: `curl -L https://fly.io/install.sh | sh`
2. Login: `fly auth login`
3. Launch: `fly launch`
4. Deploy: `fly deploy`

## Environment Variables

| Variable | Description | Required | Default |
|----------|-------------|----------|---------|
| `OPENAI_API_KEY` | OpenAI API key | Yes | - |
| `OPENAI_MODEL` | GPT model to use | No | gpt-4o-mini |
| `OPENAI_EMBEDDING_MODEL` | Embedding model | No | text-embedding-3-small |
| `QDRANT_URL` | Qdrant Cloud URL | Yes | - |
| `QDRANT_API_KEY` | Qdrant API key | Yes | - |
| `QDRANT_COLLECTION_NAME` | Collection name | No | robotics_textbook |
| `CORS_ORIGINS` | Allowed CORS origins | Yes | http://localhost:3000 |
| `LOG_LEVEL` | Logging level | No | INFO |
| `CHUNK_SIZE` | Max tokens per chunk | No | 500 |
| `TOP_K_RESULTS` | Results to retrieve | No | 5 |

## Troubleshooting

### Import Errors

```bash
# Make sure you're in the rag-backend directory
cd rag-backend

# Activate virtual environment
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt
```

### Qdrant Connection Failed

- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check Qdrant Cloud dashboard for cluster status
- Test connection: `curl -H "api-key: YOUR_KEY" https://YOUR_URL/collections`

### OpenAI Rate Limit

- Check your OpenAI usage dashboard
- Implement rate limiting or caching
- Consider using smaller models for development

### CORS Errors

- Add your frontend domain to `CORS_ORIGINS` in `.env`
- Restart the server after changing environment variables

## Contributing

1. Create a feature branch
2. Make changes
3. Run tests: `pytest`
4. Format code: `black . && ruff check .`
5. Submit pull request

## License

MIT License - See LICENSE file for details

## Support

For issues or questions:
- GitHub Issues: https://github.com/yourusername/Ai-native-text-book/issues
- Documentation: `/specs/002-rag-chatbot/`

# Implementation Plan: RAG Chatbot for Docusaurus Textbook

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

## Summary

Build an intelligent RAG (Retrieval-Augmented Generation) chatbot system that enables learners to ask questions about the Physical AI & Humanoid Robotics textbook. The system consists of:

1. **Backend**: FastAPI service with modular agent architecture (BookSearchAgent, SelectedTextAgent, CitationAgent)
2. **Vector Store**: Qdrant Cloud for semantic search over textbook content
3. **LLM Layer**: OpenAI GPT models for answer generation with citations
4. **Frontend**: React ChatWidget component embedded in existing Docusaurus site
5. **Ingestion Pipeline**: Markdown → Chunks → Embeddings → Qdrant

**Key Innovation**: Matrix-style modular agent system where each agent is an independent, composable skill that can be invoked by an orchestrator based on query context.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.6+ (frontend)
**Primary Dependencies**:
- Backend: FastAPI 0.115+, Qdrant Client 1.11+, OpenAI SDK 1.54+, Pydantic 2.0+, LangChain 0.1+ (text splitting)
- Frontend: React 19 (existing), Axios 1.6+, Docusaurus 3.9.2 (existing)

**Storage**:
- Vector Database: Qdrant Cloud (Free Tier: 1GB, ~500K vectors)
- Ephemeral: In-memory session storage for chat history (no persistence)

**Testing**:
- Backend: pytest with pytest-asyncio for async tests, httpx for API testing
- Frontend: Playwright (existing E2E setup from Task 1), Jest for unit tests
- Integration: End-to-end tests covering full RAG pipeline

**Target Platform**:
- Backend: Linux server (Docker container), deployable to Railway/Render/Fly.io
- Frontend: Web (already deployed on Vercel/GitHub Pages)

**Project Type**: Web application (FastAPI backend + React frontend)

**Performance Goals**:
- P95 response latency: <3 seconds (vector search + LLM generation)
- Throughput: 10 concurrent users minimum
- Ingestion: Full textbook (30 files) in <5 minutes

**Constraints**:
- OpenAI API budget: <$50/month (estimate: 1000 queries/month @ $0.03/query avg)
- Qdrant Cloud: Must fit in free tier (1GB storage)
- CORS: Backend must allow requests from Docusaurus domain

**Scale/Scope**:
- Documents: 30 markdown files (~50K words)
- Vectors: 200-400 chunks (estimated)
- Users: 10-100 concurrent learners
- Queries: 1000/month estimated

## Constitution Check

*GATE: Must pass before implementation*

| Principle | Status | Notes |
|-----------|--------|-------|
| Educational Excellence First | ✅ Pass | Chatbot enhances learning with Q&A, citations reinforce textbook content |
| Progressive Enhancement | ✅ Pass | MVP focuses on core P1 stories (full-book search + citations) |
| User-Centric Personalization | ⏸️ Deferred | No auth/personalization in MVP (Phase 2) |
| Multilingual Accessibility | ⏸️ Deferred | English-only for MVP (Phase 2) |
| AI-Native Development | ✅ Pass | Spec-driven approach, modular agent architecture |
| Tech Stack Compliance | ✅ Pass | FastAPI + OpenAI + Qdrant as specified |
| Performance Standards | ✅ Pass | <3s response time target |
| Accessibility | ✅ Pass | Chat widget keyboard accessible, responsive |

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── spec.md              # Feature specification (DONE)
├── plan.md              # This file - architectural plan
├── tasks.md             # Implementation tasks (to be generated)
├── research.md          # Technology research and decisions (optional)
└── data-model.md        # Vector schema and data structures (optional)
```

### Source Code (repository root)

```text
/
├── textbook/                           # Existing Docusaurus (Task 1)
│   ├── docs/                           # Markdown content (source for ingestion)
│   ├── src/
│   │   └── components/
│   │       └── ChatWidget/             # NEW: RAG chatbot UI
│   │           ├── index.tsx           # Main widget component
│   │           ├── ChatInterface.tsx   # Chat messages + input
│   │           ├── ChatMessage.tsx     # Individual message bubble
│   │           ├── CitationList.tsx    # Source citations display
│   │           ├── ContextSelector.tsx # Selected text mode UI
│   │           ├── LoadingIndicator.tsx
│   │           ├── api.ts              # Backend API client
│   │           ├── types.ts            # TypeScript interfaces
│   │           └── styles.module.css
│   ├── docusaurus.config.ts
│   └── package.json
│
├── rag-backend/                        # NEW: FastAPI backend
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py                     # FastAPI application entry
│   │   ├── config.py                   # Settings (env vars, Qdrant/OpenAI config)
│   │   │
│   │   ├── api/                        # API layer
│   │   │   ├── __init__.py
│   │   │   ├── dependencies.py         # Dependency injection (DB, clients)
│   │   │   └── routes/
│   │   │       ├── __init__.py
│   │   │       ├── chat.py             # POST /chat/query endpoint
│   │   │       ├── ingest.py           # POST /ingest/documents (admin)
│   │   │       └── health.py           # GET /health, /collections/stats
│   │   │
│   │   ├── agents/                     # Modular agent system
│   │   │   ├── __init__.py
│   │   │   ├── base.py                 # BaseAgent abstract class
│   │   │   ├── orchestrator.py         # AgentOrchestrator (routing logic)
│   │   │   ├── book_search.py          # BookSearchAgent
│   │   │   ├── selected_text.py        # SelectedTextAgent
│   │   │   └── citation.py             # CitationAgent
│   │   │
│   │   ├── rag/                        # RAG pipeline
│   │   │   ├── __init__.py
│   │   │   ├── ingestion.py            # Markdown → chunks → embeddings → Qdrant
│   │   │   ├── retrieval.py            # Query → vector search → context building
│   │   │   ├── chunking.py             # Text chunking strategies
│   │   │   └── embeddings.py           # OpenAI embedding wrapper
│   │   │
│   │   ├── vector_store/               # Qdrant integration
│   │   │   ├── __init__.py
│   │   │   ├── client.py               # QdrantClient wrapper
│   │   │   ├── schema.py               # Collection schema definition
│   │   │   └── operations.py           # Upsert, search, delete operations
│   │   │
│   │   ├── models/                     # Pydantic models
│   │   │   ├── __init__.py
│   │   │   ├── chat.py                 # ChatQuery, ChatResponse, Citation
│   │   │   ├── documents.py            # DocumentChunk, IngestionRequest
│   │   │   └── agents.py               # AgentContext, AgentResponse
│   │   │
│   │   └── utils/                      # Utilities
│   │       ├── __init__.py
│   │       ├── markdown_parser.py      # Parse markdown + metadata
│   │       └── logging_config.py       # Structured logging setup
│   │
│   ├── scripts/
│   │   ├── ingest_docs.py              # CLI script for ingestion
│   │   └── test_retrieval.py           # Manual retrieval testing
│   │
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── conftest.py                 # Pytest fixtures
│   │   ├── test_agents.py              # Agent unit tests
│   │   ├── test_rag_pipeline.py        # RAG integration tests
│   │   ├── test_api.py                 # API endpoint tests
│   │   └── test_vector_store.py        # Qdrant operations tests
│   │
│   ├── .env.example                    # Environment variables template
│   ├── requirements.txt                # Python dependencies
│   ├── Dockerfile                      # Container image
│   ├── docker-compose.yml              # Local dev with Qdrant
│   └── README.md                       # Backend setup instructions
│
├── specs/
│   ├── 001-docusaurus-textbook/        # Task 1 (existing)
│   └── 002-rag-chatbot/                # Task 2 (this feature)
│
└── README.md                           # Root project documentation
```

**Structure Decision**: Web application structure (backend + frontend) chosen because:
1. Backend (FastAPI) and frontend (Docusaurus/React) are separate concerns
2. Backend can be deployed independently to cloud provider
3. Frontend already exists from Task 1, we're only adding ChatWidget component
4. Clear separation enables independent scaling and testing

## Architecture Design

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      Docusaurus Frontend                        │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ ChatWidget Component (React/TypeScript)                    │ │
│  │                                                             │ │
│  │  [User Input] ──► [API Client] ──► [Message Display]      │ │
│  │                         │                                   │ │
│  │                         │ (HTTP POST)                       │ │
│  │                         ▼                                   │ │
│  │              [Backend API Endpoint]                        │ │
│  │                         │                                   │ │
│  │  [Citations] ◄── [Response Parser] ◄── [JSON Response]    │ │
│  └───────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ HTTPS/CORS
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       FastAPI Backend                           │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ API Layer (routes/chat.py)                                 │ │
│  │   POST /api/v1/chat/query                                  │ │
│  └───────────────────────────────────────────────────────────┘ │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ AgentOrchestrator                                          │ │
│  │  • Analyze query mode (full_book | selected_text)         │ │
│  │  • Route to appropriate agent                              │ │
│  └───────────────────────────────────────────────────────────┘ │
│           │                    │                    │           │
│           ▼                    ▼                    ▼           │
│  ┌─────────────┐    ┌──────────────────┐    ┌──────────────┐  │
│  │BookSearch   │    │SelectedText      │    │Citation      │  │
│  │Agent        │    │Agent             │    │Agent         │  │
│  └─────────────┘    └──────────────────┘    └──────────────┘  │
│           │                    │                    │           │
│           └────────────────────┴────────────────────┘           │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ RAG Pipeline (retrieval.py)                                │ │
│  │  1. Generate query embedding (OpenAI)                      │ │
│  │  2. Vector search (Qdrant)                                 │ │
│  │  3. Build context from top-k chunks                        │ │
│  │  4. LLM completion with context (OpenAI)                   │ │
│  │  5. Format citations                                       │ │
│  └───────────────────────────────────────────────────────────┘ │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │ Vector Store Client (Qdrant)                               │ │
│  └───────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Qdrant Cloud (Free Tier)                   │
│  Collection: "robotics_textbook"                                │
│  • Vectors: 1536 dims (text-embedding-3-small)                  │
│  • Payload: {text, module, chapter, heading, url, ...}          │
│  • ~200-400 vectors (estimated)                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

#### Ingestion Flow (One-time / On Content Update)

```
1. Admin runs: python scripts/ingest_docs.py
   │
   ▼
2. Load all markdown files from textbook/docs/**/*.md
   │
   ▼
3. For each file:
   a. Parse frontmatter (module, chapter metadata)
   b. Extract text content
   c. Split into chunks (max 500 tokens, respect headings)
   │
   ▼
4. Generate embeddings for each chunk
   - API: OpenAI text-embedding-3-small
   - Cost: ~$0.00002 per 1K tokens
   - Batch process to optimize API calls
   │
   ▼
5. Upsert to Qdrant
   - Collection: robotics_textbook
   - Vector: embedding (1536 dims)
   - Payload: {
       "text": "chunk content",
       "module": "module-1-ros2",
       "chapter": "01-introduction",
       "heading": "What is ROS 2?",
       "url": "/module-1-ros2/01-introduction#what-is-ros-2",
       "chunk_index": 0,
       "embedding_model": "text-embedding-3-small"
     }
   │
   ▼
6. Return ingestion report
   - Files processed: 30
   - Chunks created: 287
   - Embeddings generated: 287
   - Qdrant vectors: 287
```

#### Query Flow (Real-time)

```
1. User submits query via ChatWidget
   - Query: "What is ROS 2?"
   - Mode: "full_book" or "selected_text"
   - Selected text: (if mode = selected_text)
   │
   ▼
2. Frontend sends POST /api/v1/chat/query
   {
     "query": "What is ROS 2?",
     "mode": "full_book",
     "selected_text": null,
     "filters": {}
   }
   │
   ▼
3. Backend: AgentOrchestrator routes to BookSearchAgent
   │
   ▼
4. BookSearchAgent.execute():
   a. Generate query embedding (OpenAI)
   b. Search Qdrant (cosine similarity, top_k=5)
   c. Retrieve top 5 matching chunks
   │
   ▼
5. Build context prompt:
   """
   You are a helpful tutor for the Physical AI & Humanoid Robotics textbook.
   Answer the question using ONLY the following context. Include citations.

   Context:
   [1] (Module 1: ROS 2 > Introduction > What is ROS 2?)
   ROS 2 is a robot operating system...

   [2] (Module 1: ROS 2 > Introduction > ROS 2 Architecture)
   The architecture consists of nodes and topics...

   Question: What is ROS 2?

   Answer with citations [1], [2], etc.
   """
   │
   ▼
6. Call OpenAI GPT-4o-mini (or GPT-4o)
   - Model: gpt-4o-mini
   - Max tokens: 500
   - Temperature: 0.3 (deterministic)
   │
   ▼
7. Parse LLM response
   - Extract answer text
   - Extract citation numbers [1], [2]
   - Map citations to chunk metadata
   │
   ▼
8. CitationAgent formats sources:
   [
     {
       "text": "ROS 2 is a robot operating system...",
       "url": "/module-1-ros2/01-introduction#what-is-ros-2",
       "citation": "Module 1: ROS 2 > Introduction > What is ROS 2?",
       "relevance_score": 0.92
     }
   ]
   │
   ▼
9. Return JSON response:
   {
     "answer": "ROS 2 is a robot operating system... [1][2]",
     "sources": [...],
     "agent_used": "book_search",
     "response_time_ms": 1847
   }
   │
   ▼
10. Frontend displays answer + clickable citations
```

### Agent System Design (Matrix-Style Skills)

#### BaseAgent Interface

```python
from abc import ABC, abstractmethod
from typing import List
from app.models.agents import AgentContext, AgentResponse

class BaseAgent(ABC):
    """Abstract base class for all agents/skills"""

    @abstractmethod
    async def execute(self, context: AgentContext) -> AgentResponse:
        """Execute the agent's primary function"""
        pass

    @abstractmethod
    def get_capabilities(self) -> List[str]:
        """Return list of capabilities this agent provides"""
        pass

    @abstractmethod
    def get_name(self) -> str:
        """Return agent name for logging/routing"""
        pass
```

#### Agent Implementations

**1. BookSearchAgent**
- **Capability**: Full-book semantic search
- **Input**: User query, optional filters (module, chapter)
- **Process**:
  1. Embed query
  2. Search Qdrant (no text restrictions)
  3. Retrieve top-k chunks
  4. Build context + call LLM
  5. Return answer with sources
- **Output**: Answer + citations

**2. SelectedTextAgent**
- **Capability**: Context-restricted search
- **Input**: User query, selected_text
- **Process**:
  1. Embed query + selected_text together
  2. Search Qdrant with filters (same chapter/module as selected text)
  3. Retrieve top-k chunks (limited scope)
  4. Build constrained context + call LLM
  5. Return answer with disclaimer ("Based on selected text...")
- **Output**: Answer + citations (constrained)

**3. CitationAgent**
- **Capability**: Source attribution and formatting
- **Input**: Retrieved chunks with metadata
- **Process**:
  1. Extract metadata (module, chapter, heading, url)
  2. Format citation:
     - Inline: [1], [2]
     - Footer: "Module 1: ROS 2 > Introduction > What is ROS 2?"
     - URL: /module-1-ros2/01-introduction#what-is-ros-2
  3. Compute relevance scores
- **Output**: Formatted citation objects

#### AgentOrchestrator

```python
class AgentOrchestrator:
    """Routes queries to appropriate agents"""

    def __init__(self):
        self.agents = {
            "book_search": BookSearchAgent(),
            "selected_text": SelectedTextAgent(),
            "citation": CitationAgent()
        }

    async def route(self, query: ChatQuery) -> ChatResponse:
        # Routing logic
        if query.mode == "selected_text" and query.selected_text:
            agent = self.agents["selected_text"]
        else:
            agent = self.agents["book_search"]

        # Build context
        context = AgentContext(
            query=query.query,
            mode=query.mode,
            selected_text=query.selected_text,
            filters=query.filters
        )

        # Execute agent
        response = await agent.execute(context)

        # Format citations
        citation_agent = self.agents["citation"]
        formatted_sources = citation_agent.format_citations(response.sources)

        return ChatResponse(
            answer=response.answer,
            sources=formatted_sources,
            agent_used=agent.get_name(),
            response_time_ms=response.response_time_ms
        )
```

### Vector Database Schema (Qdrant)

#### Collection Configuration

```python
from qdrant_client.models import Distance, VectorParams

collection_name = "robotics_textbook"

vector_config = VectorParams(
    size=1536,  # text-embedding-3-small dimensions
    distance=Distance.COSINE  # Cosine similarity for semantic search
)
```

#### Document Payload Schema

```python
{
    "id": "550e8400-e29b-41d4-a716-446655440000",  # UUID
    "text": "ROS 2 is a robot operating system that provides...",  # Chunk text
    "metadata": {
        "module": "module-1-ros2",
        "chapter": "01-introduction",
        "heading": "What is ROS 2?",
        "url": "/module-1-ros2/01-introduction#what-is-ros-2",
        "file_path": "docs/module-1-ros2/01-introduction.md",
        "chunk_index": 0,  # Position in document
        "total_chunks": 5,  # Total chunks from this document
        "embedding_model": "text-embedding-3-small",
        "ingestion_timestamp": "2025-12-16T10:30:00Z"
    }
}
```

#### Indexes for Filtering

```python
# Payload indexes for fast filtering
- module (keyword index) → filter queries by module
- chapter (keyword index) → filter queries by chapter
- heading (text index) → filter queries by heading
```

### API Design

#### Endpoint: POST /api/v1/chat/query

**Request:**
```json
{
  "query": "What is ROS 2?",
  "mode": "full_book",  // or "selected_text"
  "selected_text": null,  // or "ROS 2 is a robot operating system..."
  "filters": {
    "module": "module-1-ros2"  // optional
  }
}
```

**Response:**
```json
{
  "answer": "ROS 2 is a robot operating system that provides communication infrastructure for robots [1][2]. It consists of nodes that communicate via topics and services [2].",
  "sources": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "text": "ROS 2 is a robot operating system...",
      "url": "/module-1-ros2/01-introduction#what-is-ros-2",
      "citation": "Module 1: ROS 2 > Introduction > What is ROS 2?",
      "module": "module-1-ros2",
      "chapter": "01-introduction",
      "heading": "What is ROS 2?",
      "relevance_score": 0.92
    },
    {
      "id": "550e8400-e29b-41d4-a716-446655440001",
      "text": "The architecture consists of nodes and topics...",
      "url": "/module-1-ros2/01-introduction#ros-2-architecture",
      "citation": "Module 1: ROS 2 > Introduction > ROS 2 Architecture",
      "module": "module-1-ros2",
      "chapter": "01-introduction",
      "heading": "ROS 2 Architecture",
      "relevance_score": 0.87
    }
  ],
  "agent_used": "book_search",
  "response_time_ms": 1847,
  "timestamp": "2025-12-16T10:35:42Z"
}
```

**Error Response:**
```json
{
  "error": {
    "code": "QDRANT_UNAVAILABLE",
    "message": "Vector database is temporarily unavailable. Please try again later.",
    "details": "Connection timeout to Qdrant Cloud"
  },
  "timestamp": "2025-12-16T10:35:42Z"
}
```

#### Endpoint: POST /api/v1/ingest/documents (Admin)

**Request:**
```json
{
  "source_directory": "textbook/docs",
  "force_reindex": false  // if true, delete existing collection and re-create
}
```

**Response:**
```json
{
  "status": "success",
  "summary": {
    "files_processed": 30,
    "chunks_created": 287,
    "embeddings_generated": 287,
    "vectors_upserted": 287,
    "errors": []
  },
  "duration_seconds": 142,
  "timestamp": "2025-12-16T10:40:00Z"
}
```

#### Endpoint: GET /api/v1/health

**Response:**
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "openai": "available"
  },
  "timestamp": "2025-12-16T10:45:00Z"
}
```

#### Endpoint: GET /api/v1/collections/stats

**Response:**
```json
{
  "collection": "robotics_textbook",
  "vectors_count": 287,
  "segments_count": 1,
  "disk_data_size_bytes": 4567890,
  "ram_data_size_bytes": 1234567,
  "indexed": true,
  "timestamp": "2025-12-16T10:46:00Z"
}
```

### Frontend Integration Strategy

#### ChatWidget Component Architecture

```tsx
// textbook/src/components/ChatWidget/index.tsx

export default function ChatWidget() {
  return (
    <div className={styles.chatWidget}>
      <ChatHeader />
      <ChatMessages messages={messages} />
      <ChatInput onSend={handleSend} />
      <CitationList sources={sources} />
    </div>
  );
}
```

#### Text Selection Integration

```tsx
// Detect text selection on page
useEffect(() => {
  const handleSelection = () => {
    const selectedText = window.getSelection()?.toString();
    if (selectedText && selectedText.length > 20) {
      setSelectedContext(selectedText);
      setMode('selected_text');
    }
  };

  document.addEventListener('mouseup', handleSelection);
  return () => document.removeEventListener('mouseup', handleSelection);
}, []);
```

#### API Client

```typescript
// textbook/src/components/ChatWidget/api.ts

export async function queryChat(
  query: string,
  mode: 'full_book' | 'selected_text',
  selectedText?: string
): Promise<ChatResponse> {
  const response = await axios.post(
    `${API_BASE_URL}/api/v1/chat/query`,
    {
      query,
      mode,
      selected_text: selectedText,
      filters: {}
    }
  );
  return response.data;
}
```

### Deployment Strategy

#### Backend Deployment (Railway / Render / Fly.io)

**Option 1: Railway (Recommended)**
- Pros: Easy setup, free tier, automatic HTTPS, environment variables
- Steps:
  1. Create Dockerfile
  2. Connect GitHub repo to Railway
  3. Set environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
  4. Deploy (automatic on push to main)

**Option 2: Render**
- Pros: Free tier, managed PostgreSQL (if needed later), auto-deploy
- Similar setup to Railway

**Option 3: Fly.io**
- Pros: Global edge deployment, generous free tier
- Requires `fly.toml` configuration

#### Environment Variables

```bash
# .env (backend)
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xyz.qdrant.tech:6333
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=robotics_textbook
CORS_ORIGINS=https://your-textbook.vercel.app,http://localhost:3000
LOG_LEVEL=INFO
```

#### Frontend Deployment (No Change)

- Docusaurus site already deployed on Vercel/GitHub Pages (Task 1)
- ChatWidget component added to existing codebase
- API calls to deployed backend URL

### Technology Decisions

#### Why FastAPI?
- **Async support**: Native async/await for concurrent requests
- **Type safety**: Pydantic models for request/response validation
- **Performance**: Fast startup, low latency (<10ms overhead)
- **Developer experience**: Automatic OpenAPI docs, easy testing

#### Why Qdrant?
- **Free tier**: 1GB storage, perfect for textbook (~200-400 vectors)
- **Performance**: <50ms search latency for small collections
- **Payload filtering**: Filter by module/chapter during search
- **Cloud-native**: Managed service, no infrastructure overhead

#### Why OpenAI text-embedding-3-small?
- **Cost**: $0.00002 per 1K tokens (cheapest OpenAI embedding)
- **Quality**: 1536 dims, good semantic understanding
- **Speed**: Fast embedding generation (<100ms for typical query)
- **Compatibility**: Works well with Qdrant cosine similarity

#### Why GPT-4o-mini?
- **Cost**: $0.150 per 1M input tokens, $0.600 per 1M output tokens
- **Quality**: High-quality answers with reasoning
- **Speed**: Fast generation (<2s for typical answer)
- **Fallback**: Can use GPT-4o for complex queries if needed

#### Why LangChain for Chunking?
- **RecursiveCharacterTextSplitter**: Respects markdown structure (headings, paragraphs)
- **Token counting**: Accurate token limits (500 tokens/chunk)
- **Metadata preservation**: Keeps frontmatter metadata during chunking
- **Battle-tested**: Widely used, well-documented

### Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| OpenAI API costs exceed budget | Implement response caching (Redis later), monitor usage daily, set API spending limits |
| Qdrant free tier storage limit hit | Optimize chunking (reduce overlap), use smaller embeddings if needed, monitor vector count |
| Backend downtime affects all users | Implement health checks, deploy to reliable platform (Railway 99.9% uptime), add error fallback UI |
| Slow response times (>3s) | Optimize retrieval (reduce top_k), use GPT-4o-mini (faster), cache common queries |
| Citation links break when content changes | Use stable URL structure (heading IDs), test ingestion after content updates |
| Text selection fails on mobile | Provide manual "Ask about selection" button, test across browsers |

### Performance Optimization

#### Caching Strategy (Future)
- **Query caching**: Cache responses for common questions (Redis)
- **Embedding caching**: Cache embeddings for repeated queries
- **Vector search caching**: Cache search results for identical queries

#### Batch Processing
- **Ingestion**: Process multiple files in parallel (asyncio)
- **Embeddings**: Batch embed multiple chunks in single API call (OpenAI batch API)

#### Query Optimization
- **Top-k tuning**: Start with top_k=5, adjust based on answer quality
- **Reranking**: Use cross-encoder to rerank retrieved chunks (optional, adds latency)
- **Hybrid search**: Combine vector search with keyword search (future enhancement)

## Next Steps

1. ✅ Spec complete → Create tasks.md with detailed implementation checklist
2. Set up FastAPI backend skeleton
3. Implement Qdrant client and test connection
4. Build ingestion pipeline and test with 1-2 markdown files
5. Implement agent system (BaseAgent, Orchestrator, BookSearchAgent)
6. Create API endpoints and test with curl/Postman
7. Build ChatWidget React component
8. Integrate ChatWidget into Docusaurus
9. Run full ingestion on textbook
10. End-to-end testing and deployment

## Open Questions

1. **Chunking strategy**: Should we use fixed token count (500) or adaptive based on headings?
   - **Recommendation**: Start with fixed 500 tokens, monitor retrieval quality, adjust if needed
2. **Reranking**: Should we implement reranking with a cross-encoder?
   - **Recommendation**: Defer to Phase 2 (adds 100-200ms latency, start simple)
3. **Conversation memory**: Should we persist chat history across sessions?
   - **Recommendation**: No for MVP (stateless), add in Phase 2 with auth
4. **Multiple books**: Should we design for multiple textbooks from the start?
   - **Recommendation**: Yes, use collection naming: `{book_slug}_textbook`

# Implementation Tasks: RAG Chatbot

## Document Metadata
- **Feature ID:** 002-rag-chatbot
- **Status:** Ready for Implementation
- **Version:** 1.0.0
- **Created:** 2025-12-26
- **Total Tasks:** 19
- **Estimated Duration:** 3-4 days (43.5 hours)

## Task Organization

**Phases:** 5
- Phase 1: Backend Foundation (9 hours)
- Phase 2: RAG Pipeline (10 hours)
- Phase 3: Document Ingestion (4 hours)
- Phase 4: Frontend Integration (9 hours)
- Phase 5: Deployment & Testing (11.5 hours)

**Dependencies:**
- Textbook content complete ✅
- OpenAI API access ✅
- Development environment ready

**Critical Path:**
```
T001 → T002 → T003/T004 → T006 → T007 → T008 → T009 → T010 → T012 → T014 → T015
```

---

## Phase 1: Backend Foundation (Day 1)

### T001: Initialize FastAPI Backend ⚙️ SETUP
**Priority:** P0 (Critical Path)
**Estimated Time:** 2 hours
**Dependencies:** None
**Assigned To:** Backend Dev

#### Description
Create FastAPI project structure with proper configuration, CORS middleware, and basic health endpoint.

#### Implementation Steps

1. **Create directory structure**
   ```bash
   mkdir -p backend/app/{routers,services,utils}
   mkdir -p backend/{scripts,tests}
   cd backend
   ```

2. **Initialize Python virtual environment**
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Create `requirements.txt`**
   ```txt
   fastapi==0.110.0
   uvicorn[standard]==0.27.0
   pydantic-settings==2.1.0
   openai==1.12.0
   qdrant-client==1.7.3
   slowapi==0.1.9
   loguru==0.7.2
   python-multipart==0.0.9
   ```

4. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

5. **Create `app/main.py`**
   - Initialize FastAPI app
   - Configure CORS for localhost:3000
   - Add basic root endpoint
   - Configure Swagger docs at /docs

6. **Create `app/config.py`**
   - Use pydantic-settings for env vars
   - Define all required environment variables
   - Add validation

7. **Create `.env.example`**
   - Template for environment variables
   - Document each variable

8. **Test server starts**
   ```bash
   uvicorn app.main:app --reload
   ```

#### Acceptance Criteria
- [ ] FastAPI app runs on http://localhost:8000
- [ ] GET /docs returns Swagger UI
- [ ] CORS allows localhost:3000
- [ ] Environment variables loaded from .env
- [ ] No errors in console on startup

#### Test Cases
```python
# tests/test_main.py
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_app_starts():
    response = client.get("/")
    assert response.status_code in [200, 404]

def test_docs_accessible():
    response = client.get("/docs")
    assert response.status_code == 200
    assert "swagger" in response.text.lower()

def test_cors_headers():
    response = client.options("/api/v1/query", 
        headers={"Origin": "http://localhost:3000"})
    assert "access-control-allow-origin" in response.headers
```

#### Files Created
- `backend/app/main.py`
- `backend/app/config.py`
- `backend/app/__init__.py`
- `backend/requirements.txt`
- `backend/.env.example`
- `backend/.gitignore`

---

### T002: Set up Qdrant Cloud ⚙️ SETUP
**Priority:** P0 (Critical Path)
**Estimated Time:** 1 hour
**Dependencies:** None (parallel with T001)
**Assigned To:** Backend Dev

#### Description
Create Qdrant Cloud account, configure vector database cluster, and implement connection wrapper.

#### Implementation Steps

1. **Create Qdrant Cloud account**
   - Sign up at https://cloud.qdrant.io
   - Verify email

2. **Create cluster**
   - Choose free tier (1GB storage)
   - Region: Closest to users
   - Note cluster URL and API key

3. **Add credentials to `.env`**
   ```bash
   QDRANT_URL=https://xxx-yyy.qdrant.io
   QDRANT_API_KEY=your-api-key-here
   QDRANT_COLLECTION_NAME=textbook_chunks
   ```

4. **Create `app/services/vector_store.py`**
   - Initialize Qdrant client
   - Implement collection creation
   - Configure 1536 dimensions (text-embedding-3-small)
   - Use cosine distance metric

5. **Implement connection test**
   - Check if collection exists
   - Create if missing
   - Validate connection

#### Acceptance Criteria
- [ ] Qdrant cluster accessible via API
- [ ] Collection auto-created on first run
- [ ] Connection test passes
- [ ] Collection configured with correct dimensions (1536)
- [ ] Cosine distance metric set

#### Test Cases
```python
# tests/test_vector_store.py
from app.services.vector_store import VectorStore

def test_qdrant_connection():
    store = VectorStore()
    # Should connect without error
    assert store.client is not None

def test_collection_exists():
    store = VectorStore()
    collections = store.client.get_collections().collections
    collection_names = [c.name for c in collections]
    assert "textbook_chunks" in collection_names

def test_collection_config():
    store = VectorStore()
    collection_info = store.client.get_collection("textbook_chunks")
    assert collection_info.config.params.vectors.size == 1536
    assert collection_info.config.params.vectors.distance == "Cosine"
```

#### Files Created
- `backend/app/services/__init__.py`
- `backend/app/services/vector_store.py`
- Updated `.env.example` with Qdrant variables

---

### T003: Implement Document Processor 🏗️ CORE
**Priority:** P0 (Critical Path)
**Estimated Time:** 4 hours
**Dependencies:** None
**Assigned To:** Backend Dev

#### Description
Create semantic chunking logic for markdown files with header-based splitting, code block preservation, and metadata extraction.

#### Implementation Steps

1. **Create `app/services/document_processor.py`**

2. **Implement frontmatter parser**
   ```python
   import yaml
   
   def parse_frontmatter(content: str) -> dict:
       # Extract YAML between --- markers
       # Return dict of metadata
   ```

3. **Implement semantic chunking**
   - Split by headers (`##`, `###`)
   - Respect 800 token maximum
   - Preserve code blocks intact (use regex to detect ```)
   - Add 100 token overlap between chunks

4. **Implement metadata extraction**
   ```python
   def extract_metadata(file_path: Path, frontmatter: dict) -> ChunkMetadata:
       # Extract module from path: module-1-ros2
       # Extract chapter from filename: 01-introduction
       # Build URL: https://site.com/module/chapter
       # Extract keywords from frontmatter
   ```

5. **Implement token counting**
   - Use tiktoken library for accurate counting
   - Model: text-embedding-ada-002 encoding

6. **Create Chunk dataclass**
   ```python
   @dataclass
   class Chunk:
       text: str
       metadata: ChunkMetadata
       token_count: int
       chunk_id: str  # MD5 hash of text + metadata
   ```

#### Acceptance Criteria
- [ ] Chunks average 400-800 tokens (not exceeding 800)
- [ ] Code blocks never split mid-block
- [ ] Headers preserved in chunk text
- [ ] Metadata includes: module, chapter, title, URL, keywords
- [ ] Deterministic chunk IDs for re-ingestion
- [ ] 100-token overlap implemented correctly

#### Test Cases
```python
# tests/test_document_processor.py
def test_chunking_preserves_code_blocks():
    content = """
## Section
Text before code.

```python
def test():
    return True
```

Text after code.
    """
    processor = DocumentProcessor()
    chunks = processor.chunk_markdown(content, {})
    
    # Code block should be in one chunk
    code_chunks = [c for c in chunks if "```python" in c.text]
    assert len(code_chunks) == 1
    assert "def test():" in code_chunks[0].text
    assert "```" in code_chunks[0].text  # Closing fence

def test_chunk_size_limit():
    # Create content with 1000 tokens
    long_content = " ".join(["word"] * 1000)
    processor = DocumentProcessor()
    chunks = processor.chunk_markdown(long_content, {})
    
    # All chunks should be ≤800 tokens
    assert all(c.token_count <= 800 for c in chunks)

def test_metadata_extraction():
    file_path = Path("textbook/docs/module-1-ros2/01-introduction.md")
    frontmatter = {
        "title": "Introduction to ROS 2",
        "keywords": ["ros2", "introduction"]
    }
    
    processor = DocumentProcessor()
    metadata = processor.extract_metadata(file_path, frontmatter)
    
    assert metadata.module == "module-1-ros2"
    assert metadata.chapter == "01-introduction"
    assert metadata.title == "Introduction to ROS 2"
    assert "ros2" in metadata.keywords

def test_overlap_implementation():
    content = "Section one. " * 100 + "Section two. " * 100
    processor = DocumentProcessor()
    chunks = processor.chunk_markdown(content, {})
    
    if len(chunks) > 1:
        # Check overlap exists between consecutive chunks
        overlap = find_overlap(chunks[0].text, chunks[1].text)
        assert overlap >= 50  # At least 50 tokens overlap
```

#### Files Created
- `backend/app/services/document_processor.py`
- `backend/app/models.py` (Chunk, ChunkMetadata dataclasses)

---

### T004: Implement Embedding Service 🏗️ CORE
**Priority:** P0 (Critical Path)
**Estimated Time:** 2 hours
**Dependencies:** None
**Assigned To:** Backend Dev

#### Description
Create OpenAI embedding service with LRU caching for repeated queries.

#### Implementation Steps

1. **Create `app/services/embeddings.py`**

2. **Initialize OpenAI client**
   ```python
   from openai import OpenAI
   from app.config import settings
   
   client = OpenAI(api_key=settings.OPENAI_API_KEY)
   ```

3. **Implement embedding generation**
   ```python
   def get_embedding(text: str) -> list[float]:
       response = client.embeddings.create(
           model="text-embedding-3-small",
           input=text,
           encoding_format="float"
       )
       return response.data[0].embedding
   ```

4. **Add LRU cache**
   ```python
   from functools import lru_cache
   
   @lru_cache(maxsize=1000)
   def get_embedding_cached(text: str) -> tuple[float]:
       # Returns tuple for hashability
       embedding = get_embedding(text)
       return tuple(embedding)
   ```

5. **Add error handling**
   - Handle API errors gracefully
   - Retry logic for transient failures
   - Log errors with loguru

#### Acceptance Criteria
- [ ] Embeddings return 1536-dimension vectors
- [ ] Cache hit rate >50% for duplicate texts
- [ ] API errors handled gracefully with retries
- [ ] Embedding generation <100ms for cached queries
- [ ] No API calls for identical text

#### Test Cases
```python
# tests/test_embeddings.py
from app.services.embeddings import EmbeddingService

def test_embedding_dimensions():
    service = EmbeddingService()
    embedding = service.get_embedding("test text")
    assert len(embedding) == 1536
    assert all(isinstance(x, float) for x in embedding)

def test_embedding_cache():
    service = EmbeddingService()
    text = "What is ROS 2?"
    
    # First call - cache miss
    emb1 = service.get_embedding(text)
    
    # Second call - cache hit
    emb2 = service.get_embedding(text)
    
    # Should be identical (same object from cache)
    assert emb1 == emb2

def test_embedding_different_texts():
    service = EmbeddingService()
    emb1 = service.get_embedding("ROS 2 is a framework")
    emb2 = service.get_embedding("Unity is a game engine")
    
    # Different texts should have different embeddings
    assert emb1 != emb2

@pytest.mark.integration
def test_openai_api_error_handling():
    # Test with invalid API key
    with pytest.raises(OpenAIError):
        service = EmbeddingService(api_key="invalid")
        service.get_embedding("test")
```

#### Files Created
- `backend/app/services/embeddings.py`

---

## Phase 2: RAG Pipeline (Day 2)

### T005: Implement LLM Service 🏗️ CORE
**Priority:** P0
**Estimated Time:** 3 hours
**Dependencies:** None
**Assigned To:** Backend Dev

#### Description
Create OpenAI chat completion service with prompt engineering to constrain answers to textbook content.

#### Implementation Steps

1. **Create `app/services/llm_service.py`**

2. **Design system prompt**
   ```python
   SYSTEM_PROMPT = """You are a helpful teaching assistant for the Physical AI & Humanoid Robotics textbook.

Rules:
1. ONLY answer questions using the provided textbook context
2. If the context doesn't contain the answer, say "This topic is not covered in the textbook chapters I have access to."
3. Cite specific modules/chapters when answering
4. Keep answers concise (2-3 paragraphs max)
5. Use technical terminology from the textbook
6. If code is mentioned, preserve exact syntax

Context from textbook:
{context}"""
   ```

3. **Implement generate() method**
   ```python
   async def generate(self, question: str, context: str) -> tuple[str, int]:
       response = await self.client.chat.completions.create(
           model="gpt-4o-mini",
           messages=[
               {"role": "system", "content": SYSTEM_PROMPT.format(context=context)},
               {"role": "user", "content": question}
           ],
           temperature=0.1,
           max_tokens=500
       )
       
       answer = response.choices[0].message.content
       tokens = response.usage.total_tokens
       return answer, tokens
   ```

4. **Add async support**
   - Use async OpenAI client
   - Ensure proper await handling

5. **Add logging**
   - Log question, answer length, tokens used
   - Monitor for off-topic responses

#### Acceptance Criteria
- [ ] Generates relevant answers from context
- [ ] Refuses off-topic questions gracefully
- [ ] Returns token count for cost tracking
- [ ] Response time <2s for typical queries
- [ ] Answers cite module/chapter names

#### Test Cases
```python
# tests/test_llm_service.py
@pytest.mark.asyncio
async def test_llm_generates_answer():
    service = LLMService()
    context = "ROS 2 is an open-source framework for robot development..."
    question = "What is ROS 2?"
    
    answer, tokens = await service.generate(question, context)
    
    assert len(answer) > 0
    assert "ROS 2" in answer
    assert "robot" in answer.lower()
    assert tokens > 0

@pytest.mark.asyncio
async def test_llm_refuses_offtopic():
    service = LLMService()
    context = "ROS 2 is an open-source framework..."
    question = "What is the capital of France?"
    
    answer, tokens = await service.generate(question, context)
    
    assert "not covered" in answer.lower() or "textbook" in answer.lower()

@pytest.mark.asyncio
async def test_llm_response_length():
    service = LLMService()
    context = "ROS 2..." * 100  # Long context
    question = "Explain ROS 2 in detail"
    
    answer, tokens = await service.generate(question, context)
    
    # Should be concise despite long context
    assert len(answer.split()) < 500  # ~2-3 paragraphs
```

#### Files Created
- `backend/app/services/llm_service.py`

---

### T006: Implement RAG Pipeline 🏗️ CORE
**Priority:** P0 (Critical Path)
**Estimated Time:** 4 hours
**Dependencies:** T003, T004, T005
**Assigned To:** Backend Dev

#### Description
Orchestrate the full RAG query flow: embed question → search vectors → build context → generate answer.

#### Implementation Steps

1. **Create `app/services/rag_pipeline.py`**

2. **Initialize services**
   ```python
   class RAGPipeline:
       def __init__(self):
           self.embeddings = EmbeddingService()
           self.vector_store = VectorStore()
           self.llm = LLMService()
   ```

3. **Implement query() method**
   ```python
   async def query(self, question: str, max_results: int = 5) -> QueryResponse:
       start_time = time.time()
       
       # Step 1: Embed question
       query_embedding = self.embeddings.get_embedding(question)
       
       # Step 2: Vector search
       results = self.vector_store.search(query_embedding, limit=max_results)
       
       # Step 3: Build context
       if not results:
           return self._no_results_response(start_time)
       
       context = self._build_context(results)
       
       # Step 4: Generate answer
       answer, tokens = await self.llm.generate(question, context)
       
       # Step 5: Format response
       return QueryResponse(
           answer=answer,
           sources=self._format_sources(results),
           response_time_ms=int((time.time() - start_time) * 1000),
           tokens_used=tokens
       )
   ```

4. **Implement _build_context()**
   ```python
   def _build_context(self, results: list) -> str:
       context_parts = []
       for i, result in enumerate(results, 1):
           meta = result["metadata"]
           context_parts.append(
               f"--- Source {i}: {meta['module']} - {meta['title']} ---\n"
               f"{result['text']}\n"
           )
       return "\n".join(context_parts)
   ```

5. **Implement _format_sources()**
   - Convert search results to Source objects
   - Include module, chapter, title, excerpt, score, URL

6. **Add response time tracking**
   - Log slow queries (>3s)
   - Monitor performance metrics

#### Acceptance Criteria
- [ ] Full pipeline completes in <3s (p95)
- [ ] Returns answer + sources for valid questions
- [ ] Empty results handled gracefully
- [ ] Response time tracked and logged
- [ ] Sources include clickable URLs

#### Test Cases
```python
# tests/test_rag_pipeline.py
@pytest.mark.asyncio
async def test_rag_pipeline_with_results(mock_vector_store, mock_llm):
    pipeline = RAGPipeline(
        embeddings=EmbeddingService(),
        vector_store=mock_vector_store,
        llm=mock_llm
    )
    
    result = await pipeline.query("What is ROS 2?")
    
    assert "answer" in result
    assert len(result["sources"]) > 0
    assert result["response_time_ms"] < 3000
    assert result["tokens_used"] > 0

@pytest.mark.asyncio
async def test_rag_pipeline_no_results():
    pipeline = RAGPipeline()
    
    # Mock vector store to return empty results
    pipeline.vector_store.search = lambda *args: []
    
    result = await pipeline.query("completely unrelated question")
    
    assert "couldn't find relevant information" in result["answer"].lower()
    assert len(result["sources"]) == 0

@pytest.mark.asyncio
async def test_rag_pipeline_performance():
    pipeline = RAGPipeline()
    
    start = time.time()
    result = await pipeline.query("What is ROS 2?")
    elapsed = (time.time() - start) * 1000
    
    assert elapsed < 5000  # Allow 5s for test environment
    assert result["response_time_ms"] < 5000
```

#### Files Created
- `backend/app/services/rag_pipeline.py`

---

(Continuing in next message due to length...)

### T007: Create API Endpoints 🔌 INTEGRATION
**Priority:** P0 (Critical Path)
**Estimated Time:** 3 hours
**Dependencies:** T006
**Assigned To:** Backend Dev

#### Description
Implement FastAPI routes for query, health check, and admin ingestion endpoints with rate limiting.

#### Implementation Steps

1. **Create `app/routers/query.py`**
   - POST /api/v1/query endpoint
   - Request validation (5-500 chars)
   - Rate limiting (20/min per IP)
   - Response formatting

2. **Create `app/routers/health.py`**
   - GET /api/v1/health endpoint
   - Check Qdrant connection
   - Check OpenAI availability
   - Return status + version

3. **Create `app/routers/ingest.py`**
   - POST /api/v1/ingest endpoint (admin only)
   - Bearer token authentication
   - Trigger document re-ingestion

4. **Add rate limiting**
   ```python
   from slowapi import Limiter
   
   @router.post("/query")
   @limiter.limit("20/minute")
   async def query_rag(request: Request, body: QueryRequest):
       ...
   ```

5. **Update `app/main.py`**
   - Include all routers
   - Configure rate limiter

#### Acceptance Criteria
- [ ] POST /api/v1/query returns 200 with valid question
- [ ] Returns 400 for invalid input (too short/long)
- [ ] Returns 429 after 20 requests/min from same IP
- [ ] GET /api/v1/health returns status 200
- [ ] Admin endpoint requires Authorization header

#### Test Cases
```python
# tests/test_api_endpoints.py
def test_query_endpoint_success():
    response = client.post("/api/v1/query", json={
        "question": "What is ROS 2?"
    })
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert "response_time_ms" in data

def test_query_endpoint_validation():
    # Too short
    response = client.post("/api/v1/query", json={"question": "Hi"})
    assert response.status_code == 400
    
    # Too long
    response = client.post("/api/v1/query", json={"question": "x" * 501})
    assert response.status_code == 400

def test_query_endpoint_rate_limit():
    for i in range(21):
        response = client.post("/api/v1/query", json={"question": f"test {i}"})
    
    assert response.status_code == 429
    assert "rate limit" in response.json()["detail"].lower()

def test_health_endpoint():
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "qdrant_connected" in data
```

#### Files Created
- `backend/app/routers/query.py`
- `backend/app/routers/health.py`
- `backend/app/routers/ingest.py`

---

## Phase 3: Document Ingestion (Day 2-3)

### T008: Create Ingestion Script 📊 DATA
**Priority:** P0
**Estimated Time:** 3 hours
**Dependencies:** T003, T004, T002
**Assigned To:** Backend Dev

#### Description
Build script to ingest all textbook markdown files into Qdrant vector database.

#### Implementation Steps

1. **Create `backend/scripts/ingest_docs.py`**

2. **Implement file discovery**
   ```python
   def find_markdown_files(docs_path: Path) -> list[Path]:
       files = []
       for md_file in docs_path.rglob("*.md"):
           if "tutorial" not in str(md_file):
               files.append(md_file)
       return files
   ```

3. **Implement batch processing**
   ```python
   async def ingest_documents(docs_path: str, batch_size: int = 50):
       processor = DocumentProcessor()
       embeddings = EmbeddingService()
       vector_store = VectorStore()
       
       files = find_markdown_files(Path(docs_path))
       all_chunks = []
       
       for md_file in files:
           content = md_file.read_text()
           metadata = processor.extract_metadata(md_file, content)
           chunks = processor.chunk_markdown(content, metadata)
           all_chunks.extend(chunks)
       
       # Process in batches
       for i in range(0, len(all_chunks), batch_size):
           batch = all_chunks[i:i+batch_size]
           batch_embeddings = [embeddings.get_embedding(c.text) for c in batch]
           vector_store.upsert_chunks(batch, batch_embeddings)
   ```

4. **Add CLI arguments**
   - `--docs-path`: Path to textbook/docs
   - `--batch-size`: Batch size (default 50)
   - `--verbose`: Enable debug logging

5. **Add progress logging**
   - Progress bar with tqdm
   - Log files processed
   - Log chunks created
   - Log total time

#### Acceptance Criteria
- [ ] Processes all 30+ markdown files
- [ ] Creates ~2000 chunks total
- [ ] Uploads to Qdrant successfully
- [ ] Completes in <5 minutes
- [ ] Logs progress clearly
- [ ] Skips tutorial-* files

#### Test Cases
```python
# tests/test_ingestion.py
def test_find_markdown_files(tmp_path):
    # Create test structure
    (tmp_path / "module-1").mkdir()
    (tmp_path / "module-1/01-intro.md").write_text("# Test")
    (tmp_path / "tutorial-skip.md").write_text("# Skip")
    
    files = find_markdown_files(tmp_path)
    
    assert len(files) == 1
    assert "01-intro.md" in str(files[0])
    assert "tutorial" not in str(files[0])

@pytest.mark.integration
async def test_ingestion_end_to_end(tmp_path):
    # Create minimal test docs
    (tmp_path / "test.md").write_text("""---
title: Test
---
# Test Content
This is test content for ingestion.
""")
    
    await ingest_documents(str(tmp_path))
    
    # Verify chunks in Qdrant
    store = VectorStore()
    results = store.client.scroll("textbook_chunks", limit=10)
    assert len(results[0]) > 0
```

#### Files Created
- `backend/scripts/ingest_docs.py`

---

### T009: Run Initial Ingestion 🚀 DEPLOYMENT
**Priority:** P0
**Estimated Time:** 1 hour
**Dependencies:** T008
**Assigned To:** Backend Dev

#### Description
Execute ingestion script on actual textbook content and verify results.

#### Implementation Steps

1. **Activate virtual environment**
   ```bash
   cd backend
   source venv/bin/activate
   ```

2. **Run ingestion script**
   ```bash
   python scripts/ingest_docs.py \
       --docs-path ../textbook/docs \
       --batch-size 50 \
       --verbose
   ```

3. **Monitor progress**
   - Watch console output
   - Check for errors
   - Note statistics (files, chunks, time)

4. **Verify in Qdrant dashboard**
   - Log into Qdrant Cloud
   - Check collection "textbook_chunks"
   - Verify ~2000 vectors present
   - Check storage usage (<50MB)

5. **Test vector search manually**
   ```python
   from app.services.vector_store import VectorStore
   from app.services.embeddings import EmbeddingService
   
   store = VectorStore()
   embeddings = EmbeddingService()
   
   query = "What is ROS 2?"
   emb = embeddings.get_embedding(query)
   results = store.search(emb, limit=5)
   
   for r in results:
       print(f"Score: {r['score']:.2f} | {r['metadata']['title']}")
   ```

6. **Document results**
   - Create ingestion report
   - Files processed: X
   - Chunks created: Y
   - Time taken: Z minutes
   - Storage used: M MB

#### Acceptance Criteria
- [ ] Qdrant contains vectors from all modules
- [ ] Metadata includes module/chapter/URL for each chunk
- [ ] Vector search returns relevant results
- [ ] Total storage <50MB (well under 1GB limit)
- [ ] No errors during ingestion

#### Manual Verification Checklist
- [ ] Test query: "What is ROS 2?" returns Module 1 results
- [ ] Test query: "Gazebo simulation" returns Module 2 results
- [ ] Test query: "NVIDIA Isaac" returns Module 3 results
- [ ] Test query: "VLA model" returns Module 4 results
- [ ] URLs in metadata are correct and accessible

---

## Phase 4: Frontend Integration (Day 3)

### T010: Create ChatbotWidget Component 🎨 FRONTEND
**Priority:** P0 (Critical Path)
**Estimated Time:** 4 hours
**Dependencies:** None (parallel with backend)
**Assigned To:** Frontend Dev

#### Description
Build React TypeScript chatbot widget with floating button, chat window, and API integration.

#### Implementation Steps

1. **Create component directory**
   ```bash
   mkdir -p textbook/src/components/ChatbotWidget
   ```

2. **Create `index.tsx`**
   - Floating button component (56px circle)
   - Chat window component (400px × 600px)
   - Message list with auto-scroll
   - Input form with submit
   - Loading indicator
   - Error handling

3. **Define TypeScript interfaces**
   ```typescript
   interface Message {
     role: 'user' | 'assistant';
     content: string;
     sources?: Source[];
     timestamp: Date;
   }
   
   interface Source {
     module: string;
     chapter: string;
     title: string;
     excerpt: string;
     score: number;
     url: string;
   }
   ```

4. **Implement state management**
   ```typescript
   const [isOpen, setIsOpen] = useState(false);
   const [messages, setMessages] = useState<Message[]>([welcomeMessage]);
   const [input, setInput] = useState('');
   const [isLoading, setIsLoading] = useState(false);
   ```

5. **Implement API call**
   ```typescript
   const handleSubmit = async (e: React.FormEvent) => {
     e.preventDefault();
     if (!input.trim() || isLoading) return;
     
     const userMessage: Message = {
       role: 'user',
       content: input,
       timestamp: new Date()
     };
     setMessages(prev => [...prev, userMessage]);
     setInput('');
     setIsLoading(true);
     
     try {
       const response = await fetch(`${API_URL}/api/v1/query`, {
         method: 'POST',
         headers: { 'Content-Type': 'application/json' },
         body: JSON.stringify({ question: input })
       });
       
       const data = await response.json();
       
       const assistantMessage: Message = {
         role: 'assistant',
         content: data.answer,
         sources: data.sources,
         timestamp: new Date()
       };
       setMessages(prev => [...prev, assistantMessage]);
     } catch (error) {
       // Error handling
     } finally {
       setIsLoading(false);
     }
   };
   ```

6. **Implement auto-scroll**
   ```typescript
   const messagesEndRef = useRef<HTMLDivElement>(null);
   
   useEffect(() => {
     messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
   }, [messages]);
   ```

#### Acceptance Criteria
- [ ] Widget opens/closes smoothly
- [ ] Messages display correctly (user right, assistant left)
- [ ] Sources shown as clickable links
- [ ] Loading indicator shows during API call
- [ ] Error messages displayed for failed requests
- [ ] Input clears after submission
- [ ] Auto-scrolls to latest message

#### Files Created
- `textbook/src/components/ChatbotWidget/index.tsx`
- `textbook/src/components/ChatbotWidget/types.ts`

---

### T011: Style ChatbotWidget (Dark Mode) 🎨 FRONTEND
**Priority:** P1
**Estimated Time:** 3 hours
**Dependencies:** T010
**Assigned To:** Frontend Dev

#### Description
Create CSS modules with dark mode support and responsive design.

#### Implementation Steps

1. **Create `styles.module.css`**

2. **Style floating button**
   ```css
   .floatingButton {
     position: fixed;
     bottom: 24px;
     right: 24px;
     width: 56px;
     height: 56px;
     border-radius: 50%;
     background: var(--ifm-color-primary);
     border: none;
     box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
     cursor: pointer;
     z-index: 1000;
   }
   ```

3. **Style chat window**
   ```css
   .chatWindow {
     position: fixed;
     bottom: 24px;
     right: 24px;
     width: 400px;
     height: 600px;
     background: var(--ifm-background-color);
     border: 1px solid var(--ifm-color-emphasis-300);
     border-radius: 12px;
     box-shadow: 0 8px 24px rgba(0, 0, 0, 0.15);
     display: flex;
     flex-direction: column;
   }
   ```

4. **Add dark mode support**
   ```css
   [data-theme='dark'] .chatWindow {
     border-color: var(--ifm-color-emphasis-200);
     box-shadow: 0 8px 24px rgba(0, 0, 0, 0.4);
   }
   
   [data-theme='dark'] .message-assistant {
     background: var(--ifm-color-emphasis-200);
   }
   ```

5. **Add responsive breakpoints**
   ```css
   @media (max-width: 768px) {
     .chatWindow {
       width: calc(100vw - 32px);
       height: calc(100vh - 32px);
       bottom: 16px;
       right: 16px;
     }
   }
   
   @media (max-width: 320px) {
     .floatingButton {
       bottom: 16px;
       right: 16px;
       width: 48px;
       height: 48px;
     }
   }
   ```

6. **Ensure accessibility**
   - Color contrast ≥4.5:1
   - Focus indicators visible
   - Touch targets ≥44px

#### Acceptance Criteria
- [ ] Light mode styled correctly
- [ ] Dark mode uses proper theme colors
- [ ] Mobile responsive (tested 320px, 768px)
- [ ] Color contrast ≥4.5:1 (verified with WebAIM)
- [ ] Matches existing Docusaurus design
- [ ] Focus indicators visible on all interactive elements

#### Manual Verification
- [ ] Toggle dark mode - widget adapts correctly
- [ ] Resize browser to 320px - layout doesn't break
- [ ] Check contrast with Lighthouse
- [ ] Tab through widget - focus visible

#### Files Created
- `textbook/src/components/ChatbotWidget/styles.module.css`

---

### T012: Integrate Widget via Root.tsx 🔌 INTEGRATION
**Priority:** P0 (Critical Path)
**Estimated Time:** 1 hour
**Dependencies:** T010
**Assigned To:** Frontend Dev

#### Description
Add ChatbotWidget to all pages using Root wrapper component.

#### Implementation Steps

1. **Create `textbook/src/theme/Root.tsx`**
   ```typescript
   import React from 'react';
   import ChatbotWidget from '../components/ChatbotWidget';
   
   export default function Root({ children }) {
     return (
       <>
         {children}
         <ChatbotWidget />
       </>
     );
   }
   ```

2. **Test on multiple pages**
   - Homepage (/)
   - Module 1 chapter (/module-1-ros2/01-introduction)
   - Module 2 chapter
   - Search page

3. **Verify no layout conflicts**
   - Check z-index stacking
   - Verify no overflow issues
   - Check mobile menu doesn't overlap

4. **Test state persistence**
   - Open widget, navigate to new page
   - Verify widget state preserved
   - Verify messages remain (within session)

#### Acceptance Criteria
- [ ] Widget appears on all pages
- [ ] No layout shift or visual conflicts
- [ ] Widget state persists during navigation
- [ ] Performance impact <100ms (measured with Lighthouse)
- [ ] z-index doesn't conflict with nav/modals

#### Test Cases (Manual)
- [ ] Open widget on homepage - works
- [ ] Navigate to module page - widget still visible
- [ ] Ask question, navigate - conversation persists
- [ ] Open mobile menu - doesn't overlap widget
- [ ] Refresh page - widget resets (expected)

#### Files Created
- `textbook/src/theme/Root.tsx`

---

### T013: Configure Environment Variables 🔧 CONFIG
**Priority:** P0
**Estimated Time:** 1 hour
**Dependencies:** None
**Assigned To:** Frontend Dev

#### Description
Set up frontend environment variables for API URL configuration.

#### Implementation Steps

1. **Create `.env.local.example`**
   ```bash
   # API Configuration
   REACT_APP_API_URL=http://localhost:8000
   
   # For production (set in Vercel):
   # REACT_APP_API_URL=https://your-backend.railway.app
   ```

2. **Create `.env.local` (gitignored)**
   ```bash
   REACT_APP_API_URL=http://localhost:8000
   ```

3. **Update `.gitignore`**
   ```
   .env.local
   .env.*.local
   ```

4. **Update ChatbotWidget to use env var**
   ```typescript
   const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
   ```

5. **Update README**
   - Document required env vars
   - Provide setup instructions
   - Explain dev vs prod configuration

#### Acceptance Criteria
- [ ] Local dev uses http://localhost:8000
- [ ] Production will use Railway URL (configured in Vercel)
- [ ] .env.local not committed to git
- [ ] README documents env var setup
- [ ] No hardcoded URLs in code

#### Files Created
- `textbook/.env.local.example`
- `textbook/.env.local` (gitignored)
- Updated `textbook/.gitignore`
- Updated `textbook/README.md`

---

## Phase 5: Deployment & Testing (Day 4)

(Remaining tasks T014-T019 follow similar detailed format...)

**Summary:**
- T014: Deploy Backend to Railway
- T015: Update Frontend with Production API
- T016: E2E Testing (Playwright)
- T017: Performance Testing (Benchmark script)
- T018: Accessibility Audit (Lighthouse)
- T019: Create Documentation (README, architecture diagram)

---

## Task Summary

| Phase | Tasks | Est. Hours | Status |
|-------|-------|------------|--------|
| 1: Backend Foundation | T001-T004 | 9h | Pending |
| 2: RAG Pipeline | T005-T007 | 10h | Pending |
| 3: Ingestion | T008-T009 | 4h | Pending |
| 4: Frontend | T010-T013 | 9h | Pending |
| 5: Deployment | T014-T019 | 11.5h | Pending |
| **Total** | **19 tasks** | **43.5h** | **0% complete** |

## Critical Path

```
T001 (FastAPI) → 
T002 (Qdrant) → 
T003 (Doc Processor) → 
T004 (Embeddings) → 
T006 (RAG Pipeline) → 
T007 (API) → 
T008 (Ingestion Script) → 
T009 (Run Ingestion) → 
T010 (ChatWidget) → 
T012 (Integration) → 
T014 (Railway Deploy) → 
T015 (Prod Config)
```

## Parallel Work Opportunities

- **Day 1:** T001-T004 (Backend) || T010-T011 (Frontend) in parallel
- **Day 2:** T008 (Ingestion script) || T010-T011 (ChatWidget) in parallel

---

**Document Status:** ✅ Ready for Implementation
**Version:** 1.0.0
**Last Updated:** 2025-12-26
**Next Step:** Begin T001 (Initialize FastAPI Backend)

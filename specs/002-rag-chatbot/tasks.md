# Tasks: RAG Chatbot for Docusaurus Textbook

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md ✅, spec.md ✅
**Feature Branch**: `002-rag-chatbot`
**Created**: 2025-12-16

**Tests**: Unit and integration tests included for all components

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Backend**: `rag-backend/app/`
- **Frontend**: `textbook/src/components/ChatWidget/`
- **Tests**: `rag-backend/tests/`, `textbook/tests/e2e/`
- **Scripts**: `rag-backend/scripts/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize FastAPI backend and prepare Docusaurus for ChatWidget integration

- [ ] T001 Create rag-backend directory structure: `mkdir -p rag-backend/{app/{api/routes,agents,rag,vector_store,models,utils},scripts,tests}`
- [ ] T002 Initialize Python project: Create `rag-backend/requirements.txt` with dependencies
- [ ] T003 [P] Create `rag-backend/.env.example` with environment variable template
- [ ] T004 [P] Create `rag-backend/Dockerfile` for containerization
- [ ] T005 [P] Create `rag-backend/docker-compose.yml` for local development
- [ ] T006 [P] Create `rag-backend/README.md` with setup instructions
- [ ] T007 Install Python dependencies: `cd rag-backend && pip install -r requirements.txt`
- [ ] T008 Create FastAPI app entry point: `rag-backend/app/main.py` (basic "Hello World")
- [ ] T009 [P] Create configuration module: `rag-backend/app/config.py` (load env vars)
- [ ] T010 Test FastAPI runs locally: `uvicorn app.main:app --reload`

**Checkpoint**: ✅ Backend skeleton runs, environment configured, dependencies installed

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Vector Store Foundation

- [ ] T011 Create Qdrant schema definition: `rag-backend/app/vector_store/schema.py`
- [ ] T012 Implement Qdrant client wrapper: `rag-backend/app/vector_store/client.py`
- [ ] T013 Implement vector store operations: `rag-backend/app/vector_store/operations.py` (upsert, search, delete)
- [ ] T014 Create collection initialization script: Add `create_collection()` function in `client.py`
- [ ] T015 Test Qdrant connection: Connect to Qdrant Cloud, create test collection

### Agent System Foundation

- [ ] T016 Create BaseAgent interface: `rag-backend/app/agents/base.py` (abstract class)
- [ ] T017 Create AgentContext model: `rag-backend/app/models/agents.py` (input to agents)
- [ ] T018 Create AgentResponse model: `rag-backend/app/models/agents.py` (output from agents)
- [ ] T019 Create AgentOrchestrator: `rag-backend/app/agents/orchestrator.py` (routing logic)

### RAG Pipeline Foundation

- [ ] T020 Create embeddings wrapper: `rag-backend/app/rag/embeddings.py` (OpenAI API client)
- [ ] T021 Create text chunking module: `rag-backend/app/rag/chunking.py` (RecursiveCharacterTextSplitter)
- [ ] T022 Create markdown parser: `rag-backend/app/utils/markdown_parser.py` (parse frontmatter + text)
- [ ] T023 Test chunking with sample markdown file

### API Models Foundation

- [ ] T024 Create chat models: `rag-backend/app/models/chat.py` (ChatQuery, ChatResponse, Citation)
- [ ] T025 [P] Create document models: `rag-backend/app/models/documents.py` (DocumentChunk, IngestionRequest)
- [ ] T026 [P] Create logging config: `rag-backend/app/utils/logging_config.py` (structured logging)

### API Infrastructure

- [ ] T027 Create health check route: `rag-backend/app/api/routes/health.py` (GET /health, /collections/stats)
- [ ] T028 Configure CORS middleware in `main.py` (allow Docusaurus domain)
- [ ] T029 Configure error handling middleware in `main.py`
- [ ] T030 Test health endpoint: `curl http://localhost:8000/api/v1/health`

**Checkpoint**: ✅ Foundation ready - Qdrant connected, agent system structure in place, API can receive requests

---

## Phase 3: User Story 4 - Admin Ingests Textbook Content (Priority: P1) 🎯

**Note**: Implementing US4 before US1 because we need documents in Qdrant before we can query

**Goal**: Admin can ingest all textbook markdown files into Qdrant vector database

**Independent Test**: Run ingestion script, verify Qdrant contains vectors, query chatbot to confirm retrieval works

### Implementation for User Story 4

- [ ] T031 Implement ingestion pipeline: `rag-backend/app/rag/ingestion.py`
  - Load markdown files from directory
  - Parse frontmatter and extract metadata
  - Chunk text (max 500 tokens)
  - Generate embeddings for each chunk
  - Upsert to Qdrant with payloads
- [ ] T032 Create ingestion script: `rag-backend/scripts/ingest_docs.py` (CLI interface)
- [ ] T033 Add progress logging to ingestion (show files processed, chunks created)
- [ ] T034 Implement error handling for ingestion (skip malformed files, log errors)
- [ ] T035 Create ingestion report generator (summary: files, chunks, embeddings, errors)
- [ ] T036 Test ingestion with 1-2 sample markdown files
- [ ] T037 Run full ingestion on textbook/docs/ directory (all 30 files)
- [ ] T038 Verify Qdrant collection stats: Check vector count, payload integrity
- [ ] T039 Create ingest API endpoint: `rag-backend/app/api/routes/ingest.py` (POST /ingest/documents)
- [ ] T040 Test ingest endpoint: `curl -X POST http://localhost:8000/api/v1/ingest/documents`

**Checkpoint**: ✅ User Story 4 complete - Textbook ingested, Qdrant contains ~200-400 vectors

---

## Phase 4: User Story 1 - Ask Questions About Full Book Content (Priority: P1) 🎯 MVP

**Goal**: Learners can ask questions about the textbook and receive answers with citations

**Independent Test**: Open chat widget, ask "What is ROS 2?", verify answer with working citation links

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T041 [P] [US1] Unit test for BookSearchAgent: `rag-backend/tests/test_agents.py`
- [ ] T042 [P] [US1] Integration test for full RAG pipeline: `rag-backend/tests/test_rag_pipeline.py`
- [ ] T043 [P] [US1] API test for chat query endpoint: `rag-backend/tests/test_api.py`

### Implementation for User Story 1

#### Backend: BookSearchAgent

- [ ] T044 [US1] Implement BookSearchAgent: `rag-backend/app/agents/book_search.py`
  - Inherit from BaseAgent
  - Implement `execute()` method
  - Embed query → search Qdrant → build context → call OpenAI → return answer
- [ ] T045 [US1] Implement retrieval pipeline: `rag-backend/app/rag/retrieval.py`
  - Query embedding generation
  - Vector search with filters
  - Context building from chunks
  - LLM completion with system prompt
- [ ] T046 [US1] Add BookSearchAgent to AgentOrchestrator routing logic
- [ ] T047 [US1] Test BookSearchAgent with sample query: "What is ROS 2?"

#### Backend: Citation Agent

- [ ] T048 [P] [US1] Implement CitationAgent: `rag-backend/app/agents/citation.py`
  - Format citations from retrieved chunks
  - Generate inline citations [1], [2]
  - Generate footer citations with URLs
- [ ] T049 [US1] Integrate CitationAgent with BookSearchAgent output

#### Backend: Chat API

- [ ] T050 [US1] Implement chat query route: `rag-backend/app/api/routes/chat.py`
  - POST /api/v1/chat/query endpoint
  - Validate request with Pydantic model
  - Call AgentOrchestrator
  - Return formatted response
- [ ] T051 [US1] Add request/response logging
- [ ] T052 [US1] Add error handling (Qdrant down, OpenAI timeout)
- [ ] T053 [US1] Test chat endpoint with curl: Query → Answer + Citations
- [ ] T054 [US1] Verify citation URLs are correct (match Docusaurus structure)

#### Frontend: ChatWidget Component

- [ ] T055 [US1] Create ChatWidget directory: `textbook/src/components/ChatWidget/`
- [ ] T056 [P] [US1] Create types: `textbook/src/components/ChatWidget/types.ts` (TypeScript interfaces)
- [ ] T057 [P] [US1] Create API client: `textbook/src/components/ChatWidget/api.ts` (axios wrapper)
- [ ] T058 [US1] Create ChatWidget main component: `textbook/src/components/ChatWidget/index.tsx`
- [ ] T059 [P] [US1] Create ChatInterface component: `textbook/src/components/ChatWidget/ChatInterface.tsx`
- [ ] T060 [P] [US1] Create ChatMessage component: `textbook/src/components/ChatWidget/ChatMessage.tsx`
- [ ] T061 [P] [US1] Create CitationList component: `textbook/src/components/ChatWidget/CitationList.tsx`
- [ ] T062 [P] [US1] Create LoadingIndicator component: `textbook/src/components/ChatWidget/LoadingIndicator.tsx`
- [ ] T063 [P] [US1] Create styles: `textbook/src/components/ChatWidget/styles.module.css`

#### Frontend: Integration

- [ ] T064 [US1] Import ChatWidget in Docusaurus layout or custom wrapper
- [ ] T065 [US1] Add floating chat button (bottom-right corner)
- [ ] T066 [US1] Implement expand/collapse chat widget
- [ ] T067 [US1] Test chat widget renders correctly
- [ ] T068 [US1] Test API call from frontend to backend (CORS configured)
- [ ] T069 [US1] Test citation links navigate to correct textbook sections
- [ ] T070 [US1] Test error handling (backend down, network error)

#### End-to-End Testing

- [ ] T071 [US1] Ask 10 test questions covering all 4 modules
- [ ] T072 [US1] Verify all citations link to correct sections
- [ ] T073 [US1] Verify response time <3 seconds for 90% of queries
- [ ] T074 [US1] Test on mobile device (responsive layout)

**Checkpoint**: ✅ User Story 1 complete - Full-book Q&A works with citations, frontend integrated

---

## Phase 5: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Learners can highlight text and ask context-restricted questions

**Independent Test**: Select paragraph, click "Ask about this", verify answer uses only selected text

### Tests for User Story 2

- [ ] T075 [P] [US2] Unit test for SelectedTextAgent: `rag-backend/tests/test_agents.py`
- [ ] T076 [P] [US2] Integration test for selected text mode: `rag-backend/tests/test_rag_pipeline.py`

### Implementation for User Story 2

#### Backend: SelectedTextAgent

- [ ] T077 [US2] Implement SelectedTextAgent: `rag-backend/app/agents/selected_text.py`
  - Accept selected_text in context
  - Embed query + selected_text together
  - Search with chapter/module filters (restrict scope)
  - Build constrained context
  - Call OpenAI with "answer only using this text" system prompt
- [ ] T078 [US2] Add SelectedTextAgent to AgentOrchestrator routing
- [ ] T079 [US2] Update chat API to handle selected_text parameter
- [ ] T080 [US2] Test selected text mode: Submit query with selected_text, verify constrained answer

#### Frontend: Text Selection

- [ ] T081 [US2] Create ContextSelector component: `textbook/src/components/ChatWidget/ContextSelector.tsx`
- [ ] T082 [US2] Implement text selection handler (detect mouseup event)
- [ ] T083 [US2] Show "Ask ChatBot about this" button when text is selected
- [ ] T084 [US2] Pre-populate chat input with selected text context
- [ ] T085 [US2] Add mode toggle: "Full Book" vs "Selected Text"
- [ ] T086 [US2] Test text selection on desktop
- [ ] T087 [US2] Test text selection on mobile (alternative: manual paste)
- [ ] T088 [US2] Verify selected text mode restricts answers correctly

**Checkpoint**: ✅ User Story 2 complete - Context-restricted Q&A works

---

## Phase 6: User Story 3 - View Source Citations (Priority: P1)

**Note**: This is integrated into US1, so most tasks are already complete

**Goal**: Every answer includes clickable citations with deep links

**Independent Test**: Ask any question, verify citations display, click citations, verify navigation

### Additional Implementation for User Story 3

- [ ] T089 [US3] Enhance citation display with module/chapter/heading hierarchy
- [ ] T090 [US3] Add relevance score display (optional)
- [ ] T091 [US3] Test deep linking to heading anchors (verify URLs match Docusaurus structure)
- [ ] T092 [US3] Add citation hover preview (show snippet of source text)
- [ ] T093 [US3] Test 20 citations for correctness (URLs, text, formatting)

**Checkpoint**: ✅ User Story 3 complete - Citations work perfectly with navigation

---

## Phase 7: User Story 5 - Chatbot Provides Fast Responses (Priority: P3)

**Goal**: Optimize performance to achieve <3s P95 latency

**Independent Test**: Ask 10 questions, measure response times, verify 90% are <3s

### Performance Optimization

- [ ] T094 [US5] Add response time tracking in backend (log latency per request)
- [ ] T095 [US5] Optimize vector search: Tune top_k parameter (start with 5, adjust)
- [ ] T096 [US5] Implement query caching (optional, defer if performance is acceptable)
- [ ] T097 [US5] Test concurrent requests: Use Locust or k6 to simulate 10 concurrent users
- [ ] T098 [US5] Profile slow queries (identify bottlenecks: embedding, search, LLM)
- [ ] T099 [US5] Add loading indicator with "Still thinking..." after 5 seconds
- [ ] T100 [US5] Verify P95 latency <3 seconds

**Checkpoint**: ✅ User Story 5 complete - Performance optimized

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Deployment, testing, documentation

### Deployment

- [ ] T101 [P] Deploy backend to Railway/Render/Fly.io
- [ ] T102 [P] Set up environment variables on cloud platform (OPENAI_API_KEY, QDRANT_URL, etc.)
- [ ] T103 Test deployed backend health endpoint
- [ ] T104 Update frontend API client to use deployed backend URL
- [ ] T105 Deploy frontend (Docusaurus site already on Vercel/GitHub Pages)
- [ ] T106 Test end-to-end on production (frontend → backend → Qdrant)
- [ ] T107 Verify CORS is configured correctly for production domain

### Testing

- [ ] T108 [P] Run all unit tests: `cd rag-backend && pytest tests/`
- [ ] T109 [P] Run integration tests
- [ ] T110 [P] Run E2E tests (if created)
- [ ] T111 Load test with 10 concurrent users (Locust/k6)
- [ ] T112 Security audit: Check for exposed API keys, SQL injection (N/A), CORS misconfig

### Documentation

- [ ] T113 Update `rag-backend/README.md` with setup instructions
- [ ] T114 [P] Document API endpoints (OpenAPI/Swagger auto-generated by FastAPI)
- [ ] T115 [P] Create user guide: "How to use the chatbot" in textbook
- [ ] T116 Update root `README.md` with Task 2 status

### Edge Case Handling

- [ ] T117 Handle empty query (frontend validation)
- [ ] T118 Handle very long selected text (truncate to 4000 tokens)
- [ ] T119 Handle Qdrant downtime (friendly error message)
- [ ] T120 Handle OpenAI rate limit (retry with backoff or queue)
- [ ] T121 Handle non-English queries (return "English only" message)

### Monitoring & Observability

- [ ] T122 Set up logging (structured logs to stdout)
- [ ] T123 Add metrics tracking (query count, latency, error rate)
- [ ] T124 [P] Set up error alerting (optional: Sentry integration)
- [ ] T125 Monitor OpenAI API costs (set budget alerts)

**Checkpoint**: ✅ Production-ready - Backend deployed, frontend integrated, tested, documented

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) ────────────────────────────────────────────►
                 │
Phase 2 (Foundational) ─────────────────────────────────────►
                         │
                         ├── Phase 3 (US4: Ingestion) ──────►
                         │            │
                         │            └─► Phase 4 (US1: Full Book Q&A) ──►
                         │                         │
                         │                         ├─► Phase 5 (US2: Selected Text) ──►
                         │                         │
                         │                         └─► Phase 6 (US3: Citations) ──►
                         │                                      │
                         │                                      └─► Phase 7 (US5: Performance) ──►
                         │                                                   │
                         └───────────────────────────────────────────────────┘
                                                                             │
                                                    Phase 8 (Polish & Deploy) ──►
```

### User Story Dependencies

| Story | Depends On | Can Parallelize With |
|-------|------------|---------------------|
| US4 (Ingestion) | Phase 2 (Foundational) | - |
| US1 (Full Book Q&A) | US4 (data must exist) | - |
| US2 (Selected Text) | US1 (uses same pipeline) | US3 |
| US3 (Citations) | US1 (integrated) | US2 |
| US5 (Performance) | US1, US2, US3 (optimize existing) | - |

### Within Each User Story

1. **Tests written FIRST** (where applicable) - ensure they FAIL
2. Backend implementation (agents, API)
3. Frontend implementation (components, integration)
4. End-to-end testing
5. Verify checkpoint before moving to next story

---

## Parallel Execution Examples

### Phase 2: Foundational Parallel Launch

```bash
# All model creation can run in parallel:
Task T024: "Create chat models in app/models/chat.py"
Task T025: "Create document models in app/models/documents.py"
Task T026: "Create logging config in app/utils/logging_config.py"

# All vector store modules in parallel:
Task T011: "Create schema in app/vector_store/schema.py"
Task T012: "Create client in app/vector_store/client.py"
Task T013: "Create operations in app/vector_store/operations.py"
```

### Phase 4 (US1): Frontend Components Parallel Launch

```bash
# All React components can be created in parallel:
Task T056: "Create types.ts"
Task T057: "Create api.ts"
Task T060: "Create ChatMessage.tsx"
Task T061: "Create CitationList.tsx"
Task T062: "Create LoadingIndicator.tsx"
Task T063: "Create styles.module.css"
```

---

## Implementation Strategy

### MVP First (User Stories 4 + 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T030) - **CRITICAL**
3. Complete Phase 3: User Story 4 - Ingestion (T031-T040)
4. Complete Phase 4: User Story 1 - Full Book Q&A (T041-T074)
5. **STOP and VALIDATE**: Test chatbot with 10 questions, verify citations work
6. Deploy to production (simplified Phase 8)

**MVP delivers**: Functional RAG chatbot with full-book Q&A and citations

### Incremental Delivery

1. **MVP**: Setup + Foundational + US4 + US1 → Browsable chatbot with Q&A
2. **+Selected Text**: US2 → Context-restricted Q&A
3. **+Citations**: US3 (already integrated) → Enhanced citation display
4. **+Performance**: US5 → Optimized for speed
5. **Production**: Phase 8 → Deployed, tested, documented

---

## Task Summary

| Phase | Task Count | Parallel Tasks | Estimated Hours |
|-------|-----------|----------------|-----------------|
| Phase 1: Setup | 10 | 6 | 3 |
| Phase 2: Foundational | 20 | 8 | 8 |
| Phase 3: US4 Ingestion | 10 | 2 | 5 |
| Phase 4: US1 Full Book Q&A | 34 | 16 | 20 |
| Phase 5: US2 Selected Text | 14 | 4 | 6 |
| Phase 6: US3 Citations | 5 | 2 | 2 |
| Phase 7: US5 Performance | 7 | 2 | 3 |
| Phase 8: Polish & Deploy | 25 | 8 | 10 |
| **Total** | **125** | **48** | **~57 hrs** |

---

## Success Criteria Mapping

| Success Criteria | Verified By Task |
|-----------------|------------------|
| SC-001: 80%+ correct answers | T071, T072 (test 10 questions) |
| SC-002: 100% citation accuracy | T072, T093 (test 20 citations) |
| SC-003: Ingestion <5 minutes | T037 (full ingestion) |
| SC-004: P95 latency <3 seconds | T100 (performance testing) |
| SC-005: 200-400 vectors in Qdrant | T038 (verify collection stats) |
| SC-006: Cross-browser support | T074 (mobile test), T106 (production test) |
| SC-007: 10 concurrent users | T097, T111 (load test) |
| SC-008: Selected text mode works | T088 (verify constrained answers) |
| SC-009: 99% uptime | T103, T106 (deployed health check) |
| SC-010: OpenAI costs <$50/month | T125 (cost monitoring) |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Tests written FIRST, ensure they FAIL before implementation
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- US4 (Ingestion) must complete before US1 (data dependency)
- US1 is the critical path for MVP

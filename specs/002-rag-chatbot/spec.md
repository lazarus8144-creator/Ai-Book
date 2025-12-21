# Feature Specification: RAG Chatbot for Docusaurus Textbook

**Feature Branch**: `002-rag-chatbot`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Build and embed a Retrieval-Augmented Generation (RAG) chatbot inside the published Docusaurus book using FastAPI, OpenAI Agents/ChatKit SDKs, and Qdrant Cloud (Free Tier). The chatbot must answer questions about the full book content AND answer questions based ONLY on user-selected text (context-restricted RAG). Backend should support document ingestion (markdown → chunks → embeddings → Qdrant) and query-time retrieval with source citations. Design reusable intelligence similar to Matrix-style Skills with Claude Code Subagents or modular Agent Skills (BookSearchAgent, SelectedTextAgent, CitationAgent)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Full Book Content (Priority: P1)

A learner is reading the robotics textbook and has a question that might be answered in a different chapter or module. They open the chatbot widget, ask "How does ROS 2 integrate with NVIDIA Isaac?" and receive an answer synthesized from multiple chapters with clickable citations linking back to specific sections.

**Why this priority**: This is the core MVP value proposition - intelligent Q&A across the entire textbook. Without this, there is no chatbot functionality.

**Independent Test**: Can be fully tested by deploying the chatbot widget, asking questions that require knowledge from multiple modules (e.g., "What's the difference between Gazebo and Isaac Sim?"), and verifying answers include accurate citations with working links.

**Acceptance Scenarios**:

1. **Given** a learner on any textbook page, **When** they open the chat widget and ask "What is ROS 2?", **Then** they receive an accurate answer with citations linking to Module 1 chapters
2. **Given** a learner asks a multi-hop question like "How do I use ROS 2 with Unity?", **When** the chatbot processes the query, **Then** it retrieves context from both Module 1 (ROS 2) and Module 2 (Unity) and synthesizes a coherent answer
3. **Given** the chatbot returns an answer, **When** the learner clicks on a citation, **Then** they navigate to the exact section in the textbook (deep link with heading anchor)
4. **Given** a learner asks a question not covered in the textbook, **When** the chatbot searches and finds no relevant content, **Then** it responds with "I couldn't find information about that in this textbook" rather than hallucinating

---

### User Story 2 - Ask Questions About Selected Text (Priority: P2)

A learner is reading a complex paragraph about VLA architecture. They highlight the paragraph, right-click or use a context menu to ask "Explain this in simpler terms" or "What does this mean?". The chatbot answers using ONLY the selected text as context, not the entire book.

**Why this priority**: This provides contextual, focused help for difficult passages. It's valuable but the textbook is still useful without it (learners can ask general questions via Story 1).

**Independent Test**: Can be tested by selecting specific text on any page, triggering the "Ask about selection" action, and verifying the chatbot's answer is constrained to only that selected text (not pulling from other chapters).

**Acceptance Scenarios**:

1. **Given** a learner highlights a paragraph, **When** they click "Ask ChatBot about this", **Then** the chat widget opens with the selected text pre-loaded as context
2. **Given** selected text is loaded in the chat, **When** the learner asks "Explain this", **Then** the chatbot answers using ONLY the selected text (not searching the full book)
3. **Given** a learner asks a question about selected text, **When** the answer is insufficient because context is limited, **Then** the chatbot offers to "Search the full book for more details"
4. **Given** selected text is cleared, **When** the learner asks a new question, **Then** the chatbot switches back to full-book search mode

---

### User Story 3 - View Source Citations and Navigate to Content (Priority: P1)

A learner receives an answer from the chatbot and wants to verify the information or read more context. Each answer includes numbered citations (e.g., [1], [2]) with the source module/chapter and a clickable link. Clicking the citation navigates directly to that section with the relevant text highlighted.

**Why this priority**: Citations provide credibility, verifiability, and learning reinforcement. This is critical for an educational tool - users must be able to trace answers back to authoritative sources.

**Independent Test**: Can be tested by asking any question, verifying citations are present in the answer, clicking each citation link, and confirming navigation to the correct textbook section.

**Acceptance Scenarios**:

1. **Given** the chatbot returns an answer, **When** the answer is displayed, **Then** it includes inline citations like "ROS 2 is a communication framework [1]" with a footnote linking to the source
2. **Given** citations are displayed, **When** the learner clicks on "[1]", **Then** they navigate to the exact section (e.g., /module-1-ros2/01-introduction#what-is-ros-2)
3. **Given** an answer cites multiple sources, **When** displayed, **Then** all citations are numbered sequentially and listed at the bottom with full references (Module > Chapter > Heading)
4. **Given** a citation link is clicked, **When** the page loads, **Then** the relevant text is highlighted or scrolled into view (if technically feasible)

---

### User Story 4 - Admin Ingests Textbook Content (Priority: P1)

An administrator or developer needs to update the chatbot's knowledge base when textbook content changes. They run an ingestion script that processes all markdown files in the Docusaurus docs folder, chunks them intelligently, generates embeddings, and uploads to Qdrant. The process completes in under 5 minutes for the full textbook and provides a summary report.

**Why this priority**: Without ingestion, there is no knowledge base. This is a foundational technical requirement, though end-users never interact with it directly.

**Independent Test**: Can be tested by running the ingestion script on the existing textbook, verifying all markdown files are processed, checking Qdrant contains the expected number of vectors, and querying the chatbot to confirm new content is retrievable.

**Acceptance Scenarios**:

1. **Given** the textbook has 30 markdown files, **When** the admin runs `python scripts/ingest_docs.py`, **Then** all files are processed and chunked into ~200-300 chunks (estimated)
2. **Given** ingestion is running, **When** processing completes, **Then** a summary report shows: files processed, chunks created, embeddings generated, Qdrant upsert status
3. **Given** content in the textbook changes, **When** the admin re-runs ingestion, **Then** existing vectors are updated or replaced (idempotent ingestion)
4. **Given** a markdown file has frontmatter metadata (module, chapter), **When** ingested, **Then** the metadata is preserved in Qdrant payloads for filtering

---

### User Story 5 - Chatbot Provides Fast Responses (Priority: P3)

A learner asks a question via the chatbot. The system retrieves relevant chunks from Qdrant, generates an answer using OpenAI, and displays it within 3 seconds. A loading indicator shows progress during retrieval and generation.

**Why this priority**: Speed improves UX but is not blocking for MVP. A 5-10 second response is still acceptable for an educational chatbot.

**Independent Test**: Can be tested by asking 10 different questions and measuring response times with browser DevTools or backend logging.

**Acceptance Scenarios**:

1. **Given** a learner submits a question, **When** the chatbot processes it, **Then** a loading spinner appears within 200ms
2. **Given** the backend receives a query, **When** it performs vector search + LLM generation, **Then** the total response time is under 3 seconds for 90% of queries
3. **Given** the backend is slow, **When** response time exceeds 5 seconds, **Then** the user sees a "Still thinking..." message to prevent perceived hang

---

### Edge Cases

- **What happens when a user asks a question in a language other than English?** (Return a message: "This chatbot currently supports English only. Please ask in English.")
- **How does the system handle very long selected text (e.g., an entire chapter)?** (Truncate to max 4000 tokens and warn user: "Selection is too long. Showing first 4000 tokens.")
- **What if Qdrant Cloud is down or unreachable?** (Return error message: "Chatbot is temporarily unavailable. Please try again later." + log error for monitoring)
- **What if a user submits an empty query?** (Show validation error: "Please enter a question.")
- **What if OpenAI API rate limit is hit?** (Queue the request or show: "High traffic detected. Please wait 30 seconds and try again.")
- **What if a user highlights text and asks a question unrelated to the selection?** (Chatbot still uses selected text as primary context but can clarify: "Based on your selected text about X, here's the answer...")
- **What if the textbook contains images or diagrams (non-text content)?** (Ingestion script skips images; chatbot cannot answer questions about visual content. Future enhancement: image embeddings.)
- **What if a query requires real-time information (e.g., "What's the latest ROS 2 version?")?** (Chatbot responds based on textbook content only and disclaims: "Based on the textbook content as of [date]...")

## Requirements *(mandatory)*

### Functional Requirements

#### Core Chatbot Functionality
- **FR-001**: System MUST provide a chat interface widget embedded in the Docusaurus site accessible from any page
- **FR-002**: System MUST support two query modes: (1) Full book search, (2) Selected text only
- **FR-003**: System MUST retrieve relevant content from Qdrant vector database using semantic search (cosine similarity)
- **FR-004**: System MUST generate answers using OpenAI GPT models (gpt-4o-mini or gpt-4o)
- **FR-005**: System MUST include source citations in every answer linking back to specific textbook sections
- **FR-006**: Chat widget MUST allow users to toggle between "Ask about full book" and "Ask about selected text" modes
- **FR-007**: System MUST display a loading indicator during query processing
- **FR-008**: System MUST handle errors gracefully (Qdrant down, OpenAI timeout, etc.) with user-friendly messages

#### Content Ingestion
- **FR-009**: System MUST provide a script to ingest all markdown files from `textbook/docs/` directory
- **FR-010**: Ingestion MUST parse markdown, extract text, and preserve metadata (module, chapter, heading, URL)
- **FR-011**: Ingestion MUST chunk text using semantic boundaries (headings, paragraphs) with max 500 tokens per chunk
- **FR-012**: Ingestion MUST generate embeddings using OpenAI `text-embedding-3-small` model (1536 dimensions)
- **FR-013**: Ingestion MUST upsert vectors to Qdrant Cloud with payloads containing: text, module, chapter, heading, url, chunk_index
- **FR-014**: Ingestion MUST be idempotent (re-running does not create duplicates)
- **FR-015**: Ingestion MUST output a summary report: files processed, chunks created, embeddings generated, errors encountered

#### Agent System (Modular Intelligence)
- **FR-016**: System MUST implement a BaseAgent interface that all agents inherit from
- **FR-017**: System MUST include a BookSearchAgent that performs full-book vector search
- **FR-018**: System MUST include a SelectedTextAgent that restricts search to user-selected context
- **FR-019**: System MUST include a CitationAgent that formats source citations with links
- **FR-020**: System MUST include an AgentOrchestrator that routes queries to appropriate agents based on mode
- **FR-021**: Agents MUST be independently testable and composable (matrix-style skills)

#### API Layer
- **FR-022**: Backend MUST expose REST API endpoint: `POST /api/v1/chat/query` for chat queries
- **FR-023**: Backend MUST expose REST API endpoint: `POST /api/v1/ingest/documents` for triggering ingestion (admin only)
- **FR-024**: Backend MUST expose REST API endpoint: `GET /api/v1/health` for health checks
- **FR-025**: Backend MUST expose REST API endpoint: `GET /api/v1/collections/stats` for Qdrant collection statistics
- **FR-026**: All API endpoints MUST support CORS to allow Docusaurus frontend to call backend
- **FR-027**: API MUST validate request payloads using Pydantic models
- **FR-028**: API MUST return structured JSON responses with: answer, sources, agent_used, response_time

#### Frontend Integration
- **FR-029**: ChatWidget component MUST be built in React/TypeScript compatible with Docusaurus
- **FR-030**: ChatWidget MUST allow users to select text on the page and trigger "Ask about selection"
- **FR-031**: ChatWidget MUST display citations as clickable links that navigate to textbook sections
- **FR-032**: ChatWidget MUST maintain chat history during the session (clear on page reload)
- **FR-033**: ChatWidget MUST be responsive and work on mobile devices (320px+ width)

### Non-Functional Requirements

- **NFR-001**: System MUST respond to 90% of queries within 3 seconds (P95 latency)
- **NFR-002**: System MUST handle at least 10 concurrent users without performance degradation
- **NFR-003**: Qdrant collection MUST fit within Qdrant Cloud free tier limits (1GB storage, ~500K vectors)
- **NFR-004**: OpenAI API usage MUST stay within budget ($50/month for embeddings + completions estimated)
- **NFR-005**: Backend MUST log all queries, responses, errors to structured logs for debugging
- **NFR-006**: System MUST be deployable via Docker for easy local development and cloud deployment

### Key Entities

- **ChatQuery**: User question with metadata (query text, mode [full_book | selected_text], selected_text, filters)
- **ChatResponse**: Generated answer with sources (answer text, source citations, agent used, response time)
- **DocumentChunk**: A piece of textbook content (text, embedding vector, metadata: module/chapter/heading/url)
- **Citation**: Source reference (text snippet, url, module, chapter, heading, relevance score)
- **Agent**: Modular skill component (BookSearchAgent, SelectedTextAgent, CitationAgent)
- **AgentContext**: Input to agents (query, selected_text, filters, conversation_history)
- **AgentResponse**: Output from agents (answer, sources, confidence score)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot correctly answers 80%+ of test questions about textbook content (measured via human evaluation on 50 test questions)
- **SC-002**: All citations link to correct textbook sections (100% accuracy on 20 test queries)
- **SC-003**: Full textbook ingestion completes in under 5 minutes with 0 errors
- **SC-004**: 90% of queries return answers within 3 seconds (P95 latency measured via backend logs)
- **SC-005**: Qdrant collection contains 200-400 vectors after full ingestion (estimated based on 30 markdown files)
- **SC-006**: ChatWidget is accessible and functional on desktop (Chrome, Firefox, Safari) and mobile (iOS Safari, Android Chrome)
- **SC-007**: System handles 10 concurrent users without errors or timeouts (load test with Locust or k6)
- **SC-008**: Selected text mode restricts answers to only the selected context (verified via 10 test cases)
- **SC-009**: Backend API achieves 99% uptime during testing period (health check monitoring)
- **SC-010**: OpenAI API costs stay under $50/month for 1000 queries/month (cost monitoring)

## Assumptions

- OpenAI API key is available with sufficient quota for embeddings and completions
- Qdrant Cloud free tier account is set up and accessible
- Textbook markdown content is well-structured with headings and frontmatter metadata
- Docusaurus site is already deployed and accessible (Task 1 complete)
- Backend can be deployed to a cloud provider (Railway, Render, Fly.io, or similar)
- Users have modern browsers with JavaScript enabled
- No user authentication required for chatbot (public access)
- No conversation persistence across sessions (stateless chatbot for MVP)
- English is the only supported language for MVP

## Dependencies

- **Task 1 Complete**: Docusaurus textbook deployed with markdown content
- **OpenAI API**: Account with API key and credits
- **Qdrant Cloud**: Free tier account provisioned
- **Cloud Hosting**: Platform for deploying FastAPI backend (Railway/Render/Fly.io)
- **External Libraries**:
  - FastAPI 0.115+
  - Qdrant Client 1.11+
  - OpenAI SDK 1.54+
  - LangChain or similar for text chunking
  - Pydantic 2.0+
  - React 19 (already in Docusaurus)

## Out of Scope

- User authentication and personalized chat history
- Conversation memory across sessions (stateless for MVP)
- Multi-language support (Urdu translation deferred to Phase 2)
- Voice input/output for chatbot
- Image/diagram understanding (text-only RAG)
- Real-time collaboration (multiple users chatting together)
- Admin UI for managing Qdrant (command-line ingestion script only)
- Advanced RAG techniques (reranking, hybrid search, query expansion) - can be added later
- Chatbot analytics dashboard (basic logging only)
- Rate limiting per user (global rate limiting only if needed)

## Constitutional Compliance Check

| Principle                          | Status    | Notes                                                      |
|------------------------------------|-----------|-----------------------------------------------------------|
| I. Educational Excellence First    | Compliant | RAG chatbot enhances learning with Q&A and citations      |
| II. Progressive Enhancement        | Compliant | MVP focuses on core chat + citations (P1/P2 stories)      |
| III. User-Centric Personalization  | N/A       | Deferred to Phase 2 (no auth/personalization in MVP)      |
| IV. Multilingual Accessibility     | N/A       | Deferred to Phase 2 (English-only for MVP)                |
| V. AI-Native Development           | Compliant | Spec-driven approach, Agent-based architecture            |
| Technical Standards - Stack        | Compliant | FastAPI + OpenAI + Qdrant as specified                    |
| Technical Standards - Performance  | Compliant | <3s response time specified                               |
| Technical Standards - Accessibility| Compliant | Chat widget responsive, keyboard accessible               |
| Development Workflow               | Compliant | Spec created before implementation                        |

## Risk Analysis

| Risk | Impact | Mitigation |
|------|--------|------------|
| OpenAI API costs exceed budget | High | Implement caching for common queries; monitor usage daily |
| Qdrant free tier insufficient storage | Medium | Optimize chunking strategy; use smaller embedding model if needed |
| Selected text mode fails on certain browsers | Low | Test across browsers; fallback to full-book mode if selection API unavailable |
| Ingestion script fails on malformed markdown | Medium | Add error handling; validate markdown structure before processing |
| Chatbot hallucinates answers | High | Use strict retrieval thresholds; add disclaimer when confidence is low |
| Backend deployment costs too high | Medium | Use free tiers (Render, Fly.io); optimize resource usage |

## Next Steps

1. Create `plan.md` with architectural decisions and technology choices
2. Create `tasks.md` with detailed implementation checklist
3. Set up FastAPI backend skeleton
4. Implement Qdrant integration and test connection
5. Build ingestion pipeline and test with sample markdown
6. Implement agent system (BaseAgent, Orchestrator, BookSearchAgent)
7. Build API endpoints and test with Postman/curl
8. Create ChatWidget React component
9. Integrate ChatWidget into Docusaurus
10. End-to-end testing and deployment

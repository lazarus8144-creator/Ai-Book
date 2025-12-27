# Feature Specification: RAG Chatbot for Textbook Q&A

## Metadata
- Feature ID: 002-rag-chatbot
- Status: In Progress
- Priority: P1 (Required - 50 points)
- Dependencies: 001-docusaurus-textbook (Complete)
- Created: 2025-12-26
- Version: 1.0.0

## Problem Statement

Students reading the Physical AI & Humanoid Robotics textbook need quick answers to questions about course content without manually searching through all chapters. The current textbook has local search functionality, but students often need more contextual, conversational answers that synthesize information from multiple chapters.

## User Stories

### US-01: Ask Questions
**As a** student reading the textbook
**I want to** ask questions about textbook content in natural language
**So that** I can quickly find relevant information without manual searching

**Acceptance Criteria:**
- [ ] Question input accepts 5-500 characters
- [ ] Response appears in <3 seconds (p95)
- [ ] Answer cites specific textbook chapters/modules
- [ ] Sources are clickable links to relevant chapters
- [ ] Widget accessible on all textbook pages

### US-02: View Conversation History
**As a** student using the chatbot
**I want to** see my previous questions and answers in the current session
**So that** I can review what I've learned and reference earlier responses

**Acceptance Criteria:**
- [ ] Chat widget displays full conversation history
- [ ] Messages persist during page navigation within session
- [ ] History clears on browser refresh (Phase 1 - no persistence)
- [ ] Scroll to latest message automatically
- [ ] Can scroll up to view earlier messages

### US-03: Handle Irrelevant Questions
**As a** student
**I want** clear feedback when asking questions outside the textbook scope
**So that** I understand the chatbot's limitations and don't waste time

**Acceptance Criteria:**
- [ ] Off-topic questions receive "not covered in textbook" message
- [ ] No hallucinated answers about content not in textbook
- [ ] Chatbot constrained to textbook content only
- [ ] Helpful error messages for unclear questions

## Functional Requirements

### FR-001: RAG Pipeline
**Priority:** P0 (Critical)

The system MUST implement a Retrieval-Augmented Generation pipeline:
- System MUST embed user questions using OpenAI text-embedding-3-small (1536 dimensions)
- System MUST search Qdrant vector database for top 5 most relevant chunks
- System MUST use minimum similarity threshold of 0.7 for relevance
- System MUST generate answers using GPT-4o-mini with retrieved context
- System MUST cite sources with module name, chapter title, excerpt, and URL
- System MUST track response time and token usage for monitoring

### FR-002: Document Ingestion
**Priority:** P0 (Critical)

The system MUST process textbook content for RAG:
- System MUST process all 30+ markdown files from `textbook/docs/`
- System MUST use semantic chunking (header-based boundaries)
- System MUST respect 800 token maximum chunk size with 100 token overlap
- System MUST preserve code blocks intact (no mid-code splitting)
- System MUST extract metadata: module, chapter, title, URL, keywords from frontmatter
- System MUST generate deterministic chunk IDs for re-ingestion
- System MUST support incremental updates (add/update single documents)

### FR-003: Chatbot Widget
**Priority:** P0 (Critical)

The frontend MUST provide an accessible chat interface:
- Widget MUST appear as floating button in bottom-right corner (24px margin)
- Widget MUST open chat window (400px × 600px desktop, full-screen mobile)
- Widget MUST support mobile viewports (320px minimum)
- Widget MUST support Docusaurus dark mode (`[data-theme='dark']`)
- Widget MUST be keyboard accessible (Tab, Enter, Esc keys)
- Widget MUST meet WCAG 2.1 AA contrast requirements (4.5:1 minimum)
- Widget MUST display loading indicator during API requests
- Widget MUST handle API errors gracefully with user-friendly messages

### FR-004: API Endpoints
**Priority:** P0 (Critical)

The backend MUST expose the following REST API endpoints:

**POST /api/v1/query**
- Request: `{ "question": string (5-500 chars), "max_results"?: number (1-10) }`
- Response: `{ "answer": string, "sources": Source[], "response_time_ms": number, "tokens_used": number }`
- Returns 400 for invalid input, 429 for rate limit, 500 for errors

**GET /api/v1/health**
- Response: `{ "status": string, "qdrant_connected": boolean, "openai_available": boolean, "version": string }`

**POST /api/v1/ingest** (Admin only)
- Headers: `Authorization: Bearer <ADMIN_TOKEN>`
- Request: `{ "action": "reingest_all" | "add_document", "document_path"?: string }`
- Response: `{ "status": string, "documents_processed": number, "chunks_created": number }`

### FR-005: Rate Limiting
**Priority:** P1 (High)

The system MUST prevent abuse:
- System MUST limit queries to 20 per minute per IP address
- System MUST limit queries to 100 per hour per IP address
- System MUST return HTTP 429 with Retry-After header when limits exceeded
- System SHOULD implement exponential backoff for repeated violations

## Non-Functional Requirements

### Performance Requirements
**Priority:** P0 (Critical - Constitution Requirement)

- **RAG Response Time:** <3 seconds (p95) - MANDATORY
- **Vector Search:** <500ms per query
- **Page Load Impact:** <100ms additional overhead from widget
- **Concurrent Users:** Support 100+ simultaneous users without degradation
- **Backend Startup:** <30 seconds cold start on Railway

### Security Requirements
**Priority:** P0 (Critical - Constitution Requirement)

- **Transport:** HTTPS only for all API communication
- **CORS:** Restricted to deployed frontend origin (Vercel/GitHub Pages)
- **Secrets:** NO hardcoded API keys - environment variables only
- **Admin Endpoints:** Bearer token authentication required
- **Input Validation:** Sanitize all user input, prevent injection attacks
- **Rate Limiting:** Enforced on all public endpoints

### Accessibility Requirements
**Priority:** P0 (Critical - Constitution Requirement)

- **WCAG 2.1 AA Compliance:** All interactive elements MUST pass
- **Screen Reader:** Compatible with NVDA, JAWS, VoiceOver
- **Keyboard Navigation:** Full functionality without mouse
- **Color Contrast:** Minimum 4.5:1 for all text
- **Focus Indicators:** Visible focus states on all interactive elements
- **ARIA Labels:** Proper semantic markup and ARIA attributes

### Budget Constraints
**Priority:** P0 (Critical - Constitution Requirement)

- **Qdrant Cloud:** Free tier (1GB vectors) - estimated 15MB usage
- **OpenAI API:** Hackathon credits only (~$1.20/month estimated)
- **Backend Hosting:** Railway free tier (500 hours/month)
- **Neon Postgres:** NOT USED in Phase 1 (deferred to Phase 2)

## Technical Stack

### Mandated Technologies (per Constitution)

| Layer | Technology | Version | Purpose |
|-------|------------|---------|---------|
| Backend Framework | FastAPI | Latest | REST API, async support |
| Vector Database | Qdrant Cloud | Cloud | Document embeddings storage |
| AI Embeddings | OpenAI text-embedding-3-small | Latest | Question/document embeddings |
| AI LLM | OpenAI GPT-4o-mini | Latest | Answer generation |
| Frontend Framework | Docusaurus | 3.9.2 | Existing textbook platform |
| Frontend Language | TypeScript | 5.6.2 | Type-safe components |
| Backend Hosting | Railway | Cloud | Free tier, no cold starts |
| Frontend Hosting | GitHub Pages | Cloud | Existing deployment |

### Additional Dependencies

**Backend:**
- `uvicorn` - ASGI server
- `pydantic-settings` - Configuration management
- `slowapi` - Rate limiting
- `loguru` - Structured logging
- `python-multipart` - File upload support

**Frontend:**
- React 19 - UI components
- CSS Modules - Scoped styling

## Success Criteria

### Must Have (Phase 1 - Required for 100 points)

- [ ] Chatbot answers questions accurately from textbook content
- [ ] Response time <3s (p95) measured and documented
- [ ] Sources link correctly to textbook chapters
- [ ] Mobile responsive (tested on 320px, 768px, 1024px viewports)
- [ ] Dark mode support working correctly
- [ ] Accessibility audit passes (Lighthouse score ≥90)
- [ ] E2E tests passing in CI/CD
- [ ] Backend deployed to Railway with health check passing
- [ ] Frontend integrated via Root.tsx on all pages
- [ ] Performance benchmarks documented

### Should Have (Phase 2 - Bonus Features)

- [ ] Conversation history persists across browser sessions (requires auth)
- [ ] User feedback mechanism (thumbs up/down on answers)
- [ ] Response streaming for progressive display
- [ ] Analytics dashboard for popular questions

### Could Have (Future Enhancements)

- [ ] Multi-turn conversations with context awareness
- [ ] Suggested follow-up questions
- [ ] Question reformulation suggestions
- [ ] Inline citations within answer text
- [ ] Answer quality scoring
- [ ] Admin dashboard for monitoring

## Out of Scope

The following are explicitly **NOT** part of Phase 1:

- ❌ General robotics Q&A (only textbook content allowed)
- ❌ Multi-language support (English only in Phase 1)
- ❌ User authentication (Phase 2 feature)
- ❌ Conversation persistence across sessions (Phase 2)
- ❌ Voice input/output
- ❌ Image or diagram understanding
- ❌ Real-time collaboration features
- ❌ User-generated content
- ❌ Integration with external robotics documentation

## Dependencies

### External Dependencies

| Dependency | Status | Risk | Mitigation |
|------------|--------|------|------------|
| Textbook content complete | ✅ Complete | Low | Already done |
| OpenAI API access | ✅ Available | Low | Hackathon credits confirmed |
| Qdrant Cloud account | ⚠️ Need to create | Low | Free tier, 5min setup |
| Railway account | ⚠️ Need to create | Low | Free tier, 10min setup |
| GitHub repo access | ✅ Available | Low | Already setup |

### Internal Dependencies

| Dependency | Status | Impact |
|------------|--------|--------|
| Docusaurus build system | ✅ Working | Chatbot widget integration |
| Custom component patterns | ✅ Established | Follow LearningObjectives pattern |
| Dark mode theme | ✅ Working | Widget must support |
| E2E test infrastructure | ✅ Working | Add chatbot tests |

## Risks & Mitigations

### Risk Matrix

| Risk | Likelihood | Impact | Mitigation Strategy |
|------|------------|--------|---------------------|
| RAG responses too slow (>3s) | Medium | High | Use gpt-4o-mini (fastest), cache embeddings, optimize chunk retrieval to top 3 if needed |
| OpenAI quota exhausted | Low | High | Aggressive rate limiting (20/min), monitor usage daily, pre-cache common questions |
| Qdrant free tier exceeded (>1GB) | Very Low | Medium | Current usage 15MB (~1.5% of limit), monitor weekly |
| Railway free hours exhausted | Medium | High | Deploy 1 week before deadline, monitor usage, fallback to Render with cold starts |
| Poor answer quality | Medium | Medium | Improve prompt engineering, adjust similarity threshold, expand chunk size |
| CORS configuration errors | Low | Medium | Test early with deployed frontend, document CORS setup |
| Mobile UX issues | Low | Low | Test on real devices, use responsive breakpoints |

### Contingency Plans

**If RAG is too slow:**
1. Reduce max_tokens to 300 (from 500)
2. Reduce vector search to top 3 (from 5)
3. Implement aggressive caching of common questions
4. Accept performance violation and document

**If OpenAI quota exhausted:**
1. Switch to Groq API (free Llama 3.1)
2. Show "Service temporarily limited" message
3. Whitelist demo questions for presentation

**If Railway hours exhausted:**
1. Switch to Render (accept 15min cold starts)
2. Use ngrok tunnel to localhost for demo
3. Request Railway credit extension

## Compliance Checklist

### Constitution Compliance (v1.0.0)

- [x] **Spec-Driven Development:** This spec created before implementation
- [x] **Test-First Discipline:** Will write tests before implementation code
- [x] **Progressive Enhancement:** Phase 1 feature with no Phase 2 dependencies
- [x] **Performance Requirements:** <3s RAG, <2s page loads documented
- [x] **Accessibility Standards:** WCAG 2.1 AA compliance required
- [x] **Security Protocols:** HTTPS, no secrets, CORS, rate limiting
- [x] **Budget Constraints:** Free tiers only (Qdrant, Railway, OpenAI credits)
- [x] **Mandatory Tech Stack:** FastAPI, Qdrant, OpenAI, Docusaurus confirmed

### Quality Gates

Before merging to main:
1. [ ] Specification reviewed and approved ✅ (This document)
2. [ ] Implementation plan created (`plan.md`)
3. [ ] Tasks defined (`tasks.md`)
4. [ ] All unit tests passing
5. [ ] All integration tests passing
6. [ ] E2E tests passing
7. [ ] Performance benchmarks met
8. [ ] Accessibility audit passed
9. [ ] Code review completed
10. [ ] Documentation complete

## Acceptance Testing

### Test Scenarios

**Scenario 1: Successful Query**
```
GIVEN: User on textbook homepage
WHEN: User opens chatbot and asks "What is ROS 2?"
THEN:
  - Response appears in <3 seconds
  - Answer mentions "Robot Operating System"
  - At least 1 source shown
  - Source links to module-1-ros2 chapter
  - Source link navigates correctly when clicked
```

**Scenario 2: Irrelevant Query**
```
GIVEN: User on any textbook page
WHEN: User asks "What is the capital of France?"
THEN:
  - Response appears
  - Answer states "not covered in textbook"
  - No sources shown
  - Helpful message about chatbot scope
```

**Scenario 3: Mobile Experience**
```
GIVEN: User on mobile device (375px viewport)
WHEN: User opens chatbot
THEN:
  - Widget expands to full screen
  - Input and buttons remain tappable (min 44px)
  - Messages display correctly
  - Keyboard doesn't obscure input field
```

**Scenario 4: Dark Mode**
```
GIVEN: User has dark mode enabled
WHEN: User opens chatbot
THEN:
  - Widget uses dark theme colors
  - Text remains readable (contrast ≥4.5:1)
  - All colors follow Docusaurus dark theme
```

**Scenario 5: Rate Limiting**
```
GIVEN: User on any page
WHEN: User sends 21 questions in 1 minute
THEN:
  - First 20 queries succeed
  - 21st query returns 429 error
  - Error message explains rate limit
  - Retry-After header indicates wait time
```

## Appendix

### Glossary

- **RAG:** Retrieval-Augmented Generation - AI technique combining vector search with LLM generation
- **Chunk:** Segment of document text with metadata, used for vector search
- **Embedding:** High-dimensional vector representation of text for semantic similarity
- **Vector Search:** Finding similar chunks using cosine similarity of embeddings
- **Qdrant:** Open-source vector database optimized for similarity search
- **FastAPI:** Modern Python web framework for building APIs
- **CORS:** Cross-Origin Resource Sharing - security mechanism for web APIs

### Related Documents

- **Constitution:** `.specify/memory/constitution.md` (v1.0.0)
- **Roadmap:** `specs/000-project-roadmap/roadmap.md`
- **Docusaurus Spec:** `specs/001-docusaurus-textbook/spec.md`
- **Architecture Plan:** `specs/002-rag-chatbot/plan.md` (to be created)
- **Implementation Tasks:** `specs/002-rag-chatbot/tasks.md` (to be created)

### Open Questions

1. **Chunk Size Optimization:** Should we A/B test different chunk sizes (600 vs 800 tokens)?
2. **Similarity Threshold:** Is 0.7 optimal, or should we tune based on answer quality feedback?
3. **Response Caching:** Should we cache responses at all in Phase 1, or wait for Phase 2?

**Decisions:**
- Q1: Start with 800 tokens (standard), optimize if needed after testing
- Q2: Start with 0.7, adjust if too many "not found" results
- Q3: Wait for Phase 2 (simpler Phase 1 implementation)

---

**Document Status:** ✅ Approved for Implementation
**Spec Version:** 1.0.0
**Last Updated:** 2025-12-26
**Next Steps:** Create `plan.md` and `tasks.md`, begin implementation

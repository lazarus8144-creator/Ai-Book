# Implementation Plan: Content Personalization

**Branch**: `004-personalization` | **Date**: 2025-12-21 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-personalization/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Content personalization adapts textbook chapters based on authenticated user's learning profile (skill level, learning goals, prior experience). Users click "Personalize for Me" button on each chapter page to generate personalized versions with adjusted content depth, examples, and explanations matching their background. The system leverages the existing RAG chatbot backend with user context to generate personalized content while preserving original educational structure. Personalized versions are cached per user/chapter and can be toggled with original content.

**Technical Approach**: Integrate with Feature 003 (Authentication) user profiles and Feature 002 (RAG Chatbot) backend. Use OpenAI GPT-4o-mini with custom prompts incorporating user skill level, goals, and experience to generate adapted markdown. Cache personalized content in Neon Postgres to avoid regeneration. Provide React UI controls (personalize button, toggle, settings) in Docusaurus frontend.

## Technical Context

**Language/Version**:
- Backend: Python 3.11+ (FastAPI 0.115.0)
- Frontend: TypeScript 5.6.2 (React 19.0.0, Docusaurus 3.9.2)

**Primary Dependencies**:
- Backend: SQLAlchemy 2.0+, OpenAI Python SDK 1.54.0+, markdown-it-py 3.0+ (markdown parsing)
- Frontend: React 19, @docusaurus/core 3.9.2, react-markdown 9.0+ (rendering)

**Storage**:
- Neon Postgres (personalized content cache, user preferences)
- Existing Qdrant vector database (RAG retrieval, read-only)

**Testing**:
- Backend: pytest 8.0+, pytest-asyncio 0.24+ (async endpoints), hypothesis 6.0+ (property-based testing for content variants)
- Frontend: Jest 29+, React Testing Library 16+, Playwright 1.40+ (E2E personalization flows)

**Target Platform**:
- Backend: Linux server (FastAPI deployment, same as existing RAG backend)
- Frontend: Static site (GitHub Pages, Docusaurus build)

**Project Type**: Web (existing Docusaurus frontend + FastAPI backend)

**Performance Goals**:
- Personalized content generation: <15 seconds for 90% of chapters (SC-003)
- Cached content retrieval: <1 second (SC-008)
- Toggle between original/personalized: <200ms (instant UI swap)
- RAG backend throughput: 10+ concurrent personalization requests

**Constraints**:
- Zero breaking changes to existing textbook functionality (FR-027)
- Preserve code block syntax and structure (FR-006, SC-005: 100% correctness)
- OpenAI API rate limits: 3,500 RPM (tier 1), handle 429 errors gracefully
- Cache invalidation on profile updates or original content changes
- Free tier budget: Neon Postgres 0.5 GB limit (estimate 50 KB per personalized chapter = ~10,000 cached versions)

**Scale/Scope**:
- 21 chapters across 4 modules (from existing textbook)
- 3 skill levels (beginner, intermediate, advanced)
- Average chapter length: 2,000-5,000 words
- Expected personalization requests: 100-200 per day initially

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Educational Excellence First
- **Compliance**: Personalization enhances learning by adapting content depth to user skill level (FR-009 to FR-012). Beginners get simpler explanations, advanced users get implementation details.
- **Verification**: Success criteria SC-001 measures 30% faster completion for beginners with personalized content (measurable educational outcome).
- **Risk Mitigation**: AI-generated content quality requires spot-checking (Assumption 7); implement content validation to ensure technical accuracy preserved.

### ✅ II. Progressive Enhancement Architecture
- **Compliance**: Personalization is Phase 2 bonus feature (+50 points). Feature 001 (Docusaurus textbook) and Feature 002 (RAG chatbot) are complete (Phase 1: 100 points).
- **Independence**: FR-027 mandates maintaining all existing textbook functionality for non-personalized views. Original content always accessible.
- **Feature Flag**: Personalization opt-in via button click (FR-001); users can view original anytime (FR-015). System can be disabled without breaking textbook.
- **Testing**: Each user story includes independent test scenarios (US1-US4 can be tested separately).

### ✅ III. User-Centric Personalization
- **Compliance**: Background profiling captured in Feature 003 (skill level, learning goals, prior experience). FR-002 to FR-004 integrate all profile fields into personalization.
- **Meaningful Adaptation**: Content presentation adapted (explanation depth, example complexity), not just styling. FR-009 to FR-014 define skill-level-specific rules.
- **Opt-Out**: Non-authenticated users see original content (Edge Case 1). Authenticated users can view original anytime (FR-015).
- **Measurability**: SC-006 requires personalized content measurably different for different skill levels (keyword analysis variance).

### ⚠️ IV. Multilingual Accessibility
- **Deferred**: Urdu translation (Feature 005) will handle multilingual personalization. MVP supports English only (Assumption 9).
- **Future Integration**: Personalization engine designed to work with translated content (personalize Urdu chapters for Urdu-speaking users).

### ✅ V. AI-Native Development
- **Compliance**: Specification created via `/sp.specify`, architecture plan via `/sp.plan`, tasks via `/sp.tasks` (all workflow steps followed).
- **PHR Usage**: Prompt History Record created for spec creation (0001-personalization-specification-creation.spec.prompt.md).
- **ADR Candidates**: Will document OpenAI prompt engineering strategy, caching vs real-time generation tradeoff (Phase 0 research).

### ✅ Mandatory Tech Stack
- **Frontend**: Docusaurus 3.9.2 (required) - personalization UI integrated as React components
- **Backend**: FastAPI 0.115.0 (required) - personalization endpoints added to existing RAG backend
- **Database**: Neon Postgres (required) - stores PersonalizedContent and PersonalizationPreferences entities
- **Vector DB**: Qdrant Cloud (existing) - read-only access for RAG retrieval (no changes)
- **Authentication**: Better-auth (Feature 003 dependency) - user profiles provide personalization context
- **AI**: OpenAI Agents/API (required) - GPT-4o-mini for content adaptation

### ✅ Performance Requirements
- **RAG Integration**: Personalization uses RAG backend; must maintain <3s response time for chatbot (no degradation)
- **Page Loads**: Cached personalized content loads <1s (SC-008), within <2s page load requirement
- **Concurrency**: System must support 100+ concurrent users (caching reduces OpenAI API load)
- **Vector Search**: No new vector search latency (uses existing RAG retrieval, <500ms maintained)
- **Generation Latency**: 15-second personalization target (SC-003) acceptable for async operation with loading indicator

### ✅ Security Protocols
- **HTTPS**: All personalization API endpoints use HTTPS (inherited from existing backend)
- **Authentication Required**: FR-024 mandates authentication before personalization (Better-auth integration)
- **Data Encryption**: Personalized content stored in Neon Postgres (encrypted at rest by default)
- **Session Security**: JWT tokens from Feature 003 (cryptographically secure, httpOnly cookies)
- **Rate Limiting**: Apply existing rate limiting to personalization endpoints (prevent abuse)
- **CORS**: Configured for deployed frontend origin (no changes from existing setup)

### ✅ Accessibility Compliance
- **Keyboard Navigation**: "Personalize for Me" button, toggle controls, settings panel fully keyboard-accessible
- **Screen Readers**: ARIA labels for loading indicators (FR-007), button states (original/personalized), settings controls
- **Focus Indicators**: Visible focus on all interactive elements (buttons, toggles, sliders)
- **Color Contrast**: Loading states, error messages meet 4.5:1 contrast ratio
- **Alternative Text**: Loading spinner has aria-label "Personalizing content for your skill level"

### ✅ Spec-Driven Development
- **Specification**: Complete spec.md with 4 user stories, 32 functional requirements, 10 success criteria
- **Validation**: Passed 12/12 quality checklist criteria; zero NEEDS CLARIFICATION markers
- **Review**: Spec approved before planning (this phase)

### ✅ Test-First Discipline
- **Test Strategy**: TDD workflow for implementation:
  - Red: Write tests for personalization API, content adaptation, caching
  - Green: Implement endpoints to make tests pass
  - Refactor: Optimize prompt engineering, cache invalidation logic
- **Contract Tests**: Verify personalization API request/response schemas
- **Integration Tests**: Test full flow (authenticate → personalize → cache → retrieve → toggle)

### ✅ Quality Gates
- **Specification Gate**: ✅ Passed (spec.md complete, validated)
- **Plan Gate**: ⏳ In Progress (this document)
- **Test Gate**: ⏳ Pending (tasks.md will define test tasks)
- **Implementation Gate**: ⏳ Pending (implementation phase)
- **Constitution Gate**: ✅ Passed (13/13 gates compliant, 1 deferred to Feature 005)
- **Review Gate**: ⏳ Pending (post-implementation)

### ✅ Budget Constraint
- **Neon Postgres**: Free tier 0.5 GB (estimate 50 KB/chapter × 3 skill levels × 21 chapters = ~3.15 MB baseline; user cache grows linearly, ~10k cached versions within limit)
- **Qdrant Cloud**: No additional cost (uses existing RAG vector database, read-only)
- **OpenAI API**: GPT-4o-mini $0.150/1M input tokens, $0.600/1M output tokens (estimate $0.03 per personalization, caching reduces cost by 90%)
- **GitHub Pages**: Free (static site unchanged)
- **No Paid Services**: All infrastructure within free tiers or hackathon credits

## Project Structure

### Documentation (this feature)

```text
specs/004-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command) - OpenAI prompt engineering, caching strategies
├── data-model.md        # Phase 1 output (/sp.plan command) - PersonalizedContent, PersonalizationPreferences entities
├── quickstart.md        # Phase 1 output (/sp.plan command) - Setup guide for personalization feature
├── contracts/           # Phase 1 output (/sp.plan command) - OpenAPI spec for personalization endpoints
│   ├── personalization-api.yaml
│   └── types.ts
├── checklists/
│   └── requirements.md  # Spec quality validation (already created)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
rag-backend/
├── app/
│   ├── auth/                      # Existing (Feature 003)
│   │   ├── models.py              # User, LearningProfile models (read-only for personalization)
│   │   └── dependencies.py        # get_current_user dependency (reused)
│   ├── personalization/           # NEW: Personalization feature
│   │   ├── __init__.py
│   │   ├── models.py              # PersonalizedContent, PersonalizationPreferences SQLAlchemy models
│   │   ├── schemas.py             # Pydantic request/response schemas
│   │   ├── service.py             # PersonalizationService (core logic)
│   │   ├── prompts.py             # OpenAI prompt templates per skill level
│   │   ├── router.py              # FastAPI endpoints (/personalize, /preferences, /recommendations)
│   │   └── cache.py               # Cache invalidation logic
│   ├── rag/                       # Existing (Feature 002)
│   │   └── service.py             # RAG retrieval (reused for personalization context)
│   └── main.py                    # Add personalization router
├── tests/
│   ├── personalization/           # NEW
│   │   ├── test_service.py        # Unit tests for PersonalizationService
│   │   ├── test_prompts.py        # Verify prompt templates per skill level
│   │   ├── test_cache.py          # Cache hit/miss, invalidation tests
│   │   └── test_api.py            # Integration tests for endpoints
│   └── integration/
│       └── test_personalization_flow.py  # E2E: auth → personalize → cache → toggle

textbook/
├── src/
│   ├── components/
│   │   ├── PersonalizeButton/     # NEW: "Personalize for Me" button component
│   │   │   ├── index.tsx
│   │   │   ├── PersonalizeButton.module.css
│   │   │   └── PersonalizeButton.test.tsx
│   │   ├── ContentToggle/         # NEW: Original/Personalized toggle
│   │   │   ├── index.tsx
│   │   │   └── ContentToggle.module.css
│   │   ├── PersonalizationSettings/  # NEW: Settings panel (verbosity, examples)
│   │   │   ├── index.tsx
│   │   │   ├── SettingsPanel.tsx
│   │   │   └── PersonalizationSettings.module.css
│   │   └── ChapterRecommendations/   # NEW: "What's Next?" recommendations
│   │       ├── index.tsx
│   │       └── Recommendations.module.css
│   ├── hooks/
│   │   ├── usePersonalization.ts  # NEW: Custom hook for personalization state
│   │   └── usePreferences.ts      # NEW: Settings state management
│   ├── services/
│   │   └── personalizationApi.ts  # NEW: API client for personalization endpoints
│   └── theme/
│       └── MDXComponents/         # Modified: Wrap MDXContent to support personalization
│           └── index.tsx
└── tests/
    └── e2e/
        └── personalization.spec.ts  # NEW: Playwright E2E tests
```

**Structure Decision**: Web application (frontend + backend). Personalization feature adds new modules to existing `rag-backend/app/` (backend) and `textbook/src/components/` (frontend). No new directories at repo root. Leverages existing authentication (Feature 003) and RAG (Feature 002) infrastructure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | All gates passed | N/A |

**Note**: Constitution gate IV (Multilingual Accessibility) deferred to Feature 005 (Urdu Translation) by design. Personalization MVP supports English only (Assumption 9 in spec). Feature 005 will extend personalization to work with translated content. This is not a violation but intentional phased delivery per Progressive Enhancement Architecture (Principle II).

## Phase 0: Research & Technology Decisions

*See [research.md](./research.md) for detailed findings*

### Research Tasks

1. **OpenAI Prompt Engineering for Skill-Level Adaptation**
   - Research: Best practices for GPT-4o-mini prompts that adapt content depth
   - Deliverable: Prompt templates for beginner, intermediate, advanced (with examples)
   - Success: Measurably different outputs for same chapter at different skill levels

2. **Content Caching Strategy**
   - Research: Cache invalidation triggers (profile update, original content change)
   - Deliverable: Decision on cache key structure (User ID + Chapter ID + content hash)
   - Success: <1s cached retrieval (SC-008), 90% cache hit rate after initial personalization

3. **Markdown Parsing and Preservation**
   - Research: Parse markdown to preserve code blocks, headings, structure (FR-005, FR-006)
   - Deliverable: Library choice (markdown-it-py vs marko) and code block extraction strategy
   - Success: 100% code syntax correctness after personalization (SC-005)

4. **Chapter Length Handling**
   - Research: Strategies for long chapters (10k+ words) within 15-second target
   - Deliverable: Decision on chunking strategy (section-by-section vs full chapter)
   - Success: 90% of chapters personalize within 15 seconds (SC-003)

5. **Personalization Quality Measurement**
   - Research: Metrics to verify content variance (keyword analysis, readability scores)
   - Deliverable: Automated tests for SC-006 (measurable differences between skill levels)
   - Success: Objective measurement of personalization effectiveness

6. **Recommendation Algorithm**
   - Research: Logic for personalized chapter recommendations based on goals/experience
   - Deliverable: Recommendation scoring function (goal match + prerequisite check)
   - Success: 75% recommendation accuracy (SC-010)

## Phase 1: Data Model & Contracts

*See [data-model.md](./data-model.md) and [contracts/](./contracts/) for complete specifications*

### Entities Summary

1. **PersonalizedContent** (new table in Neon Postgres)
   - Primary key: UUID
   - Foreign keys: user_id (User), chapter_id (string)
   - Fields: personalized_markdown, original_content_hash, created_at, last_accessed_at, is_invalid
   - Indexes: (user_id, chapter_id) unique, (last_accessed_at) for cache eviction

2. **PersonalizationPreferences** (new table in Neon Postgres)
   - Primary key: user_id (one-to-one with User)
   - Fields: verbosity_level (enum: concise/moderate/detailed), example_preference (enum: theoretical/practical/mixed), code_comment_depth (enum: minimal/standard/extensive)
   - Created/updated timestamps

3. **ChapterRecommendation** (new table in Neon Postgres)
   - Composite key: (user_id, current_chapter_id)
   - Fields: recommended_chapter_ids (array, max 3), reasoning (JSON), generated_at
   - TTL: 7 days (recommendations refresh weekly)

### API Endpoints Summary

- `POST /api/v1/personalization/personalize` - Generate personalized chapter
  - Request: `{ chapter_id, user_id (from JWT), force_regenerate }`
  - Response: `{ personalized_content, from_cache, generation_time_ms }`

- `GET /api/v1/personalization/chapter/{chapter_id}` - Retrieve personalized or original
  - Query: `?view=personalized|original`
  - Response: `{ content, view_type, last_updated }`

- `GET /api/v1/personalization/preferences` - Get user preferences
  - Response: `{ verbosity_level, example_preference, code_comment_depth }`

- `PUT /api/v1/personalization/preferences` - Update preferences
  - Request: `{ verbosity_level?, example_preference?, code_comment_depth? }`
  - Side effect: Invalidates all cached personalized content for user

- `GET /api/v1/personalization/recommendations/{chapter_id}` - Get next chapter suggestions
  - Response: `{ recommendations: [{ chapter_id, title, reason }] }`

## Integration Strategy

### With Feature 003 (Authentication)

- **User Profiles**: Read-only access to `User` and `LearningProfile` tables
- **JWT Integration**: Reuse `get_current_user` dependency for authentication
- **Profile Data**: Extract skill_level, learning_goals, prior_experience for personalization prompts

### With Feature 002 (RAG Chatbot)

- **RAG Service**: Import `RAGService.retrieve()` to get relevant textbook context for personalization
- **Vector Search**: Use existing Qdrant embeddings to find similar examples for skill-level adaptation
- **OpenAI Client**: Reuse existing OpenAI client configuration (API key, error handling)

### With Existing Textbook (Feature 001)

- **MDX Components**: Wrap `MDXContent` to intercept rendering and swap personalized content
- **Chapter Metadata**: Read chapter frontmatter (title, module, chapter_id) for personalization API calls
- **URL State**: Add `?view=personalized` query param to persist user preference across navigation

## Security Considerations

1. **Authentication Enforcement**: All personalization endpoints require valid JWT (Feature 003)
2. **User Data Isolation**: PersonalizedContent table has user_id foreign key; queries filtered by authenticated user
3. **Content Injection Prevention**: Sanitize AI-generated markdown before rendering (DOMPurify on frontend)
4. **Rate Limiting**: Apply existing rate limiter to personalization endpoints (max 10 personalizations per user per hour)
5. **Cache Poisoning**: PersonalizedContent records immutable (invalidate and regenerate, never update in place)

## Performance Optimizations

1. **Caching**: PostgreSQL table with indexes on (user_id, chapter_id) for <1s retrieval
2. **Lazy Loading**: Personalization triggered by button click (not automatic on page load)
3. **Progress Indicators**: 15-second generation shows estimated time and loading spinner (FR-007)
4. **Background Processing**: Consider Celery task queue for long chapters (future enhancement)
5. **Content Hash**: Detect original chapter changes and invalidate cache (prevent stale personalized versions)

## Rollback Plan

If personalization feature causes issues:

1. **Feature Flag Disable**: Remove "Personalize for Me" button from UI (single-line config change)
2. **Backend Graceful Degradation**: If OpenAI API fails, return original content with error message (Edge Case)
3. **Database Rollback**: Drop PersonalizedContent tables if needed (no foreign key dependencies from other features)
4. **Zero Impact on Phase 1**: Original textbook and RAG chatbot unaffected (FR-027 compliance)

## Testing Strategy

- **Unit Tests**: PersonalizationService logic, prompt template rendering, cache invalidation rules
- **Integration Tests**: Full API endpoint flows with mocked OpenAI (test caching, error handling)
- **Contract Tests**: Verify personalization API adheres to OpenAPI spec (request/response schemas)
- **E2E Tests**: Playwright scenarios for US1-US4 (authenticate → personalize → toggle → settings)
- **Performance Tests**: Measure generation time for chapters of varying lengths (validate SC-003)
- **Quality Tests**: Automated checks for code block syntax correctness (SC-005), content variance (SC-006)

## Deployment Notes

- **Database Migration**: Alembic migration to create PersonalizedContent, PersonalizationPreferences, ChapterRecommendation tables
- **Environment Variables**: `PERSONALIZATION_ENABLED=true` (feature flag), `MAX_PERSONALIZATION_LENGTH=10000` (word limit)
- **Monitoring**: Log personalization generation times, cache hit rates, OpenAI API errors
- **Cost Tracking**: Monitor OpenAI API usage (estimate $3/day for 100 personalizations with caching)

## Success Metrics (from spec.md)

Implementation will be validated against these criteria:

- **SC-001**: Beginners complete chapters 30% faster with personalized content (analytics tracking)
- **SC-002**: 80% user retention for personalization feature (usage analytics)
- **SC-003**: Personalized content generation <15 seconds for 90% of chapters (performance monitoring)
- **SC-005**: 100% code block syntax correctness (automated AST validation)
- **SC-006**: Measurable content variance between skill levels (keyword/readability analysis)
- **SC-008**: Cached content loads <1 second (database query timing)

## Next Steps

1. **Phase 0 Complete**: Generate [research.md](./research.md) with detailed technology decisions
2. **Phase 1 Complete**: Generate [data-model.md](./data-model.md), [contracts/](./contracts/), [quickstart.md](./quickstart.md)
3. **Update Agent Context**: Run `.specify/scripts/bash/update-agent-context.sh claude` to add personalization technologies
4. **Re-Validate Constitution**: Verify Phase 1 design still passes all gates
5. **Task Generation**: Run `/sp.tasks` to decompose into granular implementation tasks
6. **Implementation**: Execute tasks in priority order (US1 P1 first, then US2/US4 P2, finally US3 P3)

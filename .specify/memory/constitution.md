<!--
=============================================================================
SYNC IMPACT REPORT
=============================================================================
Version Change: (new) → v1.0.0
Ratification: Initial constitution creation

Added Principles:
- I. Educational Excellence First
- II. Progressive Enhancement Architecture
- III. User-Centric Personalization
- IV. Multilingual Accessibility
- V. AI-Native Development

Added Sections:
- Technical Standards (mandatory tech stack, performance, security, accessibility)
- Development Workflow (spec-driven, test-first, quality gates, documentation)
- Scope & Boundaries (phased delivery with point allocations)

Templates Status:
✅ .specify/templates/plan-template.md - Compatible (Constitution Check section present)
✅ .specify/templates/spec-template.md - Compatible (requirements/success criteria aligned)
✅ .specify/templates/tasks-template.md - Compatible (phased approach matches project phases)
⚠️ No command templates found in .specify/templates/commands/

Follow-up TODOs: None

=============================================================================
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Educational Excellence First

Content quality and pedagogical effectiveness MUST take precedence over technical novelty.

- All textbook content MUST be technically accurate and peer-reviewable
- Learning objectives MUST be clearly stated for each module and chapter
- Code examples MUST be tested, runnable, and educational (not just functional)
- Complex concepts MUST include progressive explanations: overview → detail → hands-on
- Content MUST follow the established module structure:
  - Module 1: Robotic Nervous System (ROS 2)
  - Module 2: Digital Twin (Gazebo & Unity)
  - Module 3: AI-Robot Brain (NVIDIA Isaac)
  - Module 4: Vision-Language-Action (VLA)
- Hardware requirements and lab setups MUST be documented with clear prerequisites

**Rationale**: A hackathon textbook project succeeds only if learners can actually learn from it. Technical sophistication means nothing if the educational value is compromised.

### II. Progressive Enhancement Architecture

Base functionality (100 points) MUST be fully working before any bonus features are attempted.

- Phase 1 (Required - 100 pts) MUST be complete and deployed before Phase 2 begins:
  - Docusaurus textbook with all module content
  - RAG chatbot answering questions about book content
- Phase 2 (Bonus - 150 pts) MUST NOT break Phase 1 functionality:
  - Authentication with background profiling (+50 pts)
  - Content personalization for logged-in users (+50 pts)
  - Urdu translation for logged-in users (+50 pts)
- Phase 3 (Bonus - 50 pts) builds on Phase 2:
  - Claude Code Subagents and Agent Skills integration
- Each phase MUST have independent test coverage
- Feature flags MUST isolate bonus features from core functionality

**Rationale**: Hackathon scoring prioritizes working deliverables. A perfect bonus feature on a broken base scores zero.

### III. User-Centric Personalization

Meaningful customization MUST be based on user background and learning preferences.

- Background profiling MUST capture relevant context:
  - Technical experience level (beginner/intermediate/advanced)
  - Domain familiarity (robotics, AI, programming)
  - Learning goals (academic, professional, hobby)
- Personalization MUST adapt content presentation, not just styling
- Users MUST be able to opt out of profiling without losing core functionality
- Personalization data MUST be stored securely and used only for educational enhancement
- The "personalize chapter" feature MUST produce measurably different content for different user profiles

**Rationale**: True personalization improves learning outcomes. Surface-level customization wastes development effort.

### IV. Multilingual Accessibility

Urdu translation MUST maintain technical accuracy and educational value.

- Technical terms MUST be handled consistently:
  - Preserve English terms where Urdu equivalents are uncommon or ambiguous
  - Provide glossaries mapping English technical terms to Urdu explanations
- Code samples MUST remain in English (programming language syntax)
- Comments and explanatory text in code MUST be translated
- Translation MUST be reviewable and correctable by native speakers
- RTL (right-to-left) layout MUST be properly supported in Docusaurus
- Translated content MUST pass the same educational quality checks as English content

**Rationale**: Poor translation actively harms learning. Technical accuracy cannot be sacrificed for localization completeness.

### V. AI-Native Development

Claude Code Subagents and Spec-Kit Plus MUST be leveraged throughout the workflow.

- All features MUST have specifications before implementation (`/sp.specify`)
- Implementation plans MUST be generated and reviewed (`/sp.plan`)
- Tasks MUST be tracked via the task system (`/sp.tasks`)
- Architectural decisions MUST be documented as ADRs (`/sp.adr`)
- Prompt History Records (PHRs) MUST capture significant AI interactions
- Agent Skills MUST be created for reusable intelligence patterns:
  - Content generation skills for textbook chapters
  - RAG integration skills for chatbot responses
  - Translation skills for Urdu conversion
- Subagents SHOULD be used for parallelizable tasks where appropriate

**Rationale**: This is an AI-native development hackathon. Demonstrating effective AI tooling usage is itself a deliverable.

## Technical Standards

### Mandatory Tech Stack

The following technologies are REQUIRED and MUST NOT be substituted without ADR justification:

| Layer | Technology | Purpose |
|-------|------------|---------|
| Frontend | Docusaurus | Static site generation for textbook |
| Backend API | FastAPI | RAG chatbot API and user services |
| Primary Database | Neon Postgres | User data, profiles, preferences |
| Vector Database | Qdrant Cloud | Document embeddings for RAG |
| Authentication | Better-auth | User authentication and sessions |
| Hosting | GitHub Pages | Zero-cost static hosting |
| AI Orchestration | OpenAI Agents | RAG chatbot implementation |

### Performance Requirements

- RAG chatbot responses MUST complete in <3 seconds (p95)
- Page loads MUST complete in <2 seconds (p95)
- System MUST support 100+ concurrent users without degradation
- Vector search MUST return relevant results in <500ms
- Authentication flows MUST complete in <1 second

### Security Protocols

- All API endpoints MUST use HTTPS
- User passwords MUST be hashed using bcrypt or argon2
- Session tokens MUST be cryptographically secure and expire appropriately
- User data MUST be encrypted at rest in Neon Postgres
- API keys and secrets MUST use environment variables, never hardcoded
- CORS MUST be configured to allow only the deployed frontend origin
- Rate limiting MUST be implemented on authentication endpoints

### Accessibility Compliance

- All interactive elements MUST meet WCAG 2.1 AA standards
- Keyboard navigation MUST be fully supported
- Screen reader compatibility MUST be tested
- Color contrast ratios MUST meet AA requirements (4.5:1 for normal text)
- Focus indicators MUST be visible
- Alternative text MUST be provided for all images and diagrams

## Development Workflow

### Spec-Driven Development

- NO implementation begins without a specification in `specs/<feature>/spec.md`
- Specifications MUST include:
  - User stories with acceptance criteria
  - Functional requirements (FR-XXX format)
  - Success criteria with measurable outcomes
- Specifications MUST be reviewed before planning begins

### Test-First Discipline

- Tests MUST be written before implementation code
- Tests MUST fail before implementation (Red phase)
- Implementation MUST make tests pass (Green phase)
- Code MUST be refactored while maintaining passing tests (Refactor phase)
- Contract tests MUST verify API boundaries
- Integration tests MUST verify user journeys

### Quality Gates

Every feature MUST pass these gates before merge:

1. **Specification Gate**: Spec exists and is approved
2. **Plan Gate**: Implementation plan reviewed
3. **Test Gate**: Tests written and initially failing
4. **Implementation Gate**: All tests passing
5. **Constitution Gate**: Compliance with all principles verified
6. **Review Gate**: Code review completed

### Documentation as Code

- PHRs MUST be created for significant development sessions
- ADRs MUST be created for architectural decisions
- Specs, plans, and tasks are version-controlled deliverables
- README MUST be kept current with setup instructions
- API documentation MUST be auto-generated from code

## Scope & Boundaries

### In Scope

| Phase | Deliverable | Points | Status |
|-------|-------------|--------|--------|
| 1 | Docusaurus textbook (4 modules) | 50 | Required |
| 1 | RAG chatbot with OpenAI Agents | 50 | Required |
| 2 | Better-auth authentication | 50 | Bonus |
| 2 | Content personalization | 50 | Bonus |
| 2 | Urdu translation | 50 | Bonus |
| 3 | Claude Subagents integration | 50 | Bonus |

**Maximum Score**: 300 points (100 required + 200 bonus)

### Out of Scope

The following are explicitly NOT part of this project:

- Mobile applications (iOS, Android)
- Languages beyond English and Urdu
- Advanced AI features not specified in hackathon requirements
- Video content or multimedia beyond static images
- Real-time collaboration features
- Paid hosting or infrastructure
- User-generated content or community features

### Boundary Decisions

- Chatbot MUST answer questions about textbook content only (not general robotics)
- Personalization MUST affect content presentation, not curriculum structure
- Translation MUST be button-triggered, not automatic language detection

## Governance

### Amendment Procedure

1. Propose amendment via PR to `constitution.md`
2. Document rationale and impact analysis
3. Verify no breaking changes to in-progress features
4. Update version number according to semantic versioning
5. Update `LAST_AMENDED_DATE` to amendment date
6. Propagate changes to dependent templates if needed

### Versioning Policy

- **MAJOR** (X.0.0): Principle removal, redefinition, or backward-incompatible governance changes
- **MINOR** (0.X.0): New principle/section added or materially expanded guidance
- **PATCH** (0.0.X): Clarifications, wording fixes, non-semantic refinements

### Compliance Review

- Every PR MUST include a Constitution Compliance statement
- Reviewers MUST verify alignment with all applicable principles
- Violations MUST be documented and justified in Complexity Tracking (plan.md)
- No merge without compliance verification

### Budget Constraint

All infrastructure MUST use free tiers:
- GitHub Pages (free for public repos)
- Neon Postgres (free tier: 0.5 GB storage)
- Qdrant Cloud (free tier: 1GB vectors)
- No paid APIs except within hackathon-provided credits

### Timeline Alignment

- **Submission Deadline**: November 30, 2025
- **Live Presentations**: November 30, 2025 (6:00 PM)
- All development MUST respect this immovable deadline
- Scope reduction preferred over deadline extension

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06

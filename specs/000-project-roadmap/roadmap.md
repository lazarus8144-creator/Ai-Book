# Project Roadmap: Physical AI & Humanoid Robotics Textbook

**Project**: Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-06
**Deadline**: November 30, 2025, 6:00 PM
**Constitution**: v1.0.0
**Maximum Score**: 300 points (100 required + 200 bonus)

---

## Executive Summary

This roadmap outlines the development plan for the hackathon project, delivering:
- **Phase 1 (Required)**: Docusaurus textbook + RAG chatbot (100 pts)
- **Phase 2 (Bonus)**: Authentication + Personalization + Urdu translation (150 pts)
- **Phase 3 (Bonus)**: Claude Code Subagents integration (50 pts)

---

## 1. Project Phases & Milestones

### Phase 1: Core Deliverables (100 Points) - REQUIRED

| Milestone | Deliverable | Points | Target Date |
|-----------|-------------|--------|-------------|
| M1.1 | Docusaurus site deployed to GitHub Pages | 25 | Week 1 |
| M1.2 | All 4 module content complete | 25 | Week 2 |
| M1.3 | RAG backend deployed (FastAPI + Qdrant) | 25 | Week 2 |
| M1.4 | Chatbot integrated and functional | 25 | Week 3 |

**Phase 1 Definition of Done**:
- [ ] Textbook accessible at GitHub Pages URL
- [ ] All 4 modules with chapters browsable
- [ ] Search functionality working
- [ ] RAG chatbot answers questions about textbook content
- [ ] <2s page loads, <3s RAG responses
- [ ] WCAG 2.1 AA compliance verified

### Phase 2: Bonus Features (150 Points)

| Milestone | Deliverable | Points | Target Date |
|-----------|-------------|--------|-------------|
| M2.1 | Better-auth authentication working | 50 | Week 3 |
| M2.2 | Content personalization functional | 50 | Week 4 |
| M2.3 | Urdu translation via button | 50 | Week 4 |

**Phase 2 Definition of Done**:
- [ ] Users can register/login
- [ ] User profiles capture background info
- [ ] Personalize button adapts content to user
- [ ] Translate button converts to Urdu
- [ ] RTL layout works correctly
- [ ] Phase 1 still fully functional

### Phase 3: Advanced Integration (50 Points)

| Milestone | Deliverable | Points | Target Date |
|-----------|-------------|--------|-------------|
| M3.1 | Claude Subagents for content generation | 25 | Week 4 |
| M3.2 | Agent Skills for translation/quiz generation | 25 | Week 4 |

**Phase 3 Definition of Done**:
- [ ] Subagents demonstrate reusable intelligence
- [ ] Agent Skills documented and functional
- [ ] Integration doesn't break Phase 1/2

---

## 2. Feature Specifications Required

| Feature | Spec Path | Status | Priority |
|---------|-----------|--------|----------|
| Docusaurus Textbook | `specs/001-docusaurus-textbook/spec.md` | ✅ Complete | P1 |
| RAG Chatbot | `specs/002-rag-chatbot/spec.md` | ❌ Needed | P1 |
| Better-auth Integration | `specs/003-authentication/spec.md` | ❌ Needed | P2 |
| Content Personalization | `specs/004-personalization/spec.md` | ❌ Needed | P2 |
| Urdu Translation | `specs/005-urdu-translation/spec.md` | ❌ Needed | P2 |
| Claude Subagents | `specs/006-subagents/spec.md` | ❌ Needed | P3 |

---

## 3. Tasks & Dependencies

### Critical Path (Must Complete in Order)

```
[Docusaurus Setup] → [Module Content] → [RAG Backend] → [Chatbot Integration]
         ↓                                    ↓
    [GitHub Pages]                    [Qdrant Vectors]
                                            ↓
                               [Authentication] → [Personalization]
                                                        ↓
                                              [Urdu Translation]
                                                        ↓
                                              [Claude Subagents]
```

### Task Breakdown

#### Week 1: Foundation

| Task ID | Task | Dependencies | Est. Hours |
|---------|------|--------------|------------|
| T001 | Initialize Docusaurus project | None | 2 |
| T002 | Configure docusaurus.config.ts | T001 | 2 |
| T003 | Create custom components (LearningObjectives, etc.) | T001 | 4 |
| T004 | Set up GitHub Actions deployment | T001 | 2 |
| T005 | Create Module 1 structure (ROS 2) | T002 | 4 |
| T006 | Write Module 1 content (chapters 1-3) | T005 | 8 |
| T007 | Create Module 2 structure (Digital Twin) | T002 | 2 |
| T008 | Write `/sp.specify` for RAG Chatbot | None | 3 |

**Week 1 Total**: ~27 hours

#### Week 2: Content & Backend

| Task ID | Task | Dependencies | Est. Hours |
|---------|------|--------------|------------|
| T009 | Write Module 1 remaining chapters | T006 | 6 |
| T010 | Write Module 2 content | T007 | 8 |
| T011 | Create Module 3 structure (NVIDIA Isaac) | T002 | 2 |
| T012 | Write Module 3 content | T011 | 8 |
| T013 | Create Module 4 structure (VLA) | T002 | 2 |
| T014 | Write Module 4 content | T013 | 8 |
| T015 | Initialize FastAPI backend | T008 spec | 3 |
| T016 | Set up Neon Postgres | T015 | 2 |
| T017 | Set up Qdrant Cloud | T015 | 2 |
| T018 | Implement document embedding pipeline | T017 | 4 |
| T019 | Create RAG query endpoint | T018 | 4 |

**Week 2 Total**: ~49 hours

#### Week 3: Integration & Bonus Start

| Task ID | Task | Dependencies | Est. Hours |
|---------|------|--------------|------------|
| T020 | Integrate chatbot widget into Docusaurus | T019 | 4 |
| T021 | Test RAG responses end-to-end | T020 | 3 |
| T022 | Performance optimization (<3s responses) | T021 | 4 |
| T023 | Accessibility audit and fixes | T014 | 4 |
| T024 | Write `/sp.specify` for Authentication | T021 | 2 |
| T025 | Implement Better-auth | T024 spec | 6 |
| T026 | Create user profile schema | T025 | 3 |
| T027 | Build profile collection UI | T026 | 4 |
| T028 | Write `/sp.specify` for Personalization | T025 | 2 |

**Week 3 Total**: ~32 hours

#### Week 4: Bonus Features & Polish

| Task ID | Task | Dependencies | Est. Hours |
|---------|------|--------------|------------|
| T029 | Implement personalization logic | T028 spec | 6 |
| T030 | Create "Personalize" button component | T029 | 3 |
| T031 | Write `/sp.specify` for Urdu Translation | T029 | 2 |
| T032 | Implement translation API | T031 spec | 4 |
| T033 | Create "Translate to Urdu" button | T032 | 3 |
| T034 | Implement RTL layout support | T033 | 4 |
| T035 | Write `/sp.specify` for Subagents | T029 | 2 |
| T036 | Create Claude Subagent skills | T035 spec | 6 |
| T037 | Final integration testing | T036 | 4 |
| T038 | Presentation preparation | T037 | 4 |

**Week 4 Total**: ~38 hours

---

## 4. Timeline (Gantt View)

```
                    Week 1          Week 2          Week 3          Week 4        DEADLINE
                    |---------------|---------------|---------------|---------------|
Docusaurus Setup    ████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
Module Content      ░░░░████████████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
RAG Backend         ░░░░░░░░░░░░░░░░████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
Chatbot Integration ░░░░░░░░░░░░░░░░░░░░░░░░░░░░████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░
                    |--- PHASE 1 COMPLETE (100 pts) ---|
Authentication      ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████████░░░░░░░░░░░░░░░░░░░░
Personalization     ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████████░░░░░░░░░░░░
Urdu Translation    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████████░░░░
                    |------- PHASE 2 COMPLETE (250 pts) -------|
Claude Subagents    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████
                    |---- PHASE 3 COMPLETE (300 pts) ----|
Presentation        ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██
                                                                              Nov 30
```

### Critical Path Highlight

**Textbook → RAG Backend → Chatbot Integration** is the critical path.
- Any delay in textbook content delays RAG training data
- RAG backend must be functional before chatbot integration
- Phase 1 MUST be complete before bonus features

---

## 5. Resource Allocation

### Tech Stack Components

| Component | Technology | Free Tier Limits | Usage |
|-----------|------------|------------------|-------|
| Frontend | Docusaurus 3.x | Unlimited (static) | Textbook site |
| Hosting | GitHub Pages | Unlimited (public repo) | Static hosting |
| Backend | FastAPI | N/A (self-hosted) | RAG API |
| Database | Neon Postgres | 0.5 GB storage | User data |
| Vectors | Qdrant Cloud | 1 GB vectors | RAG embeddings |
| Auth | Better-auth | Open source | User sessions |
| AI | OpenAI API | Hackathon credits | RAG + Translation |

### Team Roles (Hypothetical)

| Role | Responsibilities | Tasks |
|------|------------------|-------|
| Content Lead | Textbook authoring | T005-T014 |
| Frontend Dev | Docusaurus, components | T001-T004, T020, T030, T033 |
| Backend Dev | FastAPI, databases | T015-T019, T025-T026 |
| AI/ML Engineer | RAG, embeddings, Subagents | T018-T019, T029, T032, T036 |
| QA Lead | Testing, accessibility | T021-T023, T037 |

---

## 6. Validation & Quality Gates

### Gate 1: Phase 1 Checkpoint (End of Week 2)

| Check | Criteria | Pass/Fail |
|-------|----------|-----------|
| Textbook Deployed | Site accessible at GitHub Pages URL | |
| Content Complete | All 4 modules have chapter content | |
| Search Working | Local search returns relevant results | |
| Performance | Page loads < 2 seconds | |
| Accessibility | Lighthouse score 90+ | |

### Gate 2: RAG Integration (End of Week 3)

| Check | Criteria | Pass/Fail |
|-------|----------|-----------|
| RAG Functional | Chatbot answers questions correctly | |
| Response Time | RAG responses < 3 seconds | |
| Relevance | Answers sourced from textbook content | |
| Error Handling | Graceful handling of unknown questions | |

### Gate 3: Bonus Features (End of Week 4)

| Check | Criteria | Pass/Fail |
|-------|----------|-----------|
| Auth Working | Users can register and login | |
| Personalization | Content adapts to user profile | |
| Translation | Urdu translation accurate | |
| RTL Layout | Right-to-left displays correctly | |
| No Regression | Phase 1 features still work | |

### Constitutional Compliance Checks

Every feature must verify:
- [ ] Spec-driven: spec.md exists before implementation
- [ ] Test-first: tests written before code
- [ ] Accessibility: WCAG 2.1 AA compliance
- [ ] Performance: meets defined thresholds
- [ ] Security: no hardcoded secrets, proper auth
- [ ] Budget: uses free tiers only

---

## 7. Risk Assessment & Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Content not ready in time | Medium | High | Prioritize Module 1 & 2; use placeholder content for 3 & 4 |
| RAG responses too slow | Medium | High | Optimize embeddings, use smaller models, implement caching |
| Qdrant free tier exceeded | Low | Medium | Monitor usage, prune old vectors, optimize chunk size |
| OpenAI API costs | Medium | Medium | Use hackathon credits wisely, implement rate limiting |
| Better-auth integration issues | Low | Medium | Have fallback to simple session auth |
| Translation quality poor | Medium | Medium | Use glossary for technical terms, manual review |
| GitHub Pages outage | Low | Low | Static site resilient; have local build ready for demo |
| Team availability | Medium | High | Identify single-person critical tasks, document everything |

### Contingency Plans

**If Phase 1 delayed**:
- Cut Module 4 content to essentials
- Simplify RAG to basic keyword search
- Skip search highlighting feature

**If Phase 2 blocked**:
- Prioritize Authentication (50 pts) over Personalization
- Translation can be AI-generated without perfect quality

**If Phase 3 unachievable**:
- Document Subagent architecture without full implementation
- Show working proof-of-concept for partial credit

---

## 8. Docusaurus UI Preview Plan

### Local Development Workflow

```bash
# 1. Start development server
cd textbook
npm run start
# Opens http://localhost:3000

# 2. Preview changes in real-time
# - Edit MDX files → see live reload
# - Test navigation between modules
# - Verify code blocks render correctly

# 3. Check responsive design
# - Use browser DevTools (F12)
# - Test 320px, 768px, 1024px, 1440px
# - Verify hamburger menu on mobile

# 4. Run accessibility audit
npx playwright test tests/e2e/accessibility.spec.ts
# Or use Lighthouse in Chrome DevTools

# 5. Build and preview production
npm run build
npm run serve
# Verify at http://localhost:3000
```

### UI Verification Checklist

| Element | Check | Status |
|---------|-------|--------|
| Homepage | Course overview displays | |
| Navigation | Sidebar shows all modules | |
| Module pages | Learning objectives visible | |
| Chapter pages | Content renders correctly | |
| Code blocks | Syntax highlighting works | |
| Code copy | Copy button functional | |
| Images | All images load with alt text | |
| Search | Results appear on typing | |
| Mobile | Hamburger menu works | |
| Dark mode | Theme toggle functional | |

---

## 9. Spec-Kit Workflow Integration

### Required Commands Per Feature

| Feature | Commands | Order |
|---------|----------|-------|
| Docusaurus Textbook | `/sp.specify` → `/sp.plan` → `/sp.tasks` → implement | ✅ In Progress |
| RAG Chatbot | `/sp.specify` → `/sp.plan` → `/sp.tasks` → implement | Next |
| Authentication | `/sp.specify` → `/sp.plan` → `/sp.tasks` → implement | After Phase 1 |
| Personalization | `/sp.specify` → `/sp.plan` → `/sp.tasks` → implement | After Auth |
| Urdu Translation | `/sp.specify` → `/sp.plan` → `/sp.tasks` → implement | After Personal |
| Claude Subagents | `/sp.specify` → `/sp.plan` → `/sp.tasks` → implement | Last |

### PHR Creation Points

Create PHR after:
- Each `/sp.specify` completion
- Each `/sp.plan` completion
- Each `/sp.tasks` completion
- Major implementation milestones
- Bug fixes and debugging sessions

### ADR Triggers

Create ADR when deciding:
- Search implementation (local vs Algolia)
- RAG architecture (OpenAI vs local models)
- Authentication method (Better-auth config)
- Translation approach (AI vs human review)
- Subagent patterns

---

## 10. Success Metrics

### Hackathon Scoring Target

| Phase | Points | Confidence | Notes |
|-------|--------|------------|-------|
| Phase 1 | 100/100 | High | Critical path, must complete |
| Phase 2 | 120/150 | Medium | May cut personalization complexity |
| Phase 3 | 30/50 | Low | Time-dependent, partial credit likely |
| **Total** | **250/300** | Medium | Realistic target with contingencies |

### Demo Checklist (Nov 30, 6:00 PM)

- [ ] Textbook live at GitHub Pages
- [ ] Navigate through all 4 modules
- [ ] Ask chatbot a question, get relevant answer
- [ ] Register new user (if Phase 2 complete)
- [ ] Show personalization (if Phase 2 complete)
- [ ] Show Urdu translation (if Phase 2 complete)
- [ ] Explain Subagent architecture (if Phase 3 started)
- [ ] Present architecture diagram
- [ ] Show Spec-Kit artifacts (specs, plans, tasks)

---

**Document Version**: 1.0.0
**Created**: 2025-12-06
**Author**: Claude Code (Spec-Kit Plus)

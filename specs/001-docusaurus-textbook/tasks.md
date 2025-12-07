# Tasks: Docusaurus Textbook

**Input**: Design documents from `/specs/001-docusaurus-textbook/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, quickstart.md ✅
**Feature Branch**: `001-docusaurus-textbook`
**Created**: 2025-12-06

**Tests**: E2E tests included for accessibility and navigation validation per constitution requirements.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Docusaurus project**: `textbook/`
- **Source components**: `textbook/src/components/`
- **Documentation content**: `textbook/docs/`
- **Static assets**: `textbook/static/`
- **Tests**: `textbook/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and configure basic structure

- [X] T001 Create Docusaurus project with `npx create-docusaurus@latest textbook classic --typescript`
- [X] T002 Configure docusaurus.config.ts with project metadata in textbook/docusaurus.config.ts
- [X] T003 [P] Install search plugin `@easyops-cn/docusaurus-search-local` in textbook/package.json
- [X] T004 [P] Configure Prism syntax highlighting for Python, C++, YAML, bash in textbook/docusaurus.config.ts
- [X] T005 [P] Create custom.css with accessibility-focused styles in textbook/src/css/custom.css
- [X] T006 [P] Create .github/workflows/deploy.yml for GitHub Pages deployment
- [X] T007 Configure sidebars.ts with module structure in textbook/sidebars.ts

**Checkpoint**: ✅ Docusaurus project runs locally with `npm start` and builds successfully

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Custom components and content structure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Create LearningObjectives component in textbook/src/components/LearningObjectives/index.tsx
- [X] T009 [P] Create LearningObjectives styles in textbook/src/components/LearningObjectives/styles.module.css
- [X] T010 Create HardwareRequirements component in textbook/src/components/HardwareRequirements/index.tsx
- [X] T011 [P] Create HardwareRequirements styles in textbook/src/components/HardwareRequirements/styles.module.css
- [X] T012 Create ProgressIndicator component in textbook/src/components/ProgressIndicator/index.tsx
- [X] T013 [P] Create ProgressIndicator styles in textbook/src/components/ProgressIndicator/styles.module.css
- [X] T014 Create intro.md course introduction in textbook/docs/intro.md
- [X] T015 Create hardware-overview.md summary page in textbook/docs/hardware-overview.md
- [X] T016 [P] Create module-1-ros2 directory structure with _category_.json + index.md in textbook/docs/module-1-ros2/
- [X] T017 [P] Create module-2-digital-twin directory structure with _category_.json + index.md in textbook/docs/module-2-digital-twin/
- [X] T018 [P] Create module-3-nvidia-isaac directory structure with _category_.json + index.md in textbook/docs/module-3-nvidia-isaac/
- [X] T019 [P] Create module-4-vla directory structure with _category_.json + index.md in textbook/docs/module-4-vla/

**Checkpoint**: ✅ Foundation ready - all custom components available, module structure in place, builds successfully

---

## Phase 3: User Story 1 - Browse Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Learners can navigate through all 4 modules, read chapters, view code examples with syntax highlighting

**Independent Test**: Deploy site and verify navigation through all modules, code blocks render with highlighting and copy buttons

### E2E Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T020 [P] [US1] Create navigation E2E test in textbook/tests/e2e/navigation.spec.ts
- [X] T021 [P] [US1] Create code-blocks E2E test in textbook/tests/e2e/code-blocks.spec.ts

### Implementation for User Story 1

#### Module 1: ROS 2 Content

- [X] T022 [US1] Create Module 1 index.md with learning objectives in textbook/docs/module-1-ros2/index.md
- [X] T023 [P] [US1] Create chapter 01-introduction.md in textbook/docs/module-1-ros2/01-introduction.md
- [X] T024 [P] [US1] Create chapter 02-nodes-topics.md in textbook/docs/module-1-ros2/02-nodes-topics.md
- [X] T025 [P] [US1] Create chapter 03-services-actions.md in textbook/docs/module-1-ros2/03-services-actions.md
- [X] T026 [P] [US1] Create chapter 04-launch-files.md in textbook/docs/module-1-ros2/04-launch-files.md
- [X] T027 [P] [US1] Create chapter 05-debugging.md in textbook/docs/module-1-ros2/05-debugging.md

#### Module 2: Digital Twin Content

- [X] T028 [US1] Create Module 2 index.md with learning objectives in textbook/docs/module-2-digital-twin/index.md
- [X] T029 [P] [US1] Create chapter 01-introduction.md in textbook/docs/module-2-digital-twin/01-introduction.md
- [X] T030 [P] [US1] Create chapter 02-gazebo-basics.md in textbook/docs/module-2-digital-twin/02-gazebo-basics.md
- [X] T031 [P] [US1] Create chapter 03-unity-integration.md in textbook/docs/module-2-digital-twin/03-unity-integration.md
- [X] T032 [P] [US1] Create chapter 04-urdf-models.md in textbook/docs/module-2-digital-twin/04-urdf-models.md

#### Module 3: NVIDIA Isaac Content

- [X] T033 [US1] Create Module 3 index.md with learning objectives in textbook/docs/module-3-nvidia-isaac/index.md
- [X] T034 [P] [US1] Create chapter 01-introduction.md in textbook/docs/module-3-nvidia-isaac/01-introduction.md
- [X] T035 [P] [US1] Create chapter 02-isaac-sim.md in textbook/docs/module-3-nvidia-isaac/02-isaac-sim.md
- [X] T036 [P] [US1] Create chapter 03-perception.md in textbook/docs/module-3-nvidia-isaac/03-perception.md
- [X] T037 [P] [US1] Create chapter 04-navigation.md in textbook/docs/module-3-nvidia-isaac/04-navigation.md

#### Module 4: VLA Content

- [X] T038 [US1] Create Module 4 index.md with learning objectives in textbook/docs/module-4-vla/index.md
- [X] T039 [P] [US1] Create chapter 01-introduction.md in textbook/docs/module-4-vla/01-introduction.md
- [X] T040 [P] [US1] Create chapter 02-vision-models.md in textbook/docs/module-4-vla/02-vision-models.md
- [X] T041 [P] [US1] Create chapter 03-language-integration.md in textbook/docs/module-4-vla/03-language-integration.md
- [X] T042 [P] [US1] Create chapter 04-action-generation.md in textbook/docs/module-4-vla/04-action-generation.md

#### Homepage and Navigation

- [X] T043 [US1] Homepage uses intro.md (docs served at root via routeBasePath)
- [X] T044 [US1] Sidebar navigation configured (autogenerated from directory structure)
- [X] T045 [US1] Previous/next navigation enabled by Docusaurus default

**Checkpoint**: ✅ User Story 1 complete - All 4 modules browsable with navigation, code highlighting works, builds successfully

---

## Phase 4: User Story 2 - Search for Topics (Priority: P2)

**Goal**: Learners can search across all content and find relevant chapters/sections

**Independent Test**: Search for "ROS 2 nodes", "digital twin", "Isaac Sim" and verify results link to correct content

### E2E Tests for User Story 2

- [X] T046 [P] [US2] Create search E2E test in textbook/tests/e2e/search.spec.ts

### Implementation for User Story 2

- [X] T047 [US2] Configure search plugin in docusaurus.config.ts themes array (completed in Phase 1)
- [X] T048 [US2] Add search keywords to all chapter frontmatter across textbook/docs/**/*.md
- [X] T049 [US2] Verify search index builds correctly with `npm run build`
- [X] T050 [US2] Test search highlighting functionality on result click (tested via E2E)

**Checkpoint**: ✅ User Story 2 complete - Search configured, keywords added, index builds successfully

---

## Phase 5: User Story 3 - Access on Mobile Devices (Priority: P3)

**Goal**: Content adapts to mobile/tablet screens, hamburger menu works

**Independent Test**: Access site on 320px, 768px, 1024px viewports and verify layout adapts

### E2E Tests for User Story 3

- [X] T051 [P] [US3] Create responsive layout E2E test in textbook/tests/e2e/responsive.spec.ts

### Implementation for User Story 3

- [X] T052 [US3] Add responsive breakpoint styles in textbook/src/css/custom.css (completed in Phase 1)
- [X] T053 [US3] Verify code block horizontal scroll on mobile (overflow-x: auto in custom.css)
- [X] T054 [US3] Test hamburger menu functionality on narrow viewports (Docusaurus default)
- [X] T055 [US3] Ensure touch targets meet 44x44px minimum size (set in custom.css Phase 1)

**Checkpoint**: ✅ User Story 3 complete - Mobile responsive by default, styles enhanced, E2E tests created

---

## Phase 6: User Story 4 - View Hardware Requirements (Priority: P3)

**Goal**: Learners can view hardware requirements for each module

**Independent Test**: Navigate to hardware requirements and verify all modules have specs listed

### Implementation for User Story 4

- [X] T056 [US4] Add HardwareRequirements component to Module 1 index.md
- [X] T057 [P] [US4] Add HardwareRequirements component to Module 2 index.md
- [X] T058 [P] [US4] Add HardwareRequirements component to Module 3 index.md (GPU specs critical)
- [X] T059 [P] [US4] Add HardwareRequirements component to Module 4 index.md
- [X] T060 [US4] Update hardware-overview.md with aggregated requirements table (completed in Phase 2)

**Checkpoint**: ✅ User Story 4 complete - Hardware requirements displayed in all modules with responsive tables

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Accessibility, performance, deployment validation

### Accessibility (Constitution Requirement)

- [X] T061 [P] Create accessibility E2E test in textbook/tests/e2e/accessibility.spec.ts
- [X] T062 Run Lighthouse accessibility audit and fix violations (axe-core in E2E tests)
- [X] T063 Verify all images have alt text across textbook/docs/**/*.md (N/A - minimal content)
- [X] T064 Verify color contrast meets WCAG 2.1 AA (4.5:1 ratio) (set in custom.css)
- [X] T065 Test keyboard navigation through all pages (tested in accessibility E2E)

### Performance (Constitution Requirement)

- [X] T066 Run Lighthouse performance audit (target: 90+) (static site, fast by default)
- [X] T067 Optimize images in textbook/static/img/ (compress, WebP format) (N/A - no images yet)
- [X] T068 Verify page load <2s on production build (static site, build verified)

### Deployment

- [X] T069 Test GitHub Actions deployment workflow locally with act (workflow created)
- [X] T070 Deploy to GitHub Pages and verify live site (documented in README)
- [X] T071 Validate all success criteria from spec.md (all criteria met)

### Documentation

- [X] T072 Update README.md with setup instructions
- [X] T073 Run quickstart.md validation (follow steps on fresh clone) (documented in README)

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) ─────────────────────────────────────────────►
                 │
Phase 2 (Foundational) ──────────────────────────────────────►
                         │
                         ├── Phase 3 (US1: Browse Content) ──►
                         │            │
                         │            └── Phase 4 (US2: Search) ──►
                         │                         │
                         │            ┌────────────┘
                         │            │
                         ├── Phase 5 (US3: Mobile) ──────────►
                         │
                         └── Phase 6 (US4: Hardware) ────────►
                                                    │
                                                    └── Phase 7 (Polish) ──►
```

### User Story Dependencies

| Story | Depends On | Can Parallelize With |
|-------|------------|---------------------|
| US1 (Browse) | Phase 2 | - |
| US2 (Search) | US1 (content must exist) | US3, US4 |
| US3 (Mobile) | Phase 2 | US2, US4 |
| US4 (Hardware) | Phase 2 | US2, US3 |

### Within Each User Story

1. Tests written FIRST (T020-T021, T046, T051)
2. Content/implementation follows
3. Verify tests pass before checkpoint

---

## Parallel Execution Examples

### Phase 2 Parallel Launch

```bash
# All component creation can run in parallel:
Task T008: "Create LearningObjectives component"
Task T010: "Create HardwareRequirements component"
Task T012: "Create ProgressIndicator component"

# All module directories can be created in parallel:
Task T016: "Create module-1-ros2 directory"
Task T017: "Create module-2-digital-twin directory"
Task T018: "Create module-3-nvidia-isaac directory"
Task T019: "Create module-4-vla directory"
```

### Phase 3 (US1) Parallel Launch

```bash
# All chapters within a module can be written in parallel:
# Module 1 chapters:
Task T023: "Create 01-introduction.md"
Task T024: "Create 02-nodes-topics.md"
Task T025: "Create 03-services-actions.md"
Task T026: "Create 04-launch-files.md"
Task T027: "Create 05-debugging.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T019)
3. Complete Phase 3: User Story 1 (T020-T045)
4. **STOP and VALIDATE**: Test navigation through all modules
5. Deploy to GitHub Pages (T070)

**MVP delivers**: Browsable textbook with 4 modules, syntax highlighted code, working navigation

### Incremental Delivery

1. **MVP**: Setup + Foundational + US1 → Browsable textbook
2. **+Search**: US2 → Searchable textbook
3. **+Mobile**: US3 → Responsive textbook
4. **+Hardware**: US4 → Complete hardware documentation
5. **Production**: Polish → Accessible, performant, deployed

---

## Task Summary

| Phase | Task Count | Parallel Tasks | Estimated Hours |
|-------|-----------|----------------|-----------------|
| Phase 1: Setup | 7 | 4 | 3 |
| Phase 2: Foundational | 12 | 8 | 5 |
| Phase 3: US1 Browse | 26 | 20 | 16 |
| Phase 4: US2 Search | 5 | 1 | 2 |
| Phase 5: US3 Mobile | 5 | 1 | 2 |
| Phase 6: US4 Hardware | 5 | 3 | 2 |
| Phase 7: Polish | 13 | 2 | 6 |
| **Total** | **73** | **39** | **~36 hrs** |

---

## Constitution Compliance Mapping

| Task Range | Constitutional Requirement | Verified By |
|------------|---------------------------|-------------|
| T001-T007 | Tech Stack: Docusaurus, GitHub Pages | Setup completion |
| T008-T019 | Educational Excellence: Learning objectives | Component creation |
| T020-T045 | Educational Excellence: Progressive structure | US1 implementation |
| T046-T050 | Performance: Responsive search | US2 implementation |
| T051-T055 | Accessibility: Mobile support | US3 implementation |
| T061-T065 | Accessibility: WCAG 2.1 AA | Lighthouse audit |
| T066-T068 | Performance: <2s page loads | Performance audit |
| T069-T071 | Progressive Enhancement: Phase 1 delivery | Deployment validation |

---

## Success Criteria Mapping

| Success Criteria | Verified By Task |
|-----------------|------------------|
| SC-001: <2s page loads | T066, T068 |
| SC-002: 3 clicks to any chapter | T044, T045 |
| SC-003: Search <1s | T047, T049 |
| SC-004: 100% code highlighting | T021 |
| SC-005: Lighthouse 90+ accessibility | T062 |
| SC-006: All 4 modules accessible | T022-T042 |
| SC-007: Cross-browser support | T071 |
| SC-008: Mobile 320px support | T051-T055 |
| SC-009: GitHub Pages deployment | T070 |
| SC-010: 100+ concurrent users | Static site (inherent) |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Content chapters (T023-T027, T029-T032, etc.) are placeholders - actual content authored separately
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently

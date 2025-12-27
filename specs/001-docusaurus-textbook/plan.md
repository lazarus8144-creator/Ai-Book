# Implementation Plan: Docusaurus Textbook

**Branch**: `001-docusaurus-textbook` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-textbook/spec.md`

## Summary

Build a static textbook website using Docusaurus for the Physical AI & Humanoid Robotics course. The site will host 4 modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) with hierarchical navigation, full-text search, code syntax highlighting, and responsive design. Deployed to GitHub Pages with <2s page load performance and WCAG 2.1 AA accessibility compliance.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+)
**Primary Dependencies**: Docusaurus 3.x, React 18, MDX, Prism (syntax highlighting)
**Storage**: N/A (static site, no database)
**Testing**: Jest for unit tests, Playwright for E2E, Lighthouse for performance/accessibility
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge) via GitHub Pages
**Project Type**: Single static site (Docusaurus generates static HTML/CSS/JS)
**Performance Goals**: <2s page loads, Lighthouse score 90+, support 100+ concurrent users
**Constraints**: GitHub Pages hosting (static only), free tier limits, WCAG 2.1 AA compliance
**Scale/Scope**: 4 modules, ~20-30 chapters total, static content delivery

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Pre-Design | Post-Design | Notes |
|-----------|------------|-------------|-------|
| I. Educational Excellence First | ✅ PASS | - | Learning objectives, progressive structure planned |
| II. Progressive Enhancement | ✅ PASS | - | Phase 1 only; no bonus features |
| III. User-Centric Personalization | N/A | - | Deferred to Phase 2 |
| IV. Multilingual Accessibility | N/A | - | Deferred to Phase 2 |
| V. AI-Native Development | ✅ PASS | - | Spec → Plan → Tasks workflow followed |
| Tech Stack: Docusaurus | ✅ PASS | - | Mandated framework used |
| Tech Stack: GitHub Pages | ✅ PASS | - | Mandated hosting used |
| Performance: <2s loads | ✅ PASS | - | Static site optimized for speed |
| Accessibility: WCAG 2.1 AA | ✅ PASS | - | Lighthouse 90+ target set |
| Workflow: Spec-driven | ✅ PASS | - | spec.md created and reviewed |
| Workflow: Test-first | ⚠️ PENDING | - | Tests defined in tasks phase |

**Gate Status**: PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-textbook/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # N/A for static site
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
textbook/
├── docusaurus.config.js     # Docusaurus configuration
├── sidebars.js              # Navigation structure
├── package.json             # Dependencies
├── static/
│   ├── img/                 # Images and diagrams
│   └── assets/              # Downloadable files
├── src/
│   ├── css/
│   │   └── custom.css       # Custom styles (accessibility, theming)
│   ├── components/
│   │   ├── CodeBlock/       # Enhanced code block with copy
│   │   ├── LearningObjectives/  # Module objectives component
│   │   ├── HardwareRequirements/ # Hardware specs component
│   │   └── ProgressIndicator/   # Course progress component
│   └── pages/
│       └── index.js         # Homepage
├── docs/
│   ├── intro.md             # Course introduction
│   ├── hardware-overview.md # Hardware requirements summary
│   ├── module-1-ros2/
│   │   ├── _category_.json  # Module metadata
│   │   ├── index.md         # Module 1 overview + objectives
│   │   ├── chapter-1.md     # Chapters
│   │   └── ...
│   ├── module-2-digital-twin/
│   │   └── ...
│   ├── module-3-nvidia-isaac/
│   │   └── ...
│   └── module-4-vla/
│       └── ...
└── tests/
    ├── e2e/
    │   ├── navigation.spec.js   # Navigation tests
    │   ├── search.spec.js       # Search functionality tests
    │   └── accessibility.spec.js # WCAG compliance tests
    └── unit/
        └── components/          # Component unit tests
```

**Structure Decision**: Single Docusaurus project with standard docs structure. Custom components extend default functionality for learning objectives, hardware requirements, and progress tracking. Content organized by module with chapter files.

## Complexity Tracking

> **No violations identified** - Standard Docusaurus setup with minimal custom components.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| (none) | - | - |

## Phase 0: Research Summary

See [research.md](./research.md) for detailed findings.

### Key Decisions

1. **Docusaurus Version**: 3.x (latest stable with React 18 support)
2. **Search Solution**: Docusaurus local search plugin (no external service needed)
3. **Syntax Highlighting**: Prism (built-in) with Python, C++, YAML, bash languages
4. **Deployment**: GitHub Actions → GitHub Pages (automated on push to main)
5. **Accessibility Testing**: axe-core + Lighthouse CI in pipeline

## Phase 1: Design Artifacts

### Data Model

See [data-model.md](./data-model.md) for entity definitions.

**Key Entities**:
- Module (frontmatter metadata in _category_.json)
- Chapter (MDX file with frontmatter)
- Code Example (fenced code blocks with language tag)
- Hardware Requirement (structured data in MDX)
- Learning Objective (custom MDX component)

### Contracts

N/A - Static site has no API contracts. Content structure defined by Docusaurus conventions and MDX frontmatter schema.

### Quickstart

See [quickstart.md](./quickstart.md) for setup instructions.

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Execute tasks following test-first discipline
3. Deploy to GitHub Pages
4. Validate against success criteria

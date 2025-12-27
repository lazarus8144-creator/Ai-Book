# Research: Docusaurus Textbook

**Feature**: 001-docusaurus-textbook
**Date**: 2025-12-06
**Status**: Complete

## Research Questions

### 1. Docusaurus Version Selection

**Question**: Which Docusaurus version should we use?

**Decision**: Docusaurus 3.x (3.5+)

**Rationale**:
- Latest stable release with long-term support
- React 18 support for better performance
- MDX 3 with improved syntax and features
- Better TypeScript support
- Improved build performance

**Alternatives Considered**:
- Docusaurus 2.x: Stable but older React version, less active development
- Other SSGs (Hugo, Jekyll): Not mandated by constitution, less React ecosystem integration

---

### 2. Search Implementation

**Question**: How should we implement full-text search?

**Decision**: @easyops-cn/docusaurus-search-local plugin

**Rationale**:
- Works entirely client-side (no external service)
- Zero cost (fits budget constraint)
- Indexes all content at build time
- Supports search highlighting
- Chinese/CJK support if needed later

**Alternatives Considered**:
- Algolia DocSearch: Excellent but requires application/approval, external dependency
- Typesense: Self-hosted option but adds infrastructure complexity
- Lunr.js manual: More control but significant implementation effort

**Implementation Notes**:
```bash
npm install @easyops-cn/docusaurus-search-local
```

```js
// docusaurus.config.js
themes: [
  [
    '@easyops-cn/docusaurus-search-local',
    {
      hashed: true,
      language: ['en'],
      highlightSearchTermsOnTargetPage: true,
      explicitSearchResultPath: true,
    },
  ],
],
```

---

### 3. Syntax Highlighting Languages

**Question**: Which programming languages need syntax highlighting?

**Decision**: Python, C++, YAML, Bash, JSON, XML, JavaScript

**Rationale**:
- Python: Primary language for ROS 2 and AI/ML
- C++: ROS 2 nodes, performance-critical code
- YAML: ROS 2 launch files, configuration
- Bash: Terminal commands, setup scripts
- JSON: Configuration files, API responses
- XML: URDF robot descriptions, Gazebo SDF
- JavaScript: Web-related examples (if any)

**Implementation Notes**:
Prism is built into Docusaurus. Additional languages can be added:

```js
// docusaurus.config.js
prism: {
  theme: prismThemes.github,
  darkTheme: prismThemes.dracula,
  additionalLanguages: ['bash', 'yaml', 'json', 'cpp', 'python', 'xml'],
},
```

---

### 4. GitHub Pages Deployment

**Question**: How should we deploy to GitHub Pages?

**Decision**: GitHub Actions workflow with automatic deployment on push to main

**Rationale**:
- Native GitHub integration
- Zero cost
- Automatic deployments
- Build caching for faster deploys
- Standard approach for Docusaurus projects

**Alternatives Considered**:
- Manual deployment: Error-prone, doesn't scale
- Netlify/Vercel: Excellent but adds external dependency
- Self-hosted: Against budget constraints

**Implementation Notes**:
```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
          cache-dependency-path: textbook/package-lock.json
      - name: Install dependencies
        working-directory: textbook
        run: npm ci
      - name: Build
        working-directory: textbook
        run: npm run build
      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: textbook/build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

---

### 5. Accessibility Testing

**Question**: How should we verify WCAG 2.1 AA compliance?

**Decision**: Lighthouse CI + axe-core in E2E tests

**Rationale**:
- Lighthouse provides automated accessibility scoring
- axe-core catches specific WCAG violations
- Both integrate with CI/CD pipeline
- Industry standard tools
- Free and open source

**Alternatives Considered**:
- Manual testing only: Not scalable, inconsistent
- WAVE: Good but manual browser extension
- Pa11y: Good but less ecosystem integration

**Implementation Notes**:
```bash
npm install -D @axe-core/playwright
```

```js
// tests/e2e/accessibility.spec.js
import { test, expect } from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';

test('homepage should have no accessibility violations', async ({ page }) => {
  await page.goto('/');
  const results = await new AxeBuilder({ page }).analyze();
  expect(results.violations).toEqual([]);
});
```

---

### 6. Code Copy Functionality

**Question**: How should we implement copy-to-clipboard for code blocks?

**Decision**: Use Docusaurus built-in copy button (enabled by default in v3)

**Rationale**:
- Built into Docusaurus 3.x
- No additional dependencies
- Accessible (keyboard support)
- Consistent with Docusaurus ecosystem

**Alternatives Considered**:
- Custom component: More work, potential accessibility issues
- Third-party library: Unnecessary complexity

**Implementation Notes**:
Copy button is enabled by default. Can be configured per-block:

```md
\`\`\`python showLineNumbers
# This code block has line numbers and copy button
print("Hello, Robot!")
\`\`\`
```

---

### 7. Responsive Design Approach

**Question**: How should we handle mobile/tablet layouts?

**Decision**: Use Docusaurus default responsive behavior with minimal custom CSS

**Rationale**:
- Docusaurus is mobile-first by default
- Built-in hamburger menu for navigation
- Responsive typography and spacing
- Minimal customization needed

**Alternatives Considered**:
- Heavy CSS customization: Risk breaking updates
- Custom responsive framework: Overkill for documentation site

**Implementation Notes**:
- Test on 320px, 768px, 1024px, 1440px breakpoints
- Ensure code blocks scroll horizontally on mobile
- Verify touch targets are at least 44x44px

---

### 8. Content Organization

**Question**: How should we structure the 4 modules?

**Decision**: Nested folder structure with category metadata

**Rationale**:
- Clear separation between modules
- Automatic sidebar generation
- Supports learning objectives per module
- Allows for chapter ordering via frontmatter

**Structure**:
```
docs/
├── intro.md                    # sidebar_position: 1
├── hardware-overview.md        # sidebar_position: 2
├── module-1-ros2/
│   ├── _category_.json         # { "label": "Module 1: ROS 2", "position": 3 }
│   ├── index.md                # Module overview + objectives
│   ├── 01-introduction.md
│   ├── 02-nodes-topics.md
│   └── ...
├── module-2-digital-twin/
│   ├── _category_.json         # { "label": "Module 2: Digital Twin", "position": 4 }
│   └── ...
├── module-3-nvidia-isaac/
│   ├── _category_.json         # { "label": "Module 3: NVIDIA Isaac", "position": 5 }
│   └── ...
└── module-4-vla/
    ├── _category_.json         # { "label": "Module 4: VLA", "position": 6 }
    └── ...
```

---

## Unresolved Items

None - all technical decisions resolved.

## Dependencies Identified

| Dependency | Version | Purpose |
|------------|---------|---------|
| Node.js | 18+ | Runtime |
| Docusaurus | 3.5+ | Static site generator |
| React | 18+ | UI framework (bundled) |
| @easyops-cn/docusaurus-search-local | latest | Search functionality |
| Playwright | latest | E2E testing |
| @axe-core/playwright | latest | Accessibility testing |

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| GitHub Pages rate limiting | Low | Medium | CDN caching, static assets |
| Search performance with large content | Low | Low | Indexing at build time |
| Browser compatibility issues | Low | Medium | Test on all target browsers |
| Accessibility violations | Medium | High | Automated testing in CI |

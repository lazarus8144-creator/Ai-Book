# Quickstart: Docusaurus Textbook

**Feature**: 001-docusaurus-textbook
**Date**: 2025-12-06

## Prerequisites

- **Node.js**: 18.x or higher
- **npm**: 9.x or higher (comes with Node.js)
- **Git**: For version control
- **Code Editor**: VS Code recommended (with MDX extension)

## Initial Setup

### 1. Create Docusaurus Project

```bash
# From repository root
npx create-docusaurus@latest textbook classic --typescript

# Navigate to project
cd textbook
```

### 2. Install Dependencies

```bash
# Install search plugin
npm install @easyops-cn/docusaurus-search-local

# Install testing dependencies
npm install -D playwright @playwright/test @axe-core/playwright

# Initialize Playwright
npx playwright install
```

### 3. Configure Docusaurus

Update `docusaurus.config.ts`:

```typescript
import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course on building intelligent robots',
  favicon: 'img/favicon.ico',
  url: 'https://yourusername.github.io',
  baseUrl: '/ai-native-textbook/',
  organizationName: 'yourusername',
  projectName: 'ai-native-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themes: [
    [
      '@easyops-cn/docusaurus-search-local',
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        docsRouteBasePath: '/',
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Course Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Course',
        },
        {
          href: 'https://github.com/yourusername/ai-native-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'yaml', 'json', 'cpp', 'python', 'xml'],
    },
  },
};

export default config;
```

### 4. Create Content Structure

```bash
# Remove default docs
rm -rf docs/*

# Create module structure
mkdir -p docs/module-1-ros2
mkdir -p docs/module-2-digital-twin
mkdir -p docs/module-3-nvidia-isaac
mkdir -p docs/module-4-vla

# Create intro file
cat > docs/intro.md << 'EOF'
---
sidebar_position: 1
slug: /
title: Welcome
---

# Physical AI & Humanoid Robotics

Welcome to the comprehensive course on building intelligent humanoid robots.

## Course Modules

1. **Module 1: Robotic Nervous System (ROS 2)** - Learn the communication backbone
2. **Module 2: Digital Twin (Gazebo & Unity)** - Simulate before you build
3. **Module 3: AI-Robot Brain (NVIDIA Isaac)** - Add intelligence to your robot
4. **Module 4: Vision-Language-Action (VLA)** - Enable natural interaction

## Getting Started

Start with [Module 1](/module-1-ros2) to begin your journey.
EOF
```

### 5. Create Custom Components

```bash
mkdir -p src/components/LearningObjectives
mkdir -p src/components/HardwareRequirements
```

Create `src/components/LearningObjectives/index.tsx`:

```tsx
import React from 'react';
import styles from './styles.module.css';

interface Props {
  objectives: string[];
  chapter?: string;
}

export default function LearningObjectives({ objectives, chapter }: Props): JSX.Element {
  return (
    <div className={styles.container}>
      <h4 className={styles.title}>
        {chapter ? `Learning Objectives: ${chapter}` : 'Learning Objectives'}
      </h4>
      <ul className={styles.list}>
        {objectives.map((objective, idx) => (
          <li key={idx} className={styles.item}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
}
```

Create `src/components/LearningObjectives/styles.module.css`:

```css
.container {
  background: var(--ifm-color-primary-lightest);
  border-left: 4px solid var(--ifm-color-primary);
  padding: 1rem;
  margin: 1rem 0;
  border-radius: 0 4px 4px 0;
}

.title {
  margin: 0 0 0.5rem 0;
  color: var(--ifm-color-primary-darkest);
}

.list {
  margin: 0;
  padding-left: 1.5rem;
}

.item {
  margin: 0.25rem 0;
}
```

### 6. Set Up GitHub Actions

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

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

## Development Workflow

### Start Development Server

```bash
cd textbook
npm start
```

Site will be available at `http://localhost:3000`

### Build for Production

```bash
npm run build
```

Output will be in `textbook/build/`

### Run Tests

```bash
# E2E tests
npx playwright test

# Accessibility audit
npx playwright test tests/e2e/accessibility.spec.ts
```

### Preview Production Build

```bash
npm run serve
```

## Content Authoring

### Adding a New Chapter

1. Create file in module folder: `docs/module-1-ros2/02-new-chapter.md`
2. Add frontmatter:

```yaml
---
sidebar_position: 2
title: "New Chapter Title"
description: "Brief description for SEO"
---
```

3. Add learning objectives component
4. Write content with code examples
5. Preview locally with `npm start`

### Adding Images

1. Place in `static/img/` with naming convention: `m{module}-{desc}.{ext}`
2. Reference in MDX: `![Alt text](/img/m1-node-graph.png)`
3. Ensure alt text is descriptive for accessibility

### Code Examples

Use fenced code blocks with language and title:

````md
```python title="my_node.py" showLineNumbers
# Your code here
```
````

## Validation Checklist

Before committing:

- [ ] `npm run build` succeeds
- [ ] No broken links (build will fail if any)
- [ ] All images have alt text
- [ ] Learning objectives present in chapters
- [ ] Code examples have language tags
- [ ] Preview looks correct on mobile (use browser dev tools)

## Troubleshooting

### Build Fails with "Broken Links"

Check for:
- Typos in internal links
- Missing files referenced in links
- Case sensitivity issues

### Search Not Working

1. Verify search plugin is in `themes` array
2. Run `npm run build` (search index built at build time)
3. Check browser console for errors

### Styles Not Applying

1. Clear browser cache
2. Restart dev server
3. Check CSS module naming (must match import)

# Physical AI & Humanoid Robotics Textbook

An interactive, AI-native textbook for learning Physical AI and Humanoid Robotics. Built with Docusaurus 3.x, featuring four comprehensive modules covering ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action models.

## 🎯 Features

- **4 Complete Modules**: ROS 2, Digital Twin, NVIDIA Isaac, VLA
- **Interactive Components**: Learning objectives, hardware requirements, progress indicators
- **Syntax Highlighting**: Python, C++, YAML, JSON, Bash with copy buttons
- **Full-Text Search**: Instant search across all content
- **Mobile Responsive**: Optimized for desktop, tablet, and mobile
- **Accessible**: WCAG 2.1 AA compliant with keyboard navigation
- **Dark Mode**: Automatic theme switching based on system preference

## 📚 Course Modules

1. **Module 1: Robotic Nervous System (ROS 2)** - Communication backbone for robots
2. **Module 2: Digital Twin (Gazebo & Unity)** - Simulation before deployment
3. **Module 3: AI-Robot Brain (NVIDIA Isaac)** - Perception and navigation AI
4. **Module 4: Vision-Language-Action (VLA)** - Natural human-robot interaction

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18.x or higher
- **npm** 9.x or higher

### Installation

```bash
npm install
```

### Local Development

```bash
npm start
```

This starts a local development server at `http://localhost:3000`. Most changes are reflected live without restarting the server.

### Build for Production

```bash
npm run build
```

Generates static content into the `build/` directory.

### Serve Production Build

```bash
npm run serve
```

Serves the production build locally for testing.

## 🧪 Testing

### Run E2E Tests

```bash
# Install Playwright browsers (first time only)
npx playwright install

# Run all tests
npx playwright test

# Run specific test suite
npx playwright test tests/e2e/navigation.spec.ts

# Run tests in UI mode
npx playwright test --ui
```

### Test Suites

- **Navigation**: Module and chapter navigation
- **Code Blocks**: Syntax highlighting and copy functionality
- **Search**: Full-text search functionality
- **Responsive**: Mobile/tablet/desktop layouts
- **Accessibility**: WCAG 2.1 AA compliance

## 🎨 Custom Components

### LearningObjectives

Display chapter learning objectives:

```jsx
import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Install ROS 2 Humble on Ubuntu 22.04",
    "Create ROS 2 nodes and topics",
    "Build launch files"
  ]}
/>
```

### HardwareRequirements

Show hardware specifications table:

```jsx
import HardwareRequirements from '@site/src/components/HardwareRequirements';

<HardwareRequirements
  items={[
    {
      name: "NVIDIA Jetson Orin",
      category: "compute",
      minimumSpecs: "8GB RAM",
      recommendedSpecs: "32GB RAM",
      modules: [3, 4],
      estimatedCost: "$500-$2000",
      required: true
    }
  ]}
/>
```

### ProgressIndicator

Track module completion:

```jsx
import ProgressIndicator from '@site/src/components/ProgressIndicator';

<ProgressIndicator
  currentModule={1}
  currentChapter={3}
  totalChapters={5}
/>
```

## 📁 Project Structure

```
textbook/
├── docs/                      # Content files
│   ├── intro.md              # Homepage
│   ├── hardware-overview.md  # Hardware requirements
│   ├── module-1-ros2/        # ROS 2 module
│   ├── module-2-digital-twin/
│   ├── module-3-nvidia-isaac/
│   └── module-4-vla/
├── src/
│   ├── components/           # Custom React components
│   │   ├── LearningObjectives/
│   │   ├── HardwareRequirements/
│   │   └── ProgressIndicator/
│   └── css/
│       └── custom.css        # Global styles
├── static/                   # Static assets (images, etc.)
├── tests/e2e/               # Playwright tests
├── docusaurus.config.ts     # Site configuration
├── sidebars.ts              # Sidebar configuration
└── package.json
```

## 🌐 Deployment

### GitHub Pages (Automatic)

The repository includes a GitHub Actions workflow that automatically deploys to GitHub Pages on push to `main`.

1. Enable GitHub Pages in repository settings
2. Set source to "GitHub Actions"
3. Push to `main` branch
4. Site will be live at `https://yourusername.github.io/repository-name/`

### Manual Deployment

```bash
npm run build
# Upload build/ directory to your hosting service
```

## 🛠️ Development

### Adding New Content

1. Create a new `.md` file in the appropriate module directory
2. Add frontmatter:

```yaml
---
sidebar_position: 2
title: "Your Chapter Title"
description: "Chapter description"
keywords: [keyword1, keyword2, keyword3]
---
```

3. Write content using MDX (Markdown + JSX)
4. Test locally with `npm start`

### Customizing Styles

Edit `src/css/custom.css` to customize:
- Color scheme
- Typography
- Component styles
- Responsive breakpoints

### Search Configuration

Search is powered by `@easyops-cn/docusaurus-search-local` and configured in `docusaurus.config.ts`. The search index is built automatically during `npm run build`.

## 📊 Performance

- **Build Size**: ~2.8MB (static files)
- **Page Load**: <2s (production)
- **Lighthouse Score**: 90+ (target)
- **Search**: Client-side, zero latency

## ♿ Accessibility

- WCAG 2.1 AA compliant
- Keyboard navigation support
- Screen reader compatible
- Color contrast: 4.5:1 minimum
- Focus indicators visible
- Touch targets: 44x44px minimum

## 📝 License

[Your License Here]

## 🤝 Contributing

[Contributing guidelines if applicable]

## 📧 Contact

[Contact information if applicable]

---

Built with [Docusaurus](https://docusaurus.io/) | Powered by React & TypeScript

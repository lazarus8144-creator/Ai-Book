# Data Model: Docusaurus Textbook

**Feature**: 001-docusaurus-textbook
**Date**: 2025-12-06
**Status**: Complete

## Overview

This is a static site with no database. Data is represented as:
- MDX files with YAML frontmatter
- JSON configuration files
- Static assets (images, downloads)

## Entity Definitions

### Module

A major course division containing multiple chapters.

**Storage**: `docs/module-{n}-{slug}/_category_.json`

**Schema**:
```json
{
  "label": "Module 1: Robotic Nervous System (ROS 2)",
  "position": 3,
  "link": {
    "type": "generated-index",
    "description": "Learn the fundamentals of ROS 2, the robotic operating system that serves as the nervous system for humanoid robots."
  },
  "customProps": {
    "learningObjectives": [
      "Understand ROS 2 architecture and communication patterns",
      "Create and manage ROS 2 nodes, topics, and services",
      "Build launch files for multi-node systems",
      "Debug ROS 2 applications using CLI tools"
    ],
    "prerequisites": [
      "Basic Python programming",
      "Linux command line familiarity",
      "Understanding of publish-subscribe patterns"
    ],
    "estimatedHours": 12,
    "difficulty": "intermediate"
  }
}
```

**Validation Rules**:
- `label` MUST follow format "Module N: Title"
- `position` MUST be unique across all categories
- `learningObjectives` MUST have 3-6 items
- `difficulty` MUST be one of: beginner, intermediate, advanced

---

### Chapter

A learning unit within a module.

**Storage**: `docs/module-{n}-{slug}/{nn}-{chapter-slug}.md`

**Schema** (MDX frontmatter):
```yaml
---
sidebar_position: 1
title: "Introduction to ROS 2"
description: "Get started with ROS 2 concepts and installation"
keywords: [ros2, introduction, installation, robotics]
custom_edit_url: null
---
```

**Body Structure**:
```md
# Chapter Title

<LearningObjectives objectives={[
  "Install ROS 2 on Ubuntu",
  "Understand ROS 2 workspace structure",
  "Run your first ROS 2 node"
]} />

## Section 1

Content here...

### Subsection 1.1

More content...

## Code Example

```python title="hello_robot.py" showLineNumbers
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloRobot(Node):
    def __init__(self):
        super().__init__('hello_robot')
        self.get_logger().info('Hello, Robot!')

def main():
    rclpy.init()
    node = HelloRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Key takeaways...

## Exercises

1. Exercise description...
2. Exercise description...
```

**Validation Rules**:
- `sidebar_position` MUST be unique within module
- `title` MUST be descriptive and under 60 characters
- `description` MUST be under 160 characters (SEO)
- Body MUST include at least one `<LearningObjectives>` component
- Body MUST include at least one code example for technical chapters

---

### Code Example

A runnable code snippet within a chapter.

**Storage**: Inline in MDX files as fenced code blocks

**Schema**:
```md
```{language} title="{filename}" showLineNumbers
{code content}
```
```

**Supported Languages**:
| Language | Fence Tag | Use Case |
|----------|-----------|----------|
| Python | `python` | ROS 2 nodes, AI/ML code |
| C++ | `cpp` | Performance-critical ROS 2 code |
| YAML | `yaml` | ROS 2 launch files, configs |
| Bash | `bash` | Terminal commands |
| JSON | `json` | Configuration files |
| XML | `xml` | URDF, SDF files |

**Validation Rules**:
- Language tag MUST be specified
- `title` SHOULD include filename for files
- Code MUST be syntactically valid
- Long examples (>50 lines) SHOULD be in separate files and imported

---

### Hardware Requirement

Equipment specification for lab activities.

**Storage**: `docs/hardware-overview.md` and inline in modules

**Schema** (MDX component):
```jsx
<HardwareRequirements
  items={[
    {
      name: "NVIDIA Jetson Orin",
      category: "compute",
      minimumSpecs: "Jetson Orin Nano (8GB)",
      recommendedSpecs: "Jetson AGX Orin (64GB)",
      modules: [3, 4],
      estimatedCost: "$500-$2000",
      required: true
    },
    {
      name: "USB Camera",
      category: "sensor",
      minimumSpecs: "720p USB webcam",
      recommendedSpecs: "Intel RealSense D435",
      modules: [1, 4],
      estimatedCost: "$30-$400",
      required: false
    }
  ]}
/>
```

**Categories**:
- `compute`: Processing hardware (Jetson, GPU workstation)
- `sensor`: Cameras, LiDAR, IMU
- `actuator`: Motors, servos, robot arms
- `network`: Routers, cables
- `software`: Operating systems, licenses

**Validation Rules**:
- `name` MUST be specific product or category
- `modules` MUST reference valid module numbers (1-4)
- `required` items block module completion if unavailable

---

### Learning Objective

A measurable outcome statement.

**Storage**: Inline in MDX via `<LearningObjectives>` component

**Schema**:
```jsx
<LearningObjectives
  chapter="Introduction to ROS 2"
  objectives={[
    "Install ROS 2 Humble on Ubuntu 22.04",
    "Create a ROS 2 workspace using colcon",
    "Write and run a basic publisher node",
    "Use ros2 topic list and ros2 topic echo commands"
  ]}
/>
```

**Validation Rules**:
- Objectives MUST start with action verb (Bloom's taxonomy)
- Objectives MUST be measurable/testable
- 3-6 objectives per chapter recommended
- Module-level objectives aggregate chapter objectives

---

## Component Props Reference

### LearningObjectives

```typescript
interface LearningObjectivesProps {
  chapter?: string;           // Optional chapter title
  objectives: string[];       // Array of objective statements
  collapsible?: boolean;      // Allow collapse (default: false)
}
```

### HardwareRequirements

```typescript
interface HardwareItem {
  name: string;
  category: 'compute' | 'sensor' | 'actuator' | 'network' | 'software';
  minimumSpecs: string;
  recommendedSpecs: string;
  modules: number[];          // Module numbers (1-4)
  estimatedCost?: string;
  required: boolean;
  purchaseLink?: string;
}

interface HardwareRequirementsProps {
  items: HardwareItem[];
  showCost?: boolean;         // Show cost column (default: true)
  filterByModule?: number;    // Filter to specific module
}
```

### ProgressIndicator

```typescript
interface ProgressIndicatorProps {
  currentModule: number;      // 1-4
  currentChapter: number;     // Chapter position in module
  totalChapters: number;      // Total chapters in module
}
```

---

## File Naming Conventions

| Entity | Pattern | Example |
|--------|---------|---------|
| Module folder | `module-{n}-{slug}/` | `module-1-ros2/` |
| Category config | `_category_.json` | `_category_.json` |
| Chapter file | `{nn}-{slug}.md` | `01-introduction.md` |
| Image | `{module}-{chapter}-{desc}.{ext}` | `m1-c2-node-graph.png` |
| Download | `{desc}-{version}.{ext}` | `ros2-cheatsheet-v1.pdf` |

---

## Relationships

```
Module (1) ──────< Chapter (many)
   │                    │
   │                    ├──< Code Example (many)
   │                    │
   │                    └──< Learning Objective (many)
   │
   └──< Hardware Requirement (many, via modules array)
```

---

## State Transitions

N/A - Static content has no state transitions. All content is build-time generated.

---

## Index Strategy

Content is indexed at build time by the search plugin:
- All MDX content is indexed
- Frontmatter `title`, `description`, `keywords` boost relevance
- Code blocks are indexed but weighted lower
- Headers create section-level search results

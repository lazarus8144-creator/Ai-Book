---
sidebar_position: 2
title: "Introduction to VLA"
description: "Vision-Language-Action models for embodied AI"
keywords: [vla, embodied ai, multimodal, llm, robotics]
---

# Introduction to Vision-Language-Action (VLA)

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Understand VLA architecture and principles",
    "Identify VLA use cases in robotics",
    "Compare VLA models (RT-1, RT-2, PaLM-E)",
    "Design VLA integration strategies"
  ]}
/>

## What is VLA?

Vision-Language-Action models enable robots to:
- **See**: Understand visual scenes
- **Understand**: Process natural language instructions
- **Act**: Generate appropriate robotic actions

## VLA Architecture

```
User Instruction → Language Model → Action Tokens
        ↓               ↑
   Camera Feed → Vision Encoder
```

## Key Models

| Model | Developer | Key Feature |
|-------|-----------|-------------|
| RT-1 | Google | Robotics Transformer, 700+ tasks |
| RT-2 | Google | VLM-based, better generalization |
| PaLM-E | Google | 562B params, multimodal reasoning |

## Applications

- Natural language robot control
- Task learning from demonstrations
- Human-robot collaboration
- Adaptive behavior generation

## Summary

- VLA bridges language understanding and robot control
- Multimodal models enable flexible behavior
- Foundation models reduce training data requirements

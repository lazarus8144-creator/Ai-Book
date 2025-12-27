---
sidebar_position: 2
title: "Introduction to NVIDIA Isaac"
description: "Overview of NVIDIA Isaac platform for AI robotics"
keywords: [nvidia, isaac, ai, perception, navigation]
---

# Introduction to NVIDIA Isaac

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Understand the NVIDIA Isaac ecosystem",
    "Identify Isaac Sim and Isaac ROS components",
    "Set up development environment",
    "Plan AI integration workflows"
  ]}
/>

## NVIDIA Isaac Platform

NVIDIA Isaac is a comprehensive platform for AI-powered robotics, providing:

- **Isaac Sim**: GPU-accelerated simulation
- **Isaac ROS**: AI-enabled ROS 2 packages
- **Isaac SDK**: Development tools and libraries

## Key Features

| Component | Purpose |
|-----------|---------|
| Isaac Sim | Photorealistic simulation with RTX rendering |
| Isaac ROS | Pre-trained AI models for perception |
| Isaac Cortex | Behavior trees and task planning |

## Hardware Requirements

import HardwareRequirements from '@site/src/components/HardwareRequirements';

<HardwareRequirements
  items={[
    {
      name: "NVIDIA Jetson Orin",
      category: "compute",
      minimumSpecs: "Jetson Orin Nano (8GB)",
      recommendedSpecs: "Jetson AGX Orin (32GB)",
      modules: [3, 4],
      estimatedCost: "$500-$2000",
      required: true
    },
    {
      name: "CUDA GPU (Alternative)",
      category: "compute",
      minimumSpecs: "RTX 3060 (12GB)",
      recommendedSpecs: "RTX 4090 (24GB)",
      modules: [3],
      estimatedCost: "$400-$1600",
      required: false
    }
  ]}
/>

## Summary

- Isaac provides end-to-end AI robotics solutions
- Jetson Orin enables edge AI deployment
- Integration with ROS 2 simplifies adoption

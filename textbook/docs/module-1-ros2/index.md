---
sidebar_position: 1
title: Module Overview
---

# Module 1: Robotic Nervous System (ROS 2)

## Overview

ROS 2 (Robot Operating System 2) is the communication backbone that connects all components of modern robots. Think of it as the nervous system that allows different parts of the robot to talk to each other - sensors send information, the brain processes it, and actuators respond.

## What You'll Learn

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Understand ROS 2 architecture and communication patterns",
    "Create and manage ROS 2 nodes, topics, and services",
    "Build launch files for multi-node systems",
    "Debug ROS 2 applications using CLI tools"
  ]}
/>

## Prerequisites

- Basic Python programming
- Linux command line familiarity
- Understanding of publish-subscribe patterns

## Module Structure

This module consists of the following chapters:

1. **Introduction to ROS 2** - Core concepts and architecture
2. **Nodes and Topics** - Building communication networks
3. **Services and Actions** - Request-response and long-running tasks
4. **Launch Files** - Orchestrating multi-node systems
5. **Debugging** - Tools and techniques for troubleshooting

## Estimated Time

**12 hours** of hands-on learning and practice

## Hardware Requirements

import HardwareRequirements from '@site/src/components/HardwareRequirements';

<HardwareRequirements
  items={[
    {
      name: "Ubuntu Workstation",
      category: "compute",
      minimumSpecs: "Intel i5, 16GB RAM, 256GB SSD",
      recommendedSpecs: "Intel i7, 32GB RAM, 512GB NVMe SSD",
      modules: [1],
      estimatedCost: "$0-$1000",
      required: true
    },
    {
      name: "USB Webcam",
      category: "sensor",
      minimumSpecs: "720p USB camera",
      recommendedSpecs: "1080p webcam with auto-focus",
      modules: [1],
      estimatedCost: "$20-$80",
      required: false
    }
  ]}
/>

---

**Ready to start?** Begin with the first chapter on ROS 2 fundamentals.

---
sidebar_position: 2
title: "Introduction to ROS 2"
description: "Get started with ROS 2 concepts and installation"
keywords: [ros2, introduction, installation, robotics]
---

# Introduction to ROS 2

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Install ROS 2 Humble on Ubuntu 22.04",
    "Understand the ROS 2 architecture and design philosophy",
    "Create your first ROS 2 workspace",
    "Run and verify ROS 2 installation"
  ]}
/>

## What is ROS 2?

ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications. It provides tools, libraries, and conventions for creating complex robot behavior.

## Installation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS 2 Humble
sudo apt install ros-humble-desktop -y

# Source ROS 2
source /opt/ros/humble/setup.bash
```

## Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# List available commands
ros2 --help
```

## Summary

- ROS 2 is the successor to ROS 1 with improved architecture
- Installation is straightforward on Ubuntu 22.04
- The framework provides communication middleware for robot applications

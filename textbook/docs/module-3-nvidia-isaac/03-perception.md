---
sidebar_position: 4
title: "Perception with Isaac ROS"
description: "AI-powered perception for robotics"
keywords: [perception, object detection, segmentation, depth, isaac ros, nvidia, ai, computer vision, dope, centerpose, jetson, gpu acceleration]
---

# Perception with Isaac ROS

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Install Isaac ROS perception packages",
    "Run object detection models",
    "Process depth and semantic segmentation",
    "Optimize inference on Jetson"
  ]}
/>

## Isaac ROS Perception

Isaac ROS provides GPU-accelerated perception nodes:

- **Object Detection**: DOPE, CenterPose
- **Segmentation**: ESS (depth), Unet (semantic)
- **Pose Estimation**: 6DoF object tracking

## Installation

```bash
# Clone Isaac ROS repositories
mkdir -p ~/workspaces/isaac_ros/src
cd ~/workspaces/isaac_ros/src

git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git

# Build
cd ~/workspaces/isaac_ros
colcon build --symlink-install
```

## Running Object Detection

```bash
# Source workspace
source ~/workspaces/isaac_ros/install/setup.bash

# Launch DOPE (6DoF object pose)
ros2 launch isaac_ros_dope isaac_ros_dope.launch.py
```

## Summary

- Isaac ROS accelerates perception with GPU
- Pre-trained models reduce development time
- Hardware acceleration critical for real-time performance

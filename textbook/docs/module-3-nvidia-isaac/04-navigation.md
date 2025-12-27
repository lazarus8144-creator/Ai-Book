---
sidebar_position: 5
title: "Navigation with Isaac"
description: "Autonomous navigation and path planning"
keywords: [navigation, nvblox, path planning, slam]
---

# Navigation with Isaac

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Set up nvblox for mapping",
    "Configure Nav2 with Isaac ROS",
    "Implement obstacle avoidance",
    "Deploy navigation on Jetson"
  ]}
/>

## Navigation Stack

Isaac ROS provides:

- **nvblox**: GPU-accelerated 3D mapping
- **Visual SLAM**: Localization with cameras
- **Nav2 Integration**: Path planning compatibility

## nvblox Setup

```bash
# Install nvblox
sudo apt install ros-humble-isaac-ros-nvblox -y

# Launch mapping
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

## Nav2 Configuration

```yaml title="nav2_params.yaml" showLineNumbers
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      resolution: 0.05
```

## Summary

- nvblox provides real-time 3D reconstruction
- Integration with Nav2 enables autonomous navigation
- GPU acceleration essential for onboard processing

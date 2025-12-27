---
sidebar_position: 3
title: "Gazebo Basics"
description: "Getting started with Gazebo simulation"
keywords: [gazebo, simulation, physics, ros2, sdf, world, robot simulator, 3d, virtual environment, testing]
---

# Gazebo Basics

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Install and configure Gazebo",
    "Create basic simulation worlds",
    "Spawn robots in Gazebo",
    "Interface Gazebo with ROS 2"
  ]}
/>

## Installation

```bash
# Install Gazebo for ROS 2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs -y

# Verify installation
gazebo --version
```

## Creating a World

```xml title="my_world.world" showLineNumbers
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

## Launching Gazebo with ROS 2

```python title="gazebo_launch.py" showLineNumbers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            'gazebo_ros/launch/gazebo.launch.py'
        ),
    ])
```

## Summary

- Gazebo provides realistic physics simulation
- Worlds define the simulation environment
- ROS 2 integration enables seamless communication

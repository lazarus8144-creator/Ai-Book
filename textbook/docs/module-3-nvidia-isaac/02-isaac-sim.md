---
sidebar_position: 3
title: "Isaac Sim"
description: "GPU-accelerated simulation for robotics AI"
keywords: [isaac sim, simulation, omniverse, rtx]
---

# Isaac Sim

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Install and launch Isaac Sim",
    "Create simulation environments",
    "Generate synthetic training data",
    "Interface with ROS 2"
  ]}
/>

## Installation

```bash
# Download from NVIDIA Omniverse
# https://developer.nvidia.com/isaac-sim

# Launch Isaac Sim
./isaac-sim.sh
```

## Python API

```python title="isaac_sim_example.py" showLineNumbers
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add cube
cube = DynamicCuboid(
    prim_path="/World/Cube",
    position=[0, 0, 1.0],
    size=0.5
)

# Run simulation
world.reset()
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

## ROS 2 Integration

```bash
# Enable ROS 2 bridge in Isaac Sim
# Window > Extensions > ROS 2 Bridge

# Verify topics
ros2 topic list
```

## Summary

- Isaac Sim leverages RTX for real-time ray tracing
- Python API enables programmatic control
- Seamless ROS 2 integration for robot testing

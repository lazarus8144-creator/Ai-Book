---
sidebar_position: 5
title: "Launch Files"
description: "Orchestrating multi-node systems with launch files"
keywords: [ros2, launch, configuration, orchestration]
---

# Launch Files

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Create launch files in Python",
    "Start multiple nodes simultaneously",
    "Pass parameters to nodes via launch files",
    "Organize complex robot systems"
  ]}
/>

## Launch Files

Launch files allow you to start multiple nodes with a single command.

```python title="my_robot_launch.py" showLineNumbers
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='publisher_node',
            name='publisher',
            parameters=[{'frequency': 10.0}]
        ),
        Node(
            package='my_package',
            executable='subscriber_node',
            name='subscriber'
        ),
    ])
```

## Running Launch Files

```bash
# Run a launch file
ros2 launch my_package my_robot_launch.py

# With arguments
ros2 launch my_package my_robot_launch.py frequency:=5.0
```

## Summary

- Launch files simplify starting complex systems
- Python launch files offer flexibility and programmatic control
- Parameters can be passed at launch time

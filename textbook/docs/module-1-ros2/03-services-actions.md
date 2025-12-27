---
sidebar_position: 4
title: "Services and Actions"
description: "Request-response and long-running task patterns in ROS 2"
keywords: [ros2, services, actions, rpc]
---

# Services and Actions

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Create and call ROS 2 services",
    "Understand the request-response pattern",
    "Implement actions for long-running tasks",
    "Handle action feedback and cancellation"
  ]}
/>

## Services

Services provide synchronous request-response communication.

```python title="add_two_ints_service.py" showLineNumbers
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

## Actions

Actions are for long-running tasks that provide feedback.

```bash
# List available actions
ros2 action list

# Send action goal
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
```

## Summary

- Services use request-response for quick operations
- Actions support feedback and cancellation for long tasks
- Both complement the publish-subscribe pattern of topics

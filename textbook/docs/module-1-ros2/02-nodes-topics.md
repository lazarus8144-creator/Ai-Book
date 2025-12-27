---
sidebar_position: 3
title: "Nodes and Topics"
description: "Understanding ROS 2 communication with nodes and topics"
keywords: [ros2, nodes, topics, pub-sub, communication, publisher, subscriber, messaging, rclpy, python, robotics]
---

# Nodes and Topics

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Create ROS 2 nodes in Python",
    "Publish messages to topics",
    "Subscribe to topics and process messages",
    "Understand the publish-subscribe pattern"
  ]}
/>

## ROS 2 Nodes

A node is a process that performs computation. Robots typically have many nodes working together.

```python title="simple_publisher.py" showLineNumbers
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from ROS 2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
```

## Topics

Topics are named buses for exchanging messages between nodes.

```bash
# List active topics
ros2 topic list

# Echo topic messages
ros2 topic echo /chatter

# Get topic info
ros2 topic info /chatter
```

## Summary

- Nodes are independent processes in ROS 2
- Topics enable publish-subscribe communication
- Multiple nodes can publish/subscribe to the same topic

---
sidebar_position: 6
title: "Debugging ROS 2"
description: "Tools and techniques for troubleshooting ROS 2 applications"
keywords: [ros2, debugging, troubleshooting, tools]
---

# Debugging ROS 2

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Use ROS 2 command-line tools for debugging",
    "Inspect node graphs and communication",
    "Monitor system performance",
    "Troubleshoot common issues"
  ]}
/>

## Essential Debugging Tools

### Node Inspection

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /my_node

# View node graph
ros2 run rqt_graph rqt_graph
```

### Topic Debugging

```bash
# Monitor topic frequency
ros2 topic hz /my_topic

# Check message types
ros2 topic type /my_topic

# View topic bandwidth
ros2 topic bw /my_topic
```

### Logging

```python title="logger_example.py" showLineNumbers
# Different log levels
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

## Common Issues

| Issue | Solution |
|-------|----------|
| Node not found | Check if node is running with `ros2 node list` |
| Topic not publishing | Verify publisher with `ros2 topic info` |
| Build errors | Clean workspace: `rm -rf build install log` |

## Summary

- ROS 2 provides comprehensive CLI tools for debugging
- Logging helps track application behavior
- Visual tools like rqt_graph show system architecture

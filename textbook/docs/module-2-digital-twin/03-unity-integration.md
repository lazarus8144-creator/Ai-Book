---
sidebar_position: 4
title: "Unity Integration"
description: "Integrating Unity with ROS 2 for photorealistic rendering"
keywords: [unity, ros2, rendering, visualization]
---

# Unity Integration

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Set up Unity for robotics",
    "Install ROS-TCP-Connector",
    "Visualize ROS 2 data in Unity",
    "Create photorealistic environments"
  ]}
/>

## Unity for Robotics

Unity provides photorealistic rendering and is increasingly used in robot simulation and training.

## ROS-TCP-Connector

```bash
# In Unity, install via Package Manager:
# - ROS TCP Connector
# - URDF Importer
```

## Connecting to ROS 2

```csharp title="ROSConnection.cs" showLineNumbers
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSPublisher : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("unity_topic");
    }
}
```

## Summary

- Unity excels at visual realism
- ROS-TCP-Connector bridges Unity and ROS 2
- Combined with Gazebo, provides comprehensive simulation

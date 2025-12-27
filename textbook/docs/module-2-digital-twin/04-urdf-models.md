---
sidebar_position: 5
title: "URDF Models"
description: "Creating robot descriptions with URDF"
keywords: [urdf, robot description, kinematics, models]
---

# URDF Models

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Understand URDF structure and syntax",
    "Define robot links and joints",
    "Add visual and collision geometry",
    "Load URDF models in Gazebo"
  ]}
/>

## URDF Structure

URDF (Unified Robot Description Format) describes robot kinematics and dynamics in XML.

```xml title="simple_robot.urdf" showLineNumbers
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Loading in Gazebo

```bash
# Spawn URDF in Gazebo
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file simple_robot.urdf
```

## Summary

- URDF defines robot structure declaratively
- Links represent rigid bodies
- Joints connect links with kinematic relationships
- Compatible with both Gazebo and RViz

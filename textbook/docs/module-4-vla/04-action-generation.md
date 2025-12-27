---
sidebar_position: 5
title: "Action Generation"
description: "From intentions to robot movements"
keywords: [action generation, motion planning, control, execution]
---

# Action Generation

import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives
  objectives={[
    "Generate robot actions from VLA outputs",
    "Implement action primitives for manipulation",
    "Execute motion plans safely",
    "Handle execution failures and recovery"
  ]}
/>

## VLA Action Generation

VLA models output action tokens that are converted to robot commands.

```python title="action_generator.py" showLineNumbers
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ActionGenerator(Node):
    def __init__(self):
        super().__init__('action_generator')
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def execute_action(self, action_token):
        """
        Convert VLA action token to robot command
        """
        cmd = Twist()

        if action_token == "move_forward":
            cmd.linear.x = 0.5
        elif action_token == "turn_left":
            cmd.angular.z = 0.5
        elif action_token == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Executing: {action_token}')
```

## Action Primitives

```python title="primitives.py" showLineNumbers
class ActionPrimitives:
    """
    High-level action primitives for VLA
    """
    def grasp(self, object_pose):
        # 1. Plan approach
        # 2. Open gripper
        # 3. Move to pre-grasp
        # 4. Move to grasp
        # 5. Close gripper
        pass

    def navigate(self, target_pose):
        # Use Nav2 for navigation
        pass

    def place(self, target_location):
        # 1. Navigate to location
        # 2. Lower object
        # 3. Open gripper
        pass
```

## Closed-Loop Execution

```python title="execution_loop.py" showLineNumbers
def execute_with_feedback(vla_model, camera, robot):
    """
    Closed-loop execution with visual feedback
    """
    instruction = "Pick up the red cube"

    for step in range(max_steps):
        # Get current observation
        image = camera.capture()

        # Get action from VLA
        action = vla_model.predict(instruction, image)

        # Execute action
        robot.execute(action)

        # Check if task complete
        if task_completed(image):
            break
```

## Summary

- Action generation converts VLA outputs to robot commands
- Primitives provide reusable building blocks
- Closed-loop execution enables adaptive behavior
- Visual feedback ensures task completion

---

**Congratulations!** You've completed all four modules of the Physical AI & Humanoid Robotics course.

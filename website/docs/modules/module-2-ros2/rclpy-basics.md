---
id: rclpy-basics
title: rclpy Basics
sidebar_label: rclpy Basics
---

# rclpy Basics

## Learning Objectives

- Write ROS 2 nodes in Python (rclpy)
- Create publishers, subscribers, services, and actions
- Handle lifecycle and error management

## Prerequisites

- Read: [Nodes and Topics](./nodes-topics)

## Code Example: Complete Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = 0.1
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
```

## System Connectivity

Next: [Module 2 Summary](./module-2-summary)

## References

- Open Robotics (2024). *rclpy Documentation*. https://docs.ros.org/en/humble/Rclpy

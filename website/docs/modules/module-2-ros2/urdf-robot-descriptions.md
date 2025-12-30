---
id: urdf-robot-descriptions
title: URDF Robot Descriptions
sidebar_label: URDF Robot Descriptions
---

# URDF Robot Descriptions

## Learning Objectives

- Understand URDF (Unified Robot Description Format)
- Describe robot kinematics and geometry in XML
- Link URDF to ROS 2 simulation and visualization

## Prerequisites

- Read: [ROS 2 Architecture](./ros2-architecture)

## URDF Structure

```mermaid
graph LR
    A[URDF File] --> B[Robot Element]
    B --> C[Link<br/>Physical Bodies]
    B --> D[Joint<br/>Connections]
    B --> E[Material/Visual]

    C -->|Defines| F[Geometry<br/>Mass<br/>Inertia]
    D -->|Defines| G[Type<br/>Axis<br/>Limits]
```

## System Connectivity

Next: [rclpy Basics](./rclpy-basics)

## References

- Open Robotics (2024). *URDF Documentation*. https://wiki.ros.org/urdf

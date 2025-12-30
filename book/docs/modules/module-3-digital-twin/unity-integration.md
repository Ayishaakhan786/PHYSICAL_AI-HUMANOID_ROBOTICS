---
id: unity-integration
title: Unity Integration
sidebar_label: Unity Integration
---

# Unity Integration

## Learning Objectives

- Understand Unity as robotics simulation platform
- Integrate Unity with ROS 2 via bridge
- Create realistic environments for robot simulation

## Prerequisites

- Read: [Gazebo Concepts](./gazebo-concepts)

## Unity vs Gazebo

| Feature | Unity | Gazebo |
|---------|--------|----------|
| **Rendering** | Photorealistic | Basic |
| **Physics** | Built-in/PhysX | ODE/Bullet/Simbody |
| **ROS Integration** | Via bridge | Native |
| **Assets** | Large store | Limited |

## Unity-ROS Bridge

```mermaid
graph LR
    A[Unity Scene] --> B[Unity-ROS Bridge]
    B --> C[ROS 2 Topics]
    C --> D[ROS 2 Nodes]

    style B fill:#ffe1e1
```

## System Connectivity

Next: [Sensor Modeling](./sensor-modeling)

## References

- Unity Technologies (2024). *Unity Robotics Hub*. https://unity.com/robotics

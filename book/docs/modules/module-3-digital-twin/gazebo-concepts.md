---
id: gazebo-concepts
title: Gazebo Concepts
sidebar_label: Gazebo Concepts
---

# Gazebo Concepts

## Learning Objectives

- Understand Gazebo physics simulator
- Create simulation worlds and robot models
- Use Gazebo with ROS 2 for sensor simulation

## Prerequisites

- Read: [Module 2: ROS 2 Fundamentals](../module-2-ros2/ros2-architecture)

## Gazebo Architecture

```mermaid
graph LR
    A[Gazebo Server<br/>Physics Engine] --> B[ROS 2 Bridge]
    B --> C[ROS 2 Topics]
    D[ROS 2 World] -->|Publishes| E[Gazebo Plugins]
    E -->|Controls physics| A
```

## Physics Engines

- **ODE** (Open Dynamics Engine)
- **Bullet** (PyBullet)
- **Simbody** (multibody dynamics)

## System Connectivity

Next: [Unity Integration](./unity-integration)

## References

- Gazebo (2024). *Gazebo Documentation*. http://gazebosim.org/tutorials

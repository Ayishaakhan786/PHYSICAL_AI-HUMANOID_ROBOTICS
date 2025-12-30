---
id: humanoid-kinematics
title: Humanoid Kinematics
sidebar_label: Humanoid Kinematics
---

# Humanoid Kinematics

## Learning Objectives

- Understand forward and inverse kinematics for humanoids
- Describe humanoid joint structure and degrees of freedom
- Explain kinematic chains for leg and arm motion

## Prerequisites

- Read: [Module 2: ROS 2 Fundamentals](../module-2-ros2/ros2-architecture)

## Kinematic Chains

```mermaid
graph TB
    A[Torso] --> B[Left Hip]
    A --> C[Right Hip]
    B --> D[Left Knee]
    C --> E[Right Knee]
    D --> F[Left Ankle]
    E --> G[Right Ankle]

    style A fill:#e1f5ff
```

## Forward vs Inverse Kinematics

| Type | Description | Complexity |
|-------|-------------|-----------|
| **Forward** | Joint angles → End-effector position | Simple, O(n) |
| **Inverse** | End-effector position → Joint angles | Complex, O(n³) |

## System Connectivity

Next: [Locomotion](./locomotion)

## References

- Kajita, S. (2007). *Inverse Kinematics for Humanoid Robots*. Springer.

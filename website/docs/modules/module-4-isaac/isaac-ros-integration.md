---
id: isaac-ros-integration
title: Isaac ROS Integration
sidebar_label: Isaac ROS Integration
---

# Isaac ROS Integration

## Learning Objectives

- Integrate Isaac Sim with ROS 2
- Use Isaac ROS packages for GPU robotics
- Bridge simulation and real-world ROS 2 workflows

## Prerequisites

- Read: [VSLAM and Nav2](./vslam-nav2)

## Isaac ROS

```mermaid
graph LR
    A[Isaac Sim] --> B[Isaac ROS]
    B --> C[ROS 2 Topics]
    C --> D[ROS 2 Nodes]
    C --> E[Real Robot]

    style B fill:#ffe1e1
```

## System Connectivity

Next: [Simulation to Real Training](./simulation-to-real-training)

## References

- NVIDIA (2024). *Isaac ROS Documentation*. https://developer.nvidia.com/isaac-ros

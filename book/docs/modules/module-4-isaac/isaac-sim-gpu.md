---
id: isaac-sim-gpu
title: Isaac Sim GPU
sidebar_label: Isaac Sim GPU
---

# Isaac Sim GPU Acceleration

## Learning Objectives

- Understand Isaac Sim architecture and GPU acceleration
- Set up Isaac Sim for robotics development
- Use Isaac Lab for reinforcement learning

## Prerequisites

- Read: [Module 3: Digital Twin](../module-3-digital-twin/gazebo-concepts)

## Isaac Sim Overview

```mermaid
graph LR
    A[Isaac Sim GUI] --> B[Omniverse Platform]
    B --> C[GPU Accelerated Physics]
    C --> D[ROS 2 Bridge]
    D --> E[Isaac Lab RL]

    style C fill:#e1ffe1
```

## GPU Benefits

- **Faster-than-real-time**: 10-100x speed
- **Parallel training**: Thousands of environments
- **Photorealistic rendering**: Ray tracing

## System Connectivity

Next: [Perception Pipelines](./perception-pipelines)

## References

- NVIDIA (2024). *Isaac Sim Documentation*. https://docs.omniverse.nvidia.com/isaac-sim

---
id: simulation-to-real-training
title: Simulation to Real Training
sidebar_label: Sim-to-Real Training
---

# Simulation to Real Training

## Learning Objectives

- Use Isaac Sim for faster-than-real-time training
- Apply domain randomization in Isaac
- Transfer trained policies to real robots

## Prerequisites

- Read: [Isaac ROS Integration](./isaac-ros-integration)

## Isaac Lab RL

```mermaid
graph TB
    A[Isaac Lab] --> B[Parallel Environments]
    B --> C[GPU Training]
    C --> D[Policy Export]
    D --> E[Robot Deployment]

    style C fill:#e1ffe1
```

## System Connectivity

Next: [Module 4 Summary](./module-4-summary)

## References

- NVIDIA (2024). *Isaac Lab Documentation*. https://docs.omniverse.nvidia.com/isaac-lab

---
id: perception-pipelines
title: Perception Pipelines
sidebar_label: Perception Pipelines
---

# Perception Pipelines

## Learning Objectives

- Understand Isaac perception components
- Use Isaac Replicator for synthetic data
- Build vision pipelines for robotics

## Prerequisites

- Read: [Isaac Sim GPU](./isaac-sim-gpu)

## Perception Components

```mermaid
graph TB
    A[Sensors] --> B[Isaac Perception]
    B --> C[Object Detection]
    B --> D[Depth Perception]
    B --> E[Semantic Segmentation]
    B --> F[Reprojection]

    style B fill:#e1f5ff
```

## Isaac Replicator

Generate synthetic training data at scale.

## System Connectivity

Next: [VSLAM and Nav2](./vslam-nav2)

## References

- NVIDIA (2024). *Isaac Perception Documentation*. https://docs.omniverse.nvidia.com

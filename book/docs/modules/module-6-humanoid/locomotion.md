---
id: locomotion
title: Locomotion
sidebar_label: Locomotion
---

# Bipedal Locomotion

## Learning Objectives

- Understand bipedal walking patterns for humanoids
- Explain dynamic walking and balance control
- Describe gait cycles for humanoid locomotion

## Prerequisites

- Read: [Humanoid Kinematics](./humanoid-kinematics)

## Gait Cycle

```mermaid
graph LR
    A[Stance Phase<br/>Support] --> B[Swing Phase<br/>Swing]
    B --> C[Stance Phase]
    C -.Double Support.- D[Transition]

    style A fill:#e1ffe1
    style B fill:#ffe1e1
```

## Zero Moment Point (ZMP)

Critical concept for bipedal stability.

## System Connectivity

Next: [Human Interaction](./human-interaction)

## References

- Kajita, S., et al. (2014). *Humanoid Walking and Running*. Springer.

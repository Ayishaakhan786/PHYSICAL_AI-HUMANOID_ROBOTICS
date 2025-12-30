---
id: domain-randomization
title: Domain Randomization
sidebar_label: Domain Randomization
---

# Domain Randomization

## Learning Objectives

- Understand domain randomization for sim-to-real
- Apply randomization to improve robustness
- Bridge the reality gap through training variability

## Prerequisites

- Read: [Hardware Summary](../hardware/hardware-summary)

## Randomization Techniques

| Parameter | Range | Purpose |
|-----------|------|---------|
| **Lighting** | 0.3-1.0 intensity | Handle real lighting variations |
| **Textures** | Random materials | Reduce overfitting |
| **Physics** | Friction 0.3-0.9 | Generalize to surfaces |
| **Camera Noise** | 0.0-0.1 | Simulate sensor noise |

## System Connectivity

Next: [Reality Gap](./reality-gap)

## References

- Tobin, J., et al. (2017). *Domain Randomization for Robotics*. arXiv.

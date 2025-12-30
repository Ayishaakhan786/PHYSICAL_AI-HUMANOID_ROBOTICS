---
id: vision-language-action
title: Vision-Language-Action
sidebar_label: Vision-Language-Action
---

# Vision-Language-Action Systems

## Learning Objectives

- Understand VLA (Vision-Language-Action) architecture
- Learn how vision and language models integrate with robotic actions
- Explore multimodal transformer architectures for robotics

## Prerequisites

- Read: [Module 4: NVIDIA Isaac](../module-4-isaac/isaac-sim-gpu)

## VLA Concept

```mermaid
graph TB
    A[Vision Input<br/>Image] --> B[VLA Model<br/>Multimodal Transformer]
    C[Language Input<br/>Command] --> B
    B --> D[Action Output<br/>Robot Control]
    D --> E[Physical Robot]

    style B fill:#e1f5ff
```

## System Connectivity

Next: [LLM Planning](./llm-planning)

## References

- Driess, D., et al. (2023). *PaLM-E: Embodied Multimodal Language Model*. arXiv.

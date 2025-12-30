---
id: conversational-robotics
title: Conversational Robotics
sidebar_label: Conversational Robotics
---

# Conversational Robotics

## Learning Objectives

- Design natural language interfaces for robots
- Handle multi-turn conversations and context
- Combine speech, vision, and action in robotic systems

## Prerequisites

- Read: [Speech Models](./speech-models)

## Conversational Pipeline

```mermaid
graph TB
    A[User Voice] --> B[Speech Recognition]
    B --> C[Text Understanding<br/>LLM]
    C --> D[Task Planning]
    D --> E[Robot Execution]
    E --> F[Response Generation]
    F --> G[Speech Synthesis<br/>TTS]

    style C fill:#e1f5ff
```

## System Connectivity

Next: [Module 5 Summary](./module-5-summary)

## References

- NVIDIA (2024). *Riva Conversational AI*. https://developer.nvidia.com/riva

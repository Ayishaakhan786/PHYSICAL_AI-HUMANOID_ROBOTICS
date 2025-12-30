---
id: llm-planning
title: LLM Planning
sidebar_label: LLM Planning
---

# LLM Planning

## Learning Objectives

- Understand how LLMs plan robotic tasks
- Implement task planning using language models
- Translate natural language to robot actions

## Prerequisites

- Read: [Vision-Language-Action](./vision-language-action)

## LLM for Task Planning

```mermaid
graph LR
    A[User Command<br/>'Pick up the red cup'] --> B[LLM Planner]
    B --> C[Task Decomposition<br/>1. Detect cup<br/>2. Plan grasp<br/>3. Execute]
    C --> D[ROS 2 Actions]
    D --> E[Robot Execution]

    style B fill:#ffe1e1
```

## System Connectivity

Next: [Speech Models](./speech-models)

## References

- OpenAI (2024). *GPT-4 for Robotics*. https://openai.com

---
id: speech-models
title: Speech Models
sidebar_label: Speech Models
---

# Speech Models

## Learning Objectives

- Use speech-to-text models for robot control
- Implement voice interfaces with Whisper and similar models
- Handle noise and ambiguity in speech recognition

## Prerequisites

- Read: [LLM Planning](./llm-planning)

## Speech Pipeline

```mermaid
graph LR
    A[Voice Input<br/>Microphone] --> B[STT Model<br/>Whisper]
    B --> C[Text Command]
    C --> D[LLM Planner]
    D --> E[Robot Actions]

    style B fill:#e1ffe1
```

## System Connectivity

Next: [Conversational Robotics](./conversational-robotics)

## References

- OpenAI (2024). *Whisper Documentation*. https://github.com/openai/whisper

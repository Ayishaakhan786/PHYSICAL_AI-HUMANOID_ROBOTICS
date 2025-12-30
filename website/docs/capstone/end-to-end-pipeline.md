---
id: end-to-end-pipeline
title: End-to-End Pipeline
sidebar_label: End-to-End Pipeline
---

# End-to-End Pipeline

## Learning Objectives

- Trace complete flow from voice to robot action
- Understand data transformations at each stage
- Identify latency and integration points

## Prerequisites

- Read: [Autonomous Humanoid System](./autonomous-humanoid-system)

## Pipeline Stages

### 1. Voice Input → Speech Recognition (100-300ms)
### 2. Text → LLM Planning (200-1000ms)
### 3. Planning → ROS 2 Actions (50-200ms)
### 4. ROS 2 → Motor Commands (10-50ms)
### 5. Commands → Actuation (10-100ms)

## Total Latency Budget: ~500-1500ms

## System Connectivity

Next: [Design Tradeoffs](./design-tradeoffs)

## References

- Real-Time Systems (2024). *Latency Analysis*. IEEE.

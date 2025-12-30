---
id: cloud-labs
title: Cloud Labs
sidebar_label: Cloud Labs
---

# Cloud Labs for Robotics

## Learning Objectives

- Understand cloud architectures for robotics
- Compare on-prem vs cloud for different use cases
- Configure cloud resources for robot development

## Prerequisites

- Read: [On-Prem Labs](./on-prem-labs)

## Cloud Components

```mermaid
graph TB
    A[GPU Clusters<br/>A100/H100] --> B[Orchestration<br/>Kubernetes]
    C[Storage<br/>Object Storage] --> B
    B --> D[Training Jobs]
    B --> E[Robot Connection<br/>VPN/Edge]
    E --> F[Real Robots]

    style B fill:#e1ffe1
```

## CapEx vs OpEx

| Factor | On-Prem | Cloud |
|--------|----------|-------|
| **Cost** | High upfront | Pay-as-you-go |
| **Scalability** | Limited | Elastic |
| **Control** | Full | Managed |
| **Latency** | Low | Network dependent |

## System Connectivity

Next: [Hardware Summary](./hardware-summary)

## References

- AWS (2024). *AWS RoboMaker*. https://aws.amazon.com/robomaker

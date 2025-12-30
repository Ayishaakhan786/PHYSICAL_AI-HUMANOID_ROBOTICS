---
id: on-prem-labs
title: On-Prem Labs
sidebar_label: On-Prem Labs
---

# On-Premises Labs

## Learning Objectives

- Design on-premise robotics labs
- Understand CapEx requirements for hardware
- Configure network and storage for robotics work

## Prerequisites

- Read: [Sensors](./sensors-robotics)

## Lab Components

```mermaid
graph TB
    A[Workstations] --> B[High-Speed Network<br/>10GbE+]
    C[GPU Servers] --> B
    B --> D[Shared Storage<br/>NAS/SAN]
    B --> E[Power & Cooling]
    F[Robots] --> B

    style B fill:#e1f5ff
```

## System Connectivity

Next: [Cloud Labs](./cloud-labs)

## References

- NVIDIA (2024). *DGX System Documentation*. https://www.nvidia.com

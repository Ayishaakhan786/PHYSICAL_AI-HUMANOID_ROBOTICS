---
id: reality-gap
title: Reality Gap
sidebar_label: Reality Gap
---

# The Reality Gap

## Learning Objectives

- Identify sources of sim-to-real discrepancies
- Understand why simulation doesn't perfectly match reality
- Apply techniques to narrow the gap

## Prerequisites

- Read: [Domain Randomization](./domain-randomization)

## Gap Sources

| Source | Simulation | Reality | Mitigation |
|---------|------------|---------|-------------|
| **Physics** | Perfect | Imperfect, drift | System ID |
| **Sensors** | Ideal/Calibrated | Noisy | Add noise |
| **Actuators** | Exact | Delay, backlash | Model delays |
| **Environment** | Controlled | Unpredictable | Randomize |

## System Connectivity

Next: [Transfer Learning](./transfer-learning)

## References

- Kuffner, J. (2008). *Sim-to-Real: Challenges and Solutions*. RSS.

---
id: environment-modeling
title: Environment Modeling
sidebar_label: Environment Modeling
---

# Environment Modeling

## Learning Objectives

- Create virtual worlds in Gazebo/Unity
- Add objects, lighting, and environmental effects
- Model dynamic environments for robot interaction

## Prerequisites

- Read: [Sensor Modeling](./sensor-modeling)

## World Files

```xml
<!-- Gazebo world file -->
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 -0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
  </world>
</sdf>
```

## System Connectivity

Next: [Gazebo vs Unity Tradeoffs](./gazebo-vs-unity-tradeoffs)

## References

- Gazebo (2024). *SDF World Format*. http://sdformat.org

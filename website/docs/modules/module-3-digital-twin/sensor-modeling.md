---
id: sensor-modeling
title: Sensor Modeling
sidebar_label: Sensor Modeling
---

# Sensor Modeling

## Learning Objectives

- Understand how sensors are modeled in simulation
- Add noise and realistic characteristics to virtual sensors
- Configure cameras, LiDAR, IMU in Gazebo/Unity

## Prerequisites

- Read: [Gazebo Concepts](./gazebo-concepts)

## Sensor Types

### Camera Simulation

```python
# Gazebo camera plugin configuration
<plugin filename="libgazebo_ros_camera.so" name="camera_controller">
  <always_on>true</always_on>
  <update_rate>30.0</update_rate>
  <camera_name>head_camera</camera_name>
  <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
  <image>
    <width>640</width>
    <height>480</height>
    <format>R8G8B8</format>
  </image>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.007</stddev>
  </noise>
</plugin>
```

### LiDAR Simulation

Add ray-casting for distance measurement.

### IMU Simulation

Model accelerometer and gyroscope with noise.

## System Connectivity

Next: [Environment Modeling](./environment-modeling)

## References

- Gazebo (2024). *Sensor Plugins*. http://gazebosim.org/tutorials

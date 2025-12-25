# Chapter 2 Code Examples

This directory contains executable code examples for Chapter 2: Sensor and Actuator Data Flow.

## Available Examples

### 1. publisher_subscriber.py
A complete example demonstrating the publisher-subscriber pattern with proper node structure, publishers, and subscribers.

### 2. service_client_server.py
An example showing how to implement and use ROS 2 services for request-response communication.

### 3. sensor_data_processing.py
A practical example of processing sensor data from multiple sources and generating appropriate responses.

### 4. joint_control.py
An example demonstrating joint control using PID control patterns.

### 5. sensor_actuator_implementation.py
An example demonstrating sensor and actuator interface implementations.

## Running the Examples

To run these examples, make sure you have ROS 2 installed and sourced:

```bash
# Source ROS 2 (example for Humble Hawksbill)
source /opt/ros/humble/setup.bash

# Run the example
python3 publisher_subscriber.py
```

## Requirements

- ROS 2 (Humble Hawksbill or later recommended)
- Python 3.8 or later
- rclpy package (included with ROS 2)
- Additional packages as specified in individual examples
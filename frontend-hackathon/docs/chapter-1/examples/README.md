# Chapter 1 Code Examples

This directory contains executable code examples for Chapter 1: ROS 2 as the Robotic Nervous System.

## Available Examples

### 1. simple_node.py
A basic ROS 2 node demonstrating:
- Node creation and initialization
- Publisher and subscriber setup
- Timer-based callbacks
- Basic logging

To run:
```bash
# Make sure you have ROS 2 installed and sourced
python3 simple_node.py
```

### 2. executor_example.py
Demonstrates different types of executors in ROS 2:
- Single-threaded executor
- Multi-threaded executor
- Node management with executors

To run:
```bash
# Make sure you have ROS 2 installed and sourced
python3 executor_example.py
```

## Requirements

To run these examples, you need:
- ROS 2 (Humble Hawksbill or later recommended)
- Python 3.8 or later
- rclpy package (typically installed with ROS 2)

## How to Use

1. Make sure your ROS 2 environment is sourced:
   ```bash
   source /opt/ros/humble/setup.bash  # or your ROS 2 distribution
   ```

2. Run the examples from a terminal where ROS 2 is properly configured

3. Use `ros2 run` or `python3` to execute the examples

## Understanding the Examples

Each example is fully documented with comments explaining:
- The purpose of the code
- Key concepts being demonstrated
- How different parts work together
- Best practices for ROS 2 development
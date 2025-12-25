---
title: Isaac Stack Overview
description: Introduction to the NVIDIA Isaac robotics platform ecosystem
sidebar_label: Isaac Stack Overview
---

# Isaac Stack Overview

## Introduction to Isaac Platform

The NVIDIA Isaac platform is a comprehensive robotics solution that includes simulation, development tools, and AI capabilities for building and deploying robotics applications. It consists of several key components designed to accelerate the development of AI-powered robots.

## Key Components and Architecture

The Isaac platform is built around several core components:

1. **Isaac Sim**: NVIDIA's robotics simulation platform built on Omniverse, providing photorealistic simulation environments with realistic physics and material properties.

2. **Isaac ROS**: A collection of hardware acceleration packages for ROS 2 that provide GPU-accelerated perception and navigation algorithms.

3. **Isaac ROS GEMs**: GPU-accelerated Execution Modules that offer specialized algorithms optimized for robotics applications.

4. **Isaac Apps**: Pre-built reference applications that demonstrate best practices and accelerate development.

## Integration with ROS 2 Ecosystem

The Isaac platform seamlessly integrates with the ROS 2 ecosystem, allowing developers to leverage existing ROS 2 tools, packages, and knowledge while benefiting from NVIDIA's GPU acceleration and simulation capabilities.

Key integration points include:
- Standard ROS 2 interfaces and message types
- Support for common ROS 2 tools like RViz and rqt
- Compatibility with popular ROS 2 packages and libraries

## Learning Objectives

After completing this chapter, you will be able to:
- Identify the main components of the Isaac ecosystem
- Understand the primary functions of each component
- Explain how Isaac components integrate with ROS 2

## Prerequisites

- Basic understanding of robotics frameworks
- Familiarity with ROS 2 concepts
- Knowledge of simulation environments

## Further Reading

- [NVIDIA Isaac Documentation](https://nvidia-isaac.readthedocs.io/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Omniverse Isaac Sim User Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/)

## Next Steps

- Continue to [Isaac Sim Introduction](./isaac-sim-introduction) to learn about simulation capabilities
- Explore [Isaac ROS Integration](./isaac-ros-integration) for GPU-accelerated algorithms
- Learn about [Synthetic Data Generation](../chapter-9/synthetic-data-concepts) in the next module
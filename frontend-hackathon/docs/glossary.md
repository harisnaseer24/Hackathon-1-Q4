---
sidebar_position: 100
---

# Glossary

## A

**Action Server**: A ROS 2 construct that handles long-running tasks with feedback and goal management, commonly used for robot behaviors that take time to complete.

**AI-ROS Bridge**: The integration layer that connects AI decision-making systems with ROS 2 control systems, enabling AI agents to control physical robots.

## B

**Behavior Tree**: A hierarchical structure used in AI systems to organize and execute complex robot behaviors in a modular and reusable way.

## C

**Cognitive-Physical Loop**: The continuous cycle of sensing → reasoning → acting → sensing that connects AI cognition with physical robot behavior, essential for humanoid robots to adapt to dynamic environments.

**Collision Element**: A URDF component that defines how a robot link interacts with the environment in physics simulation.

## D

**DDS (Data Distribution Service)**: The middleware technology underlying ROS 2 that provides reliable, real-time data exchange between nodes.

**Decision-to-Action Pipeline**: The sequence of components that converts high-level AI decisions into executable robot commands, including perception, reasoning, action planning, execution, and feedback.

## E

**Executor**: In ROS 2, a component that runs callbacks for nodes, managing the execution of publishers, subscribers, services, and timers.

## G

**Gazebo**: A 3D simulation environment that provides high-fidelity physics simulation for robotics applications.

**Gazebo Plugin**: A software component that extends Gazebo's functionality, often used to interface with ROS 2 systems.

## I

**Inertial Element**: A URDF component that defines mass properties (mass, center of mass, and inertia tensor) for physics simulation.

**Inertial Measurement Unit (IMU)**: A sensor that measures specific force, angular rate, and sometimes magnetic fields to determine orientation and movement.

## J

**Joint**: In URDF, a connection between two links that defines how they can move relative to each other.

**Joint State Publisher**: A ROS 2 node that publishes the current state of all joints in a robot.

## L

**Link**: In URDF, a rigid body part of the robot that has physical and visual properties.

**Lifecycle Node**: A ROS 2 node that follows a well-defined state machine pattern for initialization, configuration, and cleanup.

## N

**Node**: The basic execution unit in ROS 2 that performs computation, containing publishers, subscribers, services, and other ROS graph components.

## P

**Publisher**: A ROS 2 component that sends messages to a topic, allowing data to be broadcast to multiple subscribers.

**QoS (Quality of Service)**: A set of policies in ROS 2 that define how messages are delivered, including reliability, durability, and history settings.

## R

**rclpy**: The Python client library for ROS 2, providing the interface between Python programs and the ROS 2 middleware.

**ROS 2 (Robot Operating System 2)**: A flexible framework for writing robot software that provides hardware abstraction, device drivers, libraries, and message-passing functionality.

**Reactive AI**: An AI approach that responds immediately to environmental stimuli without maintaining complex internal states, suitable for immediate obstacle avoidance and emergency responses.

## S

**Service**: A ROS 2 communication pattern that provides request-response interactions between nodes, useful for specific queries or actions.

**Simulation**: The process of modeling a real-world system to study its behavior under various conditions without physical hardware.

**Subscriber**: A ROS 2 component that receives messages from a topic, allowing nodes to receive data from publishers.

## T

**Transform (TF)**: A system in ROS 2 that keeps track of coordinate frames and their relationships over time, essential for spatial reasoning.

**Transmission**: In URDF, the definition of the relationship between joints and actuators, enabling ROS control of physical or simulated joints.

**Topic**: A named bus over which nodes exchange messages in a publish-subscribe communication pattern.

## U

**URDF (Unified Robot Description Format)**: An XML-based format used in ROS to describe robot models, including links, joints, and their properties.

## V

**Visual Element**: A URDF component that defines how a robot link appears in visualization tools like RViz and Gazebo.

## W

**World**: In simulation environments like Gazebo, the environment where robot models are placed and simulated.

## X

**Xacro**: An XML macro language that extends URDF, allowing for parameterization, inclusion, and macro definitions to simplify complex robot descriptions.

## Z

**Zeroconf**: A set of technologies that enable automatic discovery of devices and services on IP networks, used in ROS 2 for node discovery.
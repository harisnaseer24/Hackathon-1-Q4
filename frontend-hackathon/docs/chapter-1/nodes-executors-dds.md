---
sidebar_position: 3
title: "Nodes, Executors, and DDS Communication"
---

# Nodes, Executors, and DDS Communication

## Introduction

The core of ROS 2's distributed architecture lies in its communication model, which is built around three fundamental concepts: nodes, executors, and the DDS (Data Distribution Service) middleware. Understanding these concepts is crucial for developing effective robotic applications, as they form the foundation of all inter-component communication in ROS 2 systems.

## Understanding Nodes

A node is the fundamental unit of computation in ROS 2. It encapsulates a single purpose within the robotic system, such as sensor data processing, motion control, or decision-making. Nodes are designed to be modular and independent, allowing complex systems to be built by connecting multiple nodes together.

### Key Characteristics of Nodes

- **Single Responsibility**: Each node should have one well-defined purpose
- **Communication Interface**: Nodes communicate with other nodes through topics, services, and actions
- **Lifecycle Management**: Nodes can be started, stopped, and managed independently
- **Resource Management**: Nodes handle their own resources (timers, subscriptions, publishers, etc.)

### Creating a Node

Here's an example of a simple ROS 2 node implementation:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.listener_callback,
            10
        )

        # Create a timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Executors: The Node Managers

Executors are responsible for managing the execution of nodes and their associated callbacks. They determine how and when callbacks are processed, enabling different execution strategies for various application needs.

### Types of Executors

#### Single-threaded Executor
The default executor that processes all callbacks in a single thread. This ensures that callbacks don't run concurrently, simplifying synchronization but potentially limiting performance.

```python
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    node1 = Node('node1')
    node2 = Node('node2')

    # Create single-threaded executor
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

#### Multi-threaded Executor
Processes callbacks using a thread pool, allowing multiple callbacks to run concurrently. This can improve performance but requires careful attention to thread safety.

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node1 = Node('node1')
    node2 = Node('node2')

    # Create multi-threaded executor with 4 threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

## DDS: The Communication Middleware

DDS (Data Distribution Service) is the underlying middleware that provides the communication infrastructure for ROS 2. It implements the publish-subscribe pattern and provides Quality of Service (QoS) policies that control how data is transmitted between nodes.

### DDS Architecture

DDS is built on a publish-subscribe model where:

- **Publishers** send data to topics without knowing who will receive it
- **Subscribers** receive data from topics without knowing who sent it
- **DDS Domain** defines the scope of communication (typically a single robot or robot fleet)

### Quality of Service (QoS) Policies

DDS provides several QoS policies that control communication behavior:

#### Reliability Policy
- **RELIABLE**: All messages are guaranteed to be delivered (at the cost of potential delays)
- **BEST_EFFORT**: Messages may be lost, but lower latency is prioritized

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable communication for critical data
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)

publisher = node.create_publisher(String, 'critical_topic', qos_profile)
```

#### Durability Policy
- **TRANSIENT_LOCAL**: Late-joining subscribers receive the most recent value
- **VOLATILE**: Late-joining subscribers only receive new values

#### History Policy
- **KEEP_LAST**: Store only the most recent N messages
- **KEEP_ALL**: Store all messages (subject to resource limits)

```python
from rclpy.qos import QoSProfile, HistoryPolicy

# Keep all messages for configuration data
qos_profile = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_ALL
)

config_publisher = node.create_publisher(Config, 'config_topic', qos_profile)
```

## Communication Patterns in ROS 2

### Topics (Publish-Subscribe)
Topics enable asynchronous, one-way communication between nodes. Publishers send messages to topics, and subscribers receive messages from topics.

```python
# Publisher example
publisher = self.create_publisher(String, 'chatter', 10)

# Subscriber example
subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10
)
```

### Services (Request-Response)
Services enable synchronous, two-way communication where a client sends a request and receives a response.

```python
from example_interfaces.srv import AddTwoInts

# Service server
def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    return response

service = self.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

# Service client
client = self.create_client(AddTwoInts, 'add_two_ints')
```

### Actions (Goal-Fedback-Result)
Actions enable long-running tasks with feedback and cancellation capabilities.

```python
from example_interfaces.action import Fibonacci
import rclpy.action

# Action server
action_server = rclpy.action.ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    execute_callback=self.execute_fibonacci
)

# Action client
action_client = rclpy.action.ActionClient(self, Fibonacci, 'fibonacci')
```

## Best Practices for Nodes and Communication

### Node Design
- Keep nodes focused on a single responsibility
- Use meaningful names that reflect the node's purpose
- Implement proper error handling and logging
- Consider resource usage and cleanup

### Communication Design
- Choose appropriate QoS policies for your use case
- Use appropriate message types for your data
- Consider bandwidth and latency requirements
- Plan for failure scenarios

### Executor Selection
- Use SingleThreadedExecutor for simple applications or when thread safety is critical
- Use MultiThreadedExecutor when you need to handle multiple callbacks concurrently
- Consider custom executors for complex scheduling requirements

## Summary

The combination of nodes, executors, and DDS middleware provides a robust and flexible communication framework for robotic applications. Nodes encapsulate functionality, executors manage execution, and DDS provides the underlying communication infrastructure with configurable quality of service. Understanding these concepts is essential for building effective ROS 2 applications that can scale from simple prototypes to complex robotic systems.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the differences between topics, services, and actions in ROS 2. When would you use each communication pattern? Provide specific examples for humanoid robot applications.

2. Describe the role of DDS (Data Distribution Service) in ROS 2. How does it differ from the communication mechanisms in ROS 1?

### Application Questions
3. Create a ROS 2 node that publishes sensor data with different QoS policies (reliable vs best effort) and observe the differences in behavior. Test with both fast (100Hz) and slow (1Hz) publishing rates.

4. Design a system with multiple nodes communicating through topics, services, and actions to implement a simple robot patrol behavior. Include at least 3 nodes and specify the communication patterns between them.

### Hands-on Practice
5. Implement a simple client-server interaction using ROS 2 services. Create a service that calculates the distance between two points and a client that calls this service with different coordinates.

6. Create an action server that implements a "move_to_goal" behavior for a robot. The action should provide feedback on progress and allow for goal cancellation.

### Critical Thinking
7. Compare the performance implications of SingleThreadedExecutor vs MultiThreadedExecutor for different types of robotic applications. When would you choose one over the other?

8. What are the trade-offs between using RELIABLE vs BEST_EFFORT QoS policies for different types of data in a humanoid robot (e.g., sensor data, control commands, logging information)?
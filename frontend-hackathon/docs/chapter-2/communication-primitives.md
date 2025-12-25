---
sidebar_position: 1
title: "ROS 2 Communication Primitives"
---

# ROS 2 Communication Primitives

## Introduction

Communication primitives form the foundation of ROS 2's distributed architecture. These primitives enable nodes to exchange information and coordinate activities across the robotic system. Understanding these primitives is essential for developing effective robotic applications, as they determine how information flows through the system and how components interact with each other.

ROS 2 provides three primary communication patterns:
1. **Topics** - Asynchronous, many-to-many publish-subscribe communication
2. **Services** - Synchronous, request-response communication
3. **Actions** - Asynchronous, goal-feedback-result communication for long-running tasks

Each pattern serves different use cases and has specific characteristics that make it suitable for particular types of interactions in robotic systems.

## Topics: Publish-Subscribe Pattern

Topics implement the publish-subscribe communication pattern, which is ideal for streaming data and broadcasting information. In this pattern, publishers send messages to topics without knowing who will receive them, and subscribers receive messages from topics without knowing who sent them.

### Key Characteristics of Topics

- **Asynchronous**: Publishers and subscribers operate independently
- **Many-to-many**: Multiple publishers can send to a topic, and multiple subscribers can receive from it
- **Decoupled**: Publishers and subscribers don't need to exist simultaneously
- **Best-effort delivery**: Messages may be lost depending on QoS settings

### Topic Implementation Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher')

        # Create publishers for different message types
        self.string_publisher = self.create_publisher(String, 'chatter', 10)
        self.counter_publisher = self.create_publisher(Int32, 'counter', 10)

        # Create a timer to publish messages periodically
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Publish string message
        string_msg = String()
        string_msg.data = f'Hello World: {self.counter}'
        self.string_publisher.publish(string_msg)

        # Publish counter message
        counter_msg = Int32()
        counter_msg.data = self.counter
        self.counter_publisher.publish(counter_msg)

        self.counter += 1

class TopicSubscriber(Node):
    def __init__(self):
        super().__init__('topic_subscriber')

        # Create subscriptions for different topics
        self.string_subscription = self.create_subscription(
            String,
            'chatter',
            self.string_callback,
            10
        )

        self.counter_subscription = self.create_subscription(
            Int32,
            'counter',
            self.counter_callback,
            10
        )

    def string_callback(self, msg):
        self.get_logger().info(f'Received string: {msg.data}')

    def counter_callback(self, msg):
        self.get_logger().info(f'Received counter: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    publisher = TopicPublisher()
    subscriber = TopicSubscriber()

    # Create an executor to manage both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS) for Topics

QoS settings allow you to configure how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Reliable communication for critical sensor data
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# Best effort for high-frequency data where some loss is acceptable
best_effort_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# Publisher with specific QoS
publisher = self.create_publisher(LaserScan, 'laser_scan', reliable_qos)
```

## Services: Request-Response Pattern

Services implement synchronous request-response communication, which is ideal for operations that have a clear request and response. This pattern blocks the client until a response is received, making it suitable for operations that must complete before continuing.

### Key Characteristics of Services

- **Synchronous**: Client waits for response before continuing
- **One-to-one**: One client talks to one server at a time
- **Request-Response**: Clear request message and response message
- **Blocking**: Client thread is blocked until response is received

### Service Implementation Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, SetBool

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')

        # Create service servers
        self.add_service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.bool_service = self.create_service(
            SetBool,
            'set_flag',
            self.set_bool_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

    def set_bool_callback(self, request, response):
        flag_value = request.data
        response.success = True
        response.message = f'Flag set to {flag_value}'
        self.get_logger().info(f'Set flag to: {flag_value}')
        return response

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')

        # Create service clients
        self.add_client = self.create_client(AddTwoInts, 'add_two_ints')
        self.bool_client = self.create_client(SetBool, 'set_flag')

        # Wait for services to be available
        while not self.add_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add service not available, waiting again...')

        while not self.bool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Bool service not available, waiting again...')

        # Call services after a delay
        self.timer = self.create_timer(2.0, self.call_services)

    def call_services(self):
        # Call add service
        add_request = AddTwoInts.Request()
        add_request.a = 42
        add_request.b = 18

        self.add_future = self.add_client.call_async(add_request)
        self.add_future.add_done_callback(self.add_callback)

        # Call bool service
        bool_request = SetBool.Request()
        bool_request.data = True

        self.bool_future = self.bool_client.call_async(bool_request)
        self.bool_future.add_done_callback(self.bool_callback)

    def add_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Add service result: {result.sum}')

    def bool_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Bool service result: {result.message}')

def main(args=None):
    rclpy.init(args=args)

    server = ServiceServer()
    client = ServiceClient()

    # Use multi-threaded executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Goal-Feedback-Result Pattern

Actions are designed for long-running tasks that require feedback during execution and may be cancelable. This pattern is ideal for navigation, manipulation, or any task that takes a significant amount of time to complete.

### Key Characteristics of Actions

- **Asynchronous**: Non-blocking for the client
- **Goal-Feedback-Result**: Three-part communication pattern
- **Cancelable**: Tasks can be canceled before completion
- **Stateful**: Maintains state during execution

### Action Implementation Example

```python
import rclpy
import time
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Initialize Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Generate Fibonacci sequence up to goal order
        for i in range(1, goal_handle.request.order):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update feedback
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # Simulate work

        # Complete the goal
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # Send goal and get future
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)

    server = FibonacciActionServer()
    client = FibonacciActionClient()

    # Use multi-threaded executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)

    # Schedule goal after a delay
    def send_goal():
        client.send_goal(10)

    # Use a timer to send the goal after initialization
    timer = client.create_timer(2.0, send_goal)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Choosing the Right Communication Pattern

### When to Use Topics
- Streaming sensor data (camera images, laser scans, IMU data)
- Broadcasting status information
- Publishing control commands
- Any scenario where you want decoupled, asynchronous communication

### When to Use Services
- Querying current state or configuration
- Requesting specific computations
- Operations that should complete before continuing
- Any scenario requiring a clear request-response pattern

### When to Use Actions
- Navigation tasks (moving to a goal)
- Manipulation tasks (grasping objects)
- Calibration procedures
- Any long-running task that needs feedback or can be canceled

## Advanced Communication Concepts

### Message Types and Custom Messages

For humanoid robotics applications, you may need to define custom message types:

```bash
# Example custom message definition (in msg/ directory)
# RobotState.msg
float64[20] joint_positions
float64[20] joint_velocities
geometry_msgs/Pose base_pose
std_msgs/Header header
```

### Lifecycle Nodes and Communication

Lifecycle nodes provide state management for complex communication patterns:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher as LifecyclePublisher

class LifecycleCommunicationNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_communication_node')
        self.pub = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.pub = self.create_lifecycle_publisher(String, 'lifecycle_topic', 10)
        self.get_logger().info(f'Configured node in state {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.pub.on_activate()
        self.get_logger().info(f'Activated node in state {state.label}')
        return TransitionCallbackReturn.SUCCESS
```

## Summary

ROS 2 communication primitives provide flexible and powerful ways to coordinate activities in robotic systems. Topics offer asynchronous, decoupled communication ideal for streaming data. Services provide synchronous request-response patterns for operations requiring completion. Actions enable long-running tasks with feedback and cancellation capabilities.

Understanding when and how to use each communication pattern is crucial for building robust and efficient robotic applications. The choice of communication pattern affects system performance, reliability, and maintainability, making it a fundamental design decision in ROS 2 development.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the differences between topics, services, and actions in ROS 2. When would you use each communication pattern? Provide specific examples for humanoid robot applications.

2. Describe the Quality of Service (QoS) settings and their impact on communication reliability and performance. What QoS settings would you use for safety-critical messages vs. best-effort sensor data?

### Application Questions
3. Design a communication architecture for a humanoid robot that needs to:
   - Stream camera images for perception (high frequency)
   - Request object detection results (on-demand)
   - Execute a navigation task to move to a target location (long-running)
   Specify the communication patterns and QoS settings for each interaction.

4. What are the trade-offs between using RELIABLE vs BEST_EFFORT QoS policies for different types of data in a humanoid robot (e.g., sensor data, control commands, logging information)?

### Hands-on Practice
5. Create a publisher that sends sensor data with different QoS policies (reliable vs best effort) and observe the differences in behavior.

6. Implement a simple client-server interaction using ROS 2 services. Create a service that calculates the distance between two points and a client that calls this service with different coordinates.

### Critical Thinking
7. How would you design a communication system that can gracefully degrade when network conditions are poor? What patterns and QoS settings would you use?

8. What are the limitations of each communication pattern, and how might you combine them to create more robust robotic systems?
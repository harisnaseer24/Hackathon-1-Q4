#!/usr/bin/env python3

"""
ROS 2 Executor Example
This example demonstrates different types of executors in ROS 2
covering concepts from Chapter 1: Nodes, Executors, and DDS Communication
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from std_msgs.msg import String
import time


class PublisherNode(Node):
    """A simple publisher node for demonstration"""

    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.counter = 0

        # Create a timer that publishes every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Message from {self.get_name()}: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1


class SubscriberNode(Node):
    """A simple subscriber node for demonstration"""

    def __init__(self, node_name):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'{self.get_name()} received: {msg.data}')


def single_threaded_example():
    """Demonstrate single-threaded executor"""
    print("=== Single-Threaded Executor Example ===")

    rclpy.init()

    # Create nodes
    pub_node = PublisherNode('publisher_node')
    sub_node1 = SubscriberNode('subscriber_node_1')
    sub_node2 = SubscriberNode('subscriber_node_2')

    # Create single-threaded executor
    executor = SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node1)
    executor.add_node(sub_node2)

    try:
        print("Running single-threaded executor (Ctrl+C to stop)...")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub_node.destroy_node()
        sub_node1.destroy_node()
        sub_node2.destroy_node()
        rclpy.shutdown()


def multi_threaded_example():
    """Demonstrate multi-threaded executor"""
    print("=== Multi-Threaded Executor Example ===")

    rclpy.init()

    # Create nodes
    pub_node = PublisherNode('publisher_node_mt')
    sub_node1 = SubscriberNode('subscriber_node_1_mt')
    sub_node2 = SubscriberNode('subscriber_node_2_mt')

    # Create multi-threaded executor with 4 threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(pub_node)
    executor.add_node(sub_node1)
    executor.add_node(sub_node2)

    try:
        print("Running multi-threaded executor (Ctrl+C to stop)...")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub_node.destroy_node()
        sub_node1.destroy_node()
        sub_node2.destroy_node()
        rclpy.shutdown()


def main():
    """Main function to demonstrate executors"""
    print("ROS 2 Executor Examples")
    print("This example shows the difference between single-threaded and multi-threaded executors")
    print("")

    # Note: In a real scenario, you would run one or the other
    # For this example, we'll just show the code structure
    print("Single-threaded executor processes all callbacks in one thread")
    print("Multi-threaded executor uses a thread pool to process callbacks concurrently")
    print("")


if __name__ == '__main__':
    main()
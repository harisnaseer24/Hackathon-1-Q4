#!/usr/bin/env python3

"""
Publisher-Subscriber Example for Chapter 2
This example demonstrates the publish-subscribe communication pattern in ROS 2
covering concepts from Chapter 2: Sensor and Actuator Data Flow
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
import math
import time


class SensorPublisher(Node):
    """
    A publisher node that simulates sensor data publishing
    Demonstrates the publisher side of the publish-subscribe pattern
    """

    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers for different message types
        self.string_publisher = self.create_publisher(String, 'chatter', 10)
        self.counter_publisher = self.create_publisher(Int32, 'counter', 10)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Create a timer to publish messages periodically (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Sensor Publisher node initialized')

    def timer_callback(self):
        """Callback function that publishes messages at regular intervals"""
        # Publish string message
        string_msg = String()
        string_msg.data = f'Hello World: {self.counter}'
        self.string_publisher.publish(string_msg)

        # Publish counter message
        counter_msg = Int32()
        counter_msg.data = self.counter
        self.counter_publisher.publish(counter_msg)

        # Publish joint state message (simulating sensor data)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        # Simulate joint positions for a simple robot
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = [
            math.sin(self.counter * 0.1),
            math.cos(self.counter * 0.15),
            math.sin(self.counter * 0.2) * 0.5
        ]
        joint_msg.velocity = [
            0.1 * math.cos(self.counter * 0.1),
            -0.15 * math.sin(self.counter * 0.15),
            0.2 * math.cos(self.counter * 0.2) * 0.5
        ]
        joint_msg.effort = [0.0, 0.0, 0.0]

        self.joint_publisher.publish(joint_msg)

        self.get_logger().info(f'Published: counter={self.counter}, joint_positions={joint_msg.position}')
        self.counter += 1


class DataSubscriber(Node):
    """
    A subscriber node that receives and processes data
    Demonstrates the subscriber side of the publish-subscribe pattern
    """

    def __init__(self):
        super().__init__('data_subscriber')

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

        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info('Data Subscriber node initialized')

    def string_callback(self, msg):
        """Callback function for string messages"""
        self.get_logger().info(f'String received: {msg.data}')

    def counter_callback(self, msg):
        """Callback function for counter messages"""
        self.get_logger().info(f'Counter received: {msg.data}')

    def joint_callback(self, msg):
        """Callback function for joint state messages"""
        self.get_logger().info(f'Joint positions: {msg.position}')
        self.get_logger().info(f'Joint velocities: {msg.velocity}')


def main(args=None):
    """
    Main function to run the publisher-subscriber example
    """
    rclpy.init(args=args)

    # Create both publisher and subscriber nodes
    publisher = SensorPublisher()
    subscriber = DataSubscriber()

    # Create an executor to manage both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)

    try:
        print("Publisher-Subscriber example running. Check the logs for published/subscribed messages.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()
        print("Publisher-Subscriber example finished.")


if __name__ == '__main__':
    main()
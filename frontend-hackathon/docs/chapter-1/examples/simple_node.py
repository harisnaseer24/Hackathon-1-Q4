#!/usr/bin/env python3

"""
Simple ROS 2 Node Example
This example demonstrates the basic structure of a ROS 2 node
covering concepts from Chapter 1: ROS 2 as the Robotic Nervous System
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleROS2Node(Node):
    """
    A simple ROS 2 node that demonstrates:
    - Node creation and initialization
    - Publisher and subscriber setup
    - Timer-based callbacks
    - Basic logging
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('simple_ros2_node')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'input_chatter',
            self.listener_callback,
            10
        )

        # Create a timer to publish messages periodically
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Counter for published messages
        self.i = 0

        self.get_logger().info('Simple ROS 2 Node initialized')

    def listener_callback(self, msg):
        """Callback function for the subscription"""
        self.get_logger().info(f'I heard: {msg.data}')

    def timer_callback(self):
        """Callback function for the timer"""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1


def main(args=None):
    """
    Main function to run the ROS 2 node
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the node
    simple_node = SimpleROS2Node()

    try:
        # Start spinning to process callbacks
        rclpy.spin(simple_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up
        simple_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
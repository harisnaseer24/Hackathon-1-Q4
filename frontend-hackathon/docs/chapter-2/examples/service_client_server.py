#!/usr/bin/env python3

"""
Service Client-Server Example for Chapter 2
This example demonstrates the request-response communication pattern in ROS 2
covering concepts from Chapter 2: Sensor and Actuator Data Flow
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, SetBool
import time


class CalculatorService(Node):
    """
    A service server that performs calculations
    Demonstrates the service server side of the request-response pattern
    """

    def __init__(self):
        super().__init__('calculator_service')

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

        self.get_logger().info('Calculator Service node initialized')

    def add_two_ints_callback(self, request, response):
        """Callback function for the add service"""
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: a={request.a}, b={request.b} -> Response: sum={response.sum}')
        return response

    def set_bool_callback(self, request, response):
        """Callback function for the boolean service"""
        flag_value = request.data
        response.success = True
        response.message = f'Flag set to {flag_value}'
        self.get_logger().info(f'Set flag to: {flag_value}')
        return response


class ServiceClient(Node):
    """
    A service client that makes requests to services
    Demonstrates the service client side of the request-response pattern
    """

    def __init__(self):
        super().__init__('service_client')

        # Create service clients
        self.add_client = self.create_client(AddTwoInts, 'add_two_ints')
        self.bool_client = self.create_client(SetBool, 'set_flag')

        # Wait for services to be available
        self.get_logger().info('Waiting for services to be available...')
        while not self.add_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add service not available, waiting again...')

        while not self.bool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Bool service not available, waiting again...')

        self.get_logger().info('Services are available!')

        # Timer to call services periodically
        self.timer = self.create_timer(2.0, self.call_services)
        self.call_count = 0

    def call_services(self):
        """Call services at regular intervals"""
        # Call add service
        add_request = AddTwoInts.Request()
        add_request.a = 10 + self.call_count
        add_request.b = 5 + self.call_count

        self.get_logger().info(f'Calling add service with a={add_request.a}, b={add_request.b}')
        self.add_future = self.add_client.call_async(add_request)
        self.add_future.add_done_callback(self.add_callback)

        # Call bool service
        bool_request = SetBool.Request()
        bool_request.data = self.call_count % 2 == 0  # Alternate true/false

        self.get_logger().info(f'Calling bool service with data={bool_request.data}')
        self.bool_future = self.bool_client.call_async(bool_request)
        self.bool_future.add_done_callback(self.bool_callback)

        self.call_count += 1

    def add_callback(self, future):
        """Callback function when add service response is received"""
        try:
            result = future.result()
            self.get_logger().info(f'Add service result: {result.sum}')
        except Exception as e:
            self.get_logger().error(f'Add service call failed: {e}')

    def bool_callback(self, future):
        """Callback function when bool service response is received"""
        try:
            result = future.result()
            self.get_logger().info(f'Bool service result: success={result.success}, message={result.message}')
        except Exception as e:
            self.get_logger().error(f'Bool service call failed: {e}')


def main(args=None):
    """
    Main function to run the service client-server example
    """
    rclpy.init(args=args)

    # Create service server and client nodes
    server = CalculatorService()
    client = ServiceClient()

    # Create an executor to manage both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)

    try:
        print("Service Client-Server example running. Check the logs for service calls.")
        print("The client will call services every 2 seconds.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()
        print("Service Client-Server example finished.")


if __name__ == '__main__':
    main()
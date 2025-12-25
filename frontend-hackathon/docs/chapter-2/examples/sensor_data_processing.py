#!/usr/bin/env python3

"""
Sensor Data Processing Example for Chapter 2
This example demonstrates processing sensor data from multiple sources
covering concepts from Chapter 2: Sensor and Actuator Data Flow
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray, Header
import math
import numpy as np


class SensorFusionNode(Node):
    """
    A node that processes data from multiple sensors
    Demonstrates sensor data fusion and processing concepts
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Publishers for processed data
        self.balance_state_pub = self.create_publisher(Float64MultiArray, 'balance_state', 10)
        self.robot_twist_pub = self.create_publisher(Twist, 'robot_twist', 10)
        self.avoidance_cmd_pub = self.create_publisher(Twist, 'obstacle_avoidance_cmd', 10)

        # Subscriptions for different sensor types
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            10
        )

        # Timer for sensor processing (50Hz)
        self.process_timer = self.create_timer(0.02, self.process_sensors)

        # Store sensor data
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_orientation = None
        self.imu_angular_velocity = None
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_max = 0.0
        self.laser_angle_increment = 0.0

        # Robot state
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.robot_velocity = {'linear': 0.0, 'angular': 0.0}

        self.get_logger().info('Sensor Fusion node initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Process IMU messages"""
        self.imu_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.imu_angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def laser_scan_callback(self, msg):
        """Process laser scan messages"""
        self.laser_ranges = list(msg.ranges)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_max = msg.angle_max
        self.laser_angle_increment = msg.angle_increment

    def process_sensors(self):
        """Main sensor processing function"""
        # Calculate balance state based on sensor data
        balance_state = self.calculate_balance_state()

        # Calculate robot twist (velocity) if possible
        twist = self.estimate_twist()

        # Check for obstacles and generate avoidance commands
        avoidance_cmd = self.check_obstacles_and_avoid()

        # Publish processed data
        if balance_state:
            balance_msg = Float64MultiArray()
            balance_msg.data = balance_state
            self.balance_state_pub.publish(balance_msg)

        if twist:
            self.robot_twist_pub.publish(twist)

        if avoidance_cmd:
            self.avoidance_cmd_pub.publish(avoidance_cmd)

    def calculate_balance_state(self):
        """Calculate balance-related metrics from sensor data"""
        if not self.joint_positions or self.imu_orientation is None:
            return None

        # Calculate balance metrics based on IMU orientation
        if self.imu_orientation:
            # Convert quaternion to roll/pitch (simplified)
            w, x, y, z = self.imu_orientation
            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            pitch = math.asin(2.0 * (w * y - z * x))

            # Calculate center of mass approximation based on joint positions
            # This is a simplified example - real CoM calculation would be more complex
            joint_names = list(self.joint_positions.keys())
            if joint_names:
                avg_position = sum(self.joint_positions.values()) / len(self.joint_positions)
            else:
                avg_position = 0.0

            # Return balance state as array [roll, pitch, center_of_mass_approx, etc.]
            return [roll, pitch, avg_position, 0.0]
        else:
            return [0.0, 0.0, 0.0, 0.0]

    def estimate_twist(self):
        """Estimate robot velocity based on sensor data"""
        # This would typically use odometry, IMU integration, or other methods
        # For this example, we'll simulate based on joint velocities
        if self.joint_velocities:
            # Simplified estimation based on joint velocities
            # In reality, this would use forward kinematics and odometry
            linear_vel = 0.0
            angular_vel = 0.0

            # Example: if certain joints are moving, estimate motion
            if 'joint1' in self.joint_velocities:
                linear_vel = self.joint_velocities['joint1'] * 0.1  # Scale factor
            if 'joint2' in self.joint_velocities:
                angular_vel = self.joint_velocities['joint2'] * 0.05  # Scale factor

            twist = Twist()
            twist.linear.x = linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular_vel

            return twist
        else:
            twist = Twist()
            return twist

    def check_obstacles_and_avoid(self):
        """Check laser scan data for obstacles and generate avoidance commands"""
        if not self.laser_ranges:
            return None

        # Find minimum distance in front of robot (simplified forward sector)
        front_sector_start = int((len(self.laser_ranges) / 2) - (len(self.laser_ranges) / 8))  # Front-left
        front_sector_end = int((len(self.laser_ranges) / 2) + (len(self.laser_ranges) / 8))    # Front-right

        # Get minimum distance in front sector
        front_distances = [
            dist for dist in self.laser_ranges[front_sector_start:front_sector_end]
            if not (math.isnan(dist) or math.isinf(dist))
        ]

        if not front_distances:
            # If no valid readings, assume clear path
            avoidance_cmd = Twist()
            avoidance_cmd.linear.x = 0.2  # Move forward slowly
            avoidance_cmd.angular.z = 0.0
            return avoidance_cmd

        min_front_dist = min(front_distances) if front_distances else float('inf')

        # Generate avoidance commands based on obstacle distance
        avoidance_cmd = Twist()

        if min_front_dist < 0.5:  # Obstacle closer than 0.5m
            # Stop and turn away from obstacle
            avoidance_cmd.linear.x = 0.0
            avoidance_cmd.angular.z = 0.5  # Turn right (positive angular velocity)
        elif min_front_dist < 1.0:  # Obstacle between 0.5m and 1.0m
            # Slow down and prepare to turn
            avoidance_cmd.linear.x = 0.1  # Move slowly
            avoidance_cmd.angular.z = 0.2  # Gentle turn
        else:
            # Path is clear, move forward
            avoidance_cmd.linear.x = 0.3  # Move forward
            avoidance_cmd.angular.z = 0.0

        return avoidance_cmd


class MockSensorPublisher(Node):
    """
    A mock sensor publisher for demonstration purposes
    Simulates sensor data for the sensor fusion node to process
    """

    def __init__(self):
        super().__init__('mock_sensor_publisher')

        # Publishers for mock sensor data
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Timer for publishing mock data (100Hz)
        self.pub_timer = self.create_timer(0.01, self.publish_mock_data)
        self.time_counter = 0.0

        self.get_logger().info('Mock Sensor Publisher node initialized')

    def publish_mock_data(self):
        """Publish mock sensor data"""
        self.time_counter += 0.01

        # Publish mock joint states
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = [
            math.sin(self.time_counter),
            math.cos(self.time_counter * 1.5),
            math.sin(self.time_counter * 2.0) * 0.5
        ]
        joint_msg.velocity = [
            math.cos(self.time_counter),
            -1.5 * math.sin(self.time_counter * 1.5),
            2.0 * math.cos(self.time_counter * 2.0) * 0.5
        ]
        joint_msg.effort = [0.0, 0.0, 0.0]

        self.joint_pub.publish(joint_msg)

        # Publish mock IMU data
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate slight tilting
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.05 * math.sin(self.time_counter * 0.5)  # Slight pitch variation
        imu_msg.orientation.z = 0.02 * math.cos(self.time_counter * 0.3)  # Slight roll variation
        imu_msg.orientation.w = math.sqrt(max(0.0, 1.0 -
            imu_msg.orientation.x**2 -
            imu_msg.orientation.y**2 -
            imu_msg.orientation.z**2))

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.05 * 0.5 * math.cos(self.time_counter * 0.5)  # Derivative of pitch
        imu_msg.angular_velocity.z = -0.02 * 0.3 * math.sin(self.time_counter * 0.3)  # Derivative of roll

        self.imu_pub.publish(imu_msg)

        # Publish mock laser scan data
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = -math.pi / 2
        scan_msg.angle_max = math.pi / 2
        scan_msg.angle_increment = math.pi / 180  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Simulate ranges with some obstacles
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        ranges = []
        for i in range(num_ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Create a scenario with obstacles at certain angles
            if -0.5 < angle < 0.5:  # Front area has an obstacle
                distance = 0.8 + 0.2 * math.sin(self.time_counter * 2 + angle * 3)
            else:
                distance = 3.0 + 1.0 * math.sin(self.time_counter + angle)  # Clear areas elsewhere

            ranges.append(distance)

        scan_msg.ranges = ranges
        self.laser_pub.publish(scan_msg)


def main(args=None):
    """
    Main function to run the sensor data processing example
    """
    rclpy.init(args=args)

    # Create both sensor fusion node and mock publisher
    sensor_fusion = SensorFusionNode()
    mock_publisher = MockSensorPublisher()

    # Create an executor to manage both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor_fusion)
    executor.add_node(mock_publisher)

    try:
        print("Sensor Data Processing example running.")
        print("The mock publisher simulates sensor data, and the fusion node processes it.")
        print("Check the logs for processed sensor data.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        sensor_fusion.destroy_node()
        mock_publisher.destroy_node()
        rclpy.shutdown()
        print("Sensor Data Processing example finished.")


if __name__ == '__main__':
    main()
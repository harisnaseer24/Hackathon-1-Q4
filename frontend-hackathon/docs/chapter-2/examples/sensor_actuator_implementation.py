#!/usr/bin/env python3

"""
Sensor and Actuator Implementation Example for Chapter 2
This example demonstrates sensor and actuator implementations for humanoid robots
covering concepts from Chapter 2: Sensor and Actuator Data Flow
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math
import time


class SensorInterfaceNode(Node):
    """
    A node that simulates sensor interface implementation
    Demonstrates how to implement sensor drivers and data acquisition
    """

    def __init__(self):
        super().__init__('sensor_interface_node')

        # Publishers for different sensor types
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.laser_scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Timer for sensor data publishing (simulating hardware interface)
        self.sensor_timer = self.create_timer(0.01, self.publish_sensor_data)  # 100Hz

        # Simulated sensor data
        self.sim_time = 0.0
        self.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint', 'shoulder_joint', 'elbow_joint']

        self.get_logger().info('Sensor Interface node initialized')

    def publish_sensor_data(self):
        """Simulate publishing sensor data from hardware interface"""
        self.sim_time += 0.01  # Increment simulation time

        # Publish joint states (simulating encoder data)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        joint_msg.name = self.joint_names

        # Simulate joint positions with periodic motion
        joint_msg.position = [
            0.1 * math.sin(self.sim_time * 2),      # hip
            0.15 * math.sin(self.sim_time * 2.5),   # knee
            0.05 * math.sin(self.sim_time * 3),     # ankle
            0.2 * math.sin(self.sim_time * 1.5),    # shoulder
            0.1 * math.sin(self.sim_time * 1.8)     # elbow
        ]

        # Calculate velocities as derivatives of positions
        joint_msg.velocity = [
            0.1 * 2 * math.cos(self.sim_time * 2),      # hip velocity
            0.15 * 2.5 * math.cos(self.sim_time * 2.5), # knee velocity
            0.05 * 3 * math.cos(self.sim_time * 3),     # ankle velocity
            0.2 * 1.5 * math.cos(self.sim_time * 1.5),  # shoulder velocity
            0.1 * 1.8 * math.cos(self.sim_time * 1.8)   # elbow velocity
        ]

        # Zero efforts for simulation
        joint_msg.effort = [0.0] * len(self.joint_names)

        self.joint_state_pub.publish(joint_msg)

        # Publish IMU data (simulating inertial measurement unit)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate orientation (slight variations)
        roll = 0.01 * math.sin(self.sim_time * 0.5)
        pitch = 0.02 * math.sin(self.sim_time * 0.7)
        yaw = 0.005 * math.sin(self.sim_time * 0.3)

        # Convert Euler to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Simulate angular velocities
        imu_msg.angular_velocity.x = 0.01 * 0.5 * math.cos(self.sim_time * 0.5)  # Roll rate
        imu_msg.angular_velocity.y = 0.02 * 0.7 * math.cos(self.sim_time * 0.7)  # Pitch rate
        imu_msg.angular_velocity.z = 0.005 * 0.3 * math.cos(self.sim_time * 0.3)  # Yaw rate

        # Simulate linear accelerations
        imu_msg.linear_acceleration.x = -0.1 * math.sin(self.sim_time * 0.5)  # Due to body motion
        imu_msg.linear_acceleration.y = -0.1 * math.sin(self.sim_time * 0.7)
        imu_msg.linear_acceleration.z = 9.81 + 0.1 * math.sin(self.sim_time * 0.3)  # Gravity + motion

        self.imu_pub.publish(imu_msg)

        # Publish laser scan data (simulating LIDAR)
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = -math.pi / 2  # -90 degrees
        scan_msg.angle_max = math.pi / 2   # 90 degrees
        scan_msg.angle_increment = math.pi / 180  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Calculate number of range measurements
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1

        # Simulate range measurements with some obstacles
        ranges = []
        for i in range(num_ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            # Create a scenario: obstacle in front, clear on sides
            if -0.5 < angle < 0.5:  # Front area
                distance = 1.0 + 0.3 * math.sin(self.sim_time * 3 + angle * 2)  # Dynamic obstacle
            else:  # Side areas
                distance = 5.0 + 1.0 * math.sin(self.sim_time + angle)  # Farther distances

            ranges.append(distance)

        scan_msg.ranges = ranges
        self.laser_scan_pub.publish(scan_msg)

        self.get_logger().debug(f'Sensor data published: joints={len(joint_msg.position)}, IMU updated, scan={len(ranges)} ranges')


class ActuatorInterfaceNode(Node):
    """
    A node that simulates actuator interface implementation
    Demonstrates how to implement actuator command processing and hardware interface
    """

    def __init__(self):
        super().__init__('actuator_interface_node')

        # Subscriptions for actuator commands
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_cmd_callback,
            10
        )

        self.twist_cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_cmd_callback,
            10
        )

        # Publishers for actuator feedback
        self.actuator_status_pub = self.create_publisher(Float64MultiArray, 'actuator_status', 10)

        # Timer for actuator simulation (1kHz for realistic actuator response)
        self.actuator_timer = self.create_timer(0.001, self.actuator_simulation_step)

        # Current actuator states
        self.desired_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # 5 joints
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.actuator_enabled = [True, True, True, True, True]  # All actuators enabled initially

        # Actuator parameters for simulation
        self.actuator_response_time = 0.05  # 50ms response time
        self.max_velocity = 2.0  # rad/s
        self.max_effort = 100.0  # N*m

        self.get_logger().info('Actuator Interface node initialized')

    def joint_cmd_callback(self, msg):
        """Handle joint command messages"""
        # Update desired joint positions from command
        if len(msg.data) >= 5:
            self.desired_joint_positions = list(msg.data[:5])
        elif len(msg.data) == 3:  # For 3-joint examples
            self.desired_joint_positions[:3] = list(msg.data)
        else:
            self.get_logger().warn(f'Insufficient joint command data: {len(msg.data)} values, expected at least 3')

    def velocity_cmd_callback(self, msg):
        """Handle velocity command messages (could translate to joint commands)"""
        # In a real system, this might convert twist commands to joint commands
        # For this example, we'll just log the command
        self.get_logger().info(f'Velocity command received: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def actuator_simulation_step(self):
        """Simulate actuator behavior and update joint positions"""
        dt = 0.001  # 1ms simulation step

        # Update each joint position based on desired position
        for i in range(len(self.current_joint_positions)):
            if not self.actuator_enabled[i]:
                continue

            # Calculate the difference between desired and current position
            pos_diff = self.desired_joint_positions[i] - self.current_joint_positions[i]

            # Simulate actuator response with first-order lag
            # The actuator moves toward the desired position with a time constant
            max_step = self.max_velocity * dt  # Maximum change per time step
            pos_step = max(-max_step, min(max_step, pos_diff * dt / self.actuator_response_time))

            # Update position
            self.current_joint_positions[i] += pos_step

            # Calculate velocity (simple numerical differentiation)
            self.current_joint_velocities[i] = pos_step / dt

        # Publish actuator status
        status_msg = Float64MultiArray()
        # Pack current positions, velocities, and status into a flat array
        # Format: [pos0, vel0, enabled0, pos1, vel1, enabled1, ...]
        for i in range(len(self.current_joint_positions)):
            status_msg.data.extend([
                self.current_joint_positions[i],
                self.current_joint_velocities[i],
                1.0 if self.actuator_enabled[i] else 0.0
            ])

        self.actuator_status_pub.publish(status_msg)

        # Log for debugging
        if self.get_clock().now().nanoseconds / 1e9 % 1.0 < dt:  # Log every second
            self.get_logger().debug(f'Actuator status: positions={self.current_joint_positions[:3]}')

    def enable_actuator(self, joint_index, enable=True):
        """Enable or disable a specific actuator"""
        if 0 <= joint_index < len(self.actuator_enabled):
            self.actuator_enabled[joint_index] = enable
            status = "enabled" if enable else "disabled"
            self.get_logger().info(f'Actuator {joint_index} {status}')
        else:
            self.get_logger().error(f'Invalid joint index: {joint_index}')


def main(args=None):
    """
    Main function to run the sensor and actuator implementation example
    """
    rclpy.init(args=args)

    # Create sensor and actuator interface nodes
    sensor_interface = SensorInterfaceNode()
    actuator_interface = ActuatorInterfaceNode()

    # Create an executor to manage both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor_interface)
    executor.add_node(actuator_interface)

    try:
        print("Sensor and Actuator Implementation example running.")
        print("The sensor interface simulates hardware sensors publishing data.")
        print("The actuator interface simulates receiving commands and controlling actuators.")
        print("Check the logs for sensor publications and actuator responses.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        sensor_interface.destroy_node()
        actuator_interface.destroy_node()
        rclpy.shutdown()
        print("Sensor and Actuator Implementation example finished.")


if __name__ == '__main__':
    main()
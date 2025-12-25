#!/usr/bin/env python3

"""
Joint Control Example for Chapter 2
This example demonstrates joint control using PID control patterns
covering concepts from Chapter 2: Sensor and Actuator Data Flow
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math


class JointController(Node):
    """
    A joint controller implementing PID control for robot joints
    Demonstrates joint-level control patterns and actuator command generation
    """

    def __init__(self):
        super().__init__('joint_controller')

        # Publishers for joint commands
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.joint_trajectory_pub = self.create_publisher(Float64MultiArray, 'joint_trajectory_commands', 10)

        # Subscriptions for joint state feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # PID controller parameters
        self.kp = 50.0   # Proportional gain
        self.ki = 5.0    # Integral gain
        self.kd = 10.0   # Derivative gain

        # PID state variables for each joint
        self.integral_error = {}
        self.previous_error = {}
        self.previous_time = None

        # Desired positions (trajectory)
        self.desired_positions = {
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        # Current joint positions and velocities
        self.current_positions = {}
        self.current_velocities = {}

        # Trajectory generation parameters
        self.trajectory_time = 0.0
        self.trajectory_duration = 5.0  # 5 seconds for each trajectory segment

        # Control mode
        self.control_mode = "position"  # "position", "velocity", or "effort"

        self.get_logger().info('Joint Controller node initialized')

    def joint_state_callback(self, msg):
        """Update current joint state from feedback"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_velocities[name] = msg.velocity[i]

    def control_loop(self):
        """Main control loop implementing PID control"""
        current_time = time.time()
        if self.previous_time is None:
            self.previous_time = current_time
            return

        dt = current_time - self.previous_time
        self.previous_time = current_time

        # Update trajectory
        self.update_trajectory()

        # Calculate PID commands for each joint
        joint_commands = Float64MultiArray()

        for joint_name in ['hip_joint', 'knee_joint', 'ankle_joint']:
            current_pos = self.current_positions.get(joint_name, 0.0)
            desired_pos = self.desired_positions.get(joint_name, 0.0)

            # Calculate error
            error = desired_pos - current_pos

            # Update integral (with anti-windup)
            self.integral_error[joint_name] = self.integral_error.get(joint_name, 0.0) + error * dt
            # Limit integral to prevent windup
            max_integral = 10.0
            self.integral_error[joint_name] = max(-max_integral, min(max_integral, self.integral_error[joint_name]))

            # Calculate derivative
            if joint_name in self.previous_error:
                derivative = (error - self.previous_error[joint_name]) / dt
            else:
                derivative = 0.0

            # Store current error for next iteration
            self.previous_error[joint_name] = error

            # Calculate PID output
            if self.control_mode == "effort":
                # PID output as effort/torque
                output = (self.kp * error +
                         self.ki * self.integral_error[joint_name] +
                         self.kd * derivative)
            elif self.control_mode == "position":
                # For position control, we might want to calculate a desired position change
                # For this example, we'll use PID to calculate the next desired position
                pid_output = (self.kp * error +
                             self.ki * self.integral_error[joint_name] +
                             self.kd * derivative)
                # Limit the output to prevent large jumps
                max_output = 0.1  # Limit to 0.1 rad per control cycle
                pid_output = max(-max_output, min(max_output, pid_output))
                output = current_pos + pid_output
            else:
                # Default to effort control
                output = (self.kp * error +
                         self.ki * self.integral_error[joint_name] +
                         self.kd * derivative)

            joint_commands.data.append(output)

        # Publish joint commands based on control mode
        if self.control_mode == "effort":
            self.joint_cmd_pub.publish(joint_commands)
        else:
            self.joint_trajectory_pub.publish(joint_commands)

        # Log control performance
        if self.get_logger().level <= 10:  # DEBUG level
            self.get_logger().debug(f'Joint commands: {joint_commands.data}')

    def update_trajectory(self):
        """Update desired joint positions based on planned trajectory"""
        # Update trajectory time
        self.trajectory_time += 0.01  # dt = 0.01s

        # Generate a periodic trajectory (e.g., walking pattern)
        # This is a simple example - real trajectories would be more complex
        cycle_time = self.trajectory_time % self.trajectory_duration

        # Create a walking-like pattern for demonstration
        self.desired_positions['hip_joint'] = 0.1 * math.sin(cycle_time * 1.2)
        self.desired_positions['knee_joint'] = 0.15 * math.sin(cycle_time * 1.2 + math.pi/3)
        self.desired_positions['ankle_joint'] = 0.05 * math.sin(cycle_time * 1.2 + 2*math.pi/3)

    def set_control_mode(self, mode):
        """Set the control mode (position, velocity, or effort)"""
        if mode in ["position", "velocity", "effort"]:
            self.control_mode = mode
            self.get_logger().info(f'Control mode set to: {mode}')
        else:
            self.get_logger().error(f'Invalid control mode: {mode}. Use position, velocity, or effort')


class JointStateSimulator(Node):
    """
    A simulator for joint states to demonstrate the controller
    In a real robot, this would be replaced by actual hardware interface
    """

    def __init__(self):
        super().__init__('joint_state_simulator')

        # Publishers for simulated joint states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriptions for joint commands
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_cmd_callback,
            10
        )

        self.joint_traj_sub = self.create_subscription(
            Float64MultiArray,
            'joint_trajectory_commands',
            self.joint_traj_callback,
            10
        )

        # Timer for simulation (100Hz)
        self.sim_timer = self.create_timer(0.01, self.simulation_step)

        # Simulated joint state
        self.joint_positions = {'hip_joint': 0.0, 'knee_joint': 0.0, 'ankle_joint': 0.0}
        self.joint_velocities = {'hip_joint': 0.0, 'knee_joint': 0.0, 'ankle_joint': 0.0}
        self.joint_efforts = {'hip_joint': 0.0, 'knee_joint': 0.0, 'ankle_joint': 0.0}

        # Command inputs
        self.effort_commands = [0.0, 0.0, 0.0]
        self.position_commands = [0.0, 0.0, 0.0]

        # Simulation parameters
        self.joint_inertia = 0.1  # Simplified inertia for simulation
        self.joint_damping = 0.5  # Damping coefficient

        self.get_logger().info('Joint State Simulator node initialized')

    def joint_cmd_callback(self, msg):
        """Handle effort commands"""
        if len(msg.data) >= 3:
            self.effort_commands = list(msg.data[:3])

    def joint_traj_callback(self, msg):
        """Handle position trajectory commands"""
        if len(msg.data) >= 3:
            self.position_commands = list(msg.data[:3])

    def simulation_step(self):
        """Simulation step to update joint states based on commands"""
        dt = 0.01  # 100Hz

        # Update each joint based on commands
        joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']

        for i, joint_name in enumerate(joint_names):
            # Get desired position (from trajectory) and effort (from PID)
            desired_pos = self.position_commands[i] if i < len(self.position_commands) else 0.0
            effort_cmd = self.effort_commands[i] if i < len(self.effort_commands) else 0.0

            # Simple PD control to follow position commands
            current_pos = self.joint_positions[joint_name]
            current_vel = self.joint_velocities[joint_name]

            # Calculate position error
            pos_error = desired_pos - current_pos

            # PD control for position following
            kp = 100.0  # Position gain
            kv = 10.0   # Velocity gain (damping)

            effort_cmd += kp * pos_error - kv * current_vel

            # Update joint state using simple physics
            # F = ma, so acceleration = F/m (using inertia instead of mass)
            acceleration = (effort_cmd - self.joint_damping * current_vel) / self.joint_inertia

            # Update velocity and position using Euler integration
            new_velocity = current_vel + acceleration * dt
            new_position = current_pos + new_velocity * dt

            # Update the state
            self.joint_positions[joint_name] = new_position
            self.joint_velocities[joint_name] = new_velocity
            self.joint_efforts[joint_name] = effort_cmd

        # Publish updated joint state
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        joint_state_msg.name = joint_names
        joint_state_msg.position = [self.joint_positions[name] for name in joint_names]
        joint_state_msg.velocity = [self.joint_velocities[name] for name in joint_names]
        joint_state_msg.effort = [self.joint_efforts[name] for name in joint_names]

        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    """
    Main function to run the joint control example
    """
    rclpy.init(args=args)

    # Create controller and simulator nodes
    controller = JointController()
    simulator = JointStateSimulator()

    # Create an executor to manage both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(simulator)

    try:
        print("Joint Control example running.")
        print("The controller generates commands and the simulator processes them.")
        print("Check the logs for joint positions and control commands.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        controller.destroy_node()
        simulator.destroy_node()
        rclpy.shutdown()
        print("Joint Control example finished.")


if __name__ == '__main__':
    main()
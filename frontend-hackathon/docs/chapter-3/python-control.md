---
sidebar_position: 1
title: "Python-based Control using rclpy"
---

# Python-based Control using rclpy

## Introduction

Python has become one of the most popular languages for robotics development, particularly for AI and machine learning applications. The `rclpy` package provides Python bindings for ROS 2, enabling developers to create ROS 2 nodes, publishers, subscribers, services, and actions using Python. This chapter explores how to use `rclpy` for controlling humanoid robots, bridging AI algorithms with robotic systems.

The `rclpy` library offers a clean and intuitive API that closely mirrors the underlying ROS 2 C++ client library (`rclcpp`), while leveraging Python's simplicity and rich ecosystem of scientific computing libraries.

## Understanding rclpy Architecture

### Client Library vs. Middleware

`rclpy` serves as a client library that provides a Python interface to the ROS 2 middleware (typically DDS). It handles the complexities of serialization, communication, and node management, allowing developers to focus on application logic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonControlNode(Node):
    def __init__(self):
        super().__init__('python_control_node')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'python_chatter', 10)

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'input_python_chatter',
            self.listener_callback,
            10
        )

        # Create a timer to publish messages periodically
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.i = 0

        # Log initialization
        self.get_logger().info('Python Control Node initialized')

    def listener_callback(self, msg):
        """Callback function for the subscription"""
        self.get_logger().info(f'I heard: {msg.data}')

    def timer_callback(self):
        """Callback function for the timer"""
        msg = String()
        msg.data = f'Hello from Python: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    """Main function to run the Python control node"""
    rclpy.init(args=args)

    node = PythonControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Python Control Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Setting Up Python Control Environment

### Installation and Dependencies

Before developing with `rclpy`, ensure your Python environment is properly set up:

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash  # or your ROS 2 distribution

# Install additional Python packages for robotics
pip3 install numpy matplotlib pyquaternion transforms3d
```

### Virtual Environment Setup

It's recommended to use a virtual environment for Python robotics development:

```bash
# Create a virtual environment
python3 -m venv ros2_robotics_env

# Activate the environment
source ros2_robotics_env/bin/activate

# Install additional packages
pip3 install numpy scipy matplotlib pyquaternion transforms3d
```

## Core Concepts in rclpy

### Node Creation and Management

A node is the fundamental unit of computation in ROS 2. In `rclpy`, nodes are created by subclassing the `Node` class:

```python
import rclpy
from rclpy.node import Node

class HumanoidControllerNode(Node):
    def __init__(self):
        # Initialize the parent Node class with a node name
        super().__init__('humanoid_controller_node')

        # Initialize internal state
        self.initialized = False
        self.control_frequency = 100  # Hz

        # Log successful initialization
        self.get_logger().info(
            f'Humanoid Controller Node initialized with control frequency: {self.control_frequency}Hz'
        )

        # Additional initialization can be done here
        self.setup_control_parameters()

    def setup_control_parameters(self):
        """Initialize control parameters and configuration"""
        # Example: Set up PID gains
        self.pid_gains = {
            'hip_joint': {'kp': 100.0, 'ki': 10.0, 'kd': 5.0},
            'knee_joint': {'kp': 150.0, 'ki': 15.0, 'kd': 7.5},
            'ankle_joint': {'kp': 80.0, 'ki': 8.0, 'kd': 4.0}
        }

        # Example: Set up joint limits
        self.joint_limits = {
            'hip_joint': (-1.5, 1.5),
            'knee_joint': (0.0, 2.5),
            'ankle_joint': (-0.8, 0.8)
        }

        self.initialized = True
        self.get_logger().info('Control parameters initialized')
```

### Publishers and Subscriptions

Publishers and subscribers form the backbone of ROS 2 communication:

```python
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class CommunicationNode(Node):
    def __init__(self):
        super().__init__('communication_node')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )

        self.body_cmd_pub = self.create_publisher(
            Twist,
            'body_motion_commands',
            10
        )

        # Subscriptions
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

        # Internal state
        self.current_joint_positions = {}
        self.imu_orientation = None

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Process IMU messages"""
        self.imu_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

    def publish_joint_commands(self, commands):
        """Publish joint commands to the robot"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.joint_cmd_pub.publish(cmd_msg)

    def publish_body_motion(self, linear_x, angular_z):
        """Publish body motion commands"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.body_cmd_pub.publish(twist_msg)
```

### Services and Actions

Services and actions provide request-response and goal-oriented communication patterns:

```python
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci
import rclpy.action

class ServiceActionNode(Node):
    def __init__(self):
        super().__init__('service_action_node')

        # Service server
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # Action server
        self.action_server = rclpy.action.ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_fibonacci
        )

        # Action client
        self.action_client = rclpy.action.ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def add_two_ints_callback(self, request, response):
        """Service callback for adding two integers"""
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

    def execute_fibonacci(self, goal_handle):
        """Action server callback for Fibonacci sequence"""
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate work (in real implementation, this would be actual computation)
            from time import sleep
            sleep(0.1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result
```

## Advanced rclpy Patterns

### Async/Await Pattern

For more complex control scenarios, Python's async/await pattern can be used with rclpy:

```python
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

class AsyncControlNode(Node):
    def __init__(self):
        super().__init__('async_control_node')

        self.publisher = self.create_publisher(String, 'async_chatter', 10)
        self.subscription = self.create_subscription(
            String,
            'async_input',
            self.async_listener_callback,
            10
        )

        # Timer for async operations
        self.timer = self.create_timer(1.0, self.async_operation_timer)

        # Store futures for async operations
        self.pending_operations = []

    def async_listener_callback(self, msg):
        """Handle incoming messages asynchronously"""
        self.get_logger().info(f'Async handler received: {msg.data}')
        # Could trigger async operations here

    def async_operation_timer(self):
        """Timer callback that might initiate async operations"""
        self.get_logger().info('Initiating async operation')
        # In a real implementation, this might call async functions
```

### Lifecycle Nodes

For complex control systems, lifecycle nodes provide state management:

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher as LifecyclePublisher
from std_msgs.msg import String

class LifecycleControlNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_control_node')
        self.pub = None
        self.timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node - create publishers, subscribers, etc."""
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)
        self.get_logger().info(f'Configured node in state {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node - start timers, enable publishers"""
        self.pub.on_activate()
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info(f'Activated node in state {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node - stop timers, disable publishers"""
        self.timer.cancel()
        self.pub.on_deactivate()
        self.get_logger().info(f'Deactivated node in state {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up resources"""
        self.destroy_publisher(self.pub)
        self.get_logger().info(f'Cleaned up node in state {state.label}')
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Timer callback for active state"""
        msg = String()
        msg.data = 'Lifecycle node active!'
        self.pub.publish(msg)
```

## Integration with AI Libraries

### TensorFlow/PyTorch Integration

Python control allows easy integration with deep learning frameworks:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class AIPoweredController(Node):
    def __init__(self):
        super().__init__('ai_powered_controller')

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Subscriptions for sensor data
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for control commands
        self.control_pub = self.create_publisher(
            Float64MultiArray,
            'ai_control_commands',
            10
        )

        # Initialize AI model (placeholder)
        self.ai_model = self.load_ai_model()

        # State variables
        self.latest_image = None
        self.model_predictions = []

    def load_ai_model(self):
        """Load or initialize the AI model"""
        # In a real implementation, this would load a trained model
        # For example: return tf.keras.models.load_model('path/to/model')
        # Or: import torch and load a PyTorch model
        self.get_logger().info('AI model loaded')
        return None  # Placeholder

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image

            # Process image with AI model
            control_commands = self.process_with_ai(cv_image)

            # Publish control commands
            if control_commands is not None:
                cmd_msg = Float64MultiArray()
                cmd_msg.data = control_commands
                self.control_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_with_ai(self, image):
        """Process image with AI model and return control commands"""
        # In a real implementation, this would run inference
        # For example:
        # resized_img = cv2.resize(image, (224, 224))
        # normalized_img = resized_img / 255.0
        # prediction = self.ai_model.predict(np.expand_dims(normalized_img, axis=0))
        # return self.convert_prediction_to_commands(prediction)

        # Placeholder implementation
        height, width, _ = image.shape
        center_x, center_y = width // 2, height // 2

        # Simple example: move toward bright regions
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        moments = cv2.moments(gray)

        if moments["m00"] != 0:
            centroid_x = int(moments["m10"] / moments["m00"])
            centroid_y = int(moments["m01"] / moments["m00"])

            # Generate control commands based on centroid position
            dx = (centroid_x - center_x) / center_x  # Normalize to [-1, 1]
            dy = (centroid_y - center_y) / center_y  # Normalize to [-1, 1]

            return [dx * 0.1, dy * 0.1, 0.0]  # Linear and angular commands

        return [0.0, 0.0, 0.0]
```

### Scientific Computing Integration

Python's rich scientific computing ecosystem integrates seamlessly with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
import numpy as np
from scipy import signal
import math

class ScientificController(Node):
    def __init__(self):
        super().__init__('scientific_controller')

        # Subscriptions
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'balance_commands', 10)

        # Initialize state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_orientation = None
        self.imu_angular_velocity = None

        # Filtering parameters
        self.filter_b, self.filter_a = signal.butter(3, 0.1, btype='low')  # Low-pass filter
        self.filtered_signals = {}

        # Timing
        self.prev_time = None

    def joint_callback(self, msg):
        """Process joint state with scientific computing"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

        # Apply filtering to noisy signals
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Example: Smooth joint velocity estimation using filtering
        for joint_name, velocity in self.joint_velocities.items():
            if joint_name not in self.filtered_signals:
                self.filtered_signals[joint_name] = [velocity] * 10
            else:
                # Append new value and maintain window
                self.filtered_signals[joint_name].append(velocity)
                if len(self.filtered_signals[joint_name]) > 10:
                    self.filtered_signals[joint_name] = self.filtered_signals[joint_name][1:]

                # Apply digital filter
                filtered_values = signal.filtfilt(
                    self.filter_b, self.filter_a,
                    self.filtered_signals[joint_name]
                )
                smoothed_velocity = filtered_values[-1]

                # Use smoothed velocity for control
                self.compute_balance_correction(joint_name, smoothed_velocity)

    def imu_callback(self, msg):
        """Process IMU data with scientific computing"""
        self.imu_orientation = [
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ]
        self.imu_angular_velocity = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]

        # Convert quaternion to Euler angles for control
        roll, pitch, yaw = self.quaternion_to_euler(self.imu_orientation)

        # Calculate balance correction based on tilt
        balance_correction = self.calculate_balance_control(roll, pitch)

        # Publish balance commands
        cmd = Twist()
        cmd.linear.x = balance_correction[0]
        cmd.angular.z = balance_correction[1]
        self.cmd_pub.publish(cmd)

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        x, y, z, w = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def calculate_balance_control(self, roll, pitch):
        """Calculate balance control corrections using scientific methods"""
        # Simple PD controller for balance
        kp_roll = 10.0
        kd_roll = 1.0
        kp_pitch = 15.0
        kd_pitch = 1.5

        # Desired angles (typically 0 for upright position)
        desired_roll = 0.0
        desired_pitch = 0.0

        # Calculate errors
        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch

        # Calculate control outputs
        roll_correction = kp_roll * roll_error  # + kd_roll * roll_rate_error
        pitch_correction = kp_pitch * pitch_error  # + kd_pitch * pitch_rate_error

        # Combine corrections
        linear_correction = pitch_correction * 0.1  # Forward/backward
        angular_correction = roll_correction * 0.1  # Turning to balance

        return [linear_correction, angular_correction]
```

## Best Practices for Python Control

### Error Handling and Robustness

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import traceback

class RobustControllerNode(Node):
    def __init__(self):
        super().__init__('robust_controller_node')

        # Declare parameters with defaults
        self.declare_parameter('control_frequency', 100)
        self.declare_parameter('safety_timeout', 1.0)
        self.declare_parameter('max_joint_velocity', 5.0)

        # Get parameters with error handling
        try:
            self.control_frequency = self.get_parameter('control_frequency').value
            self.safety_timeout = self.get_parameter('safety_timeout').value
            self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Parameter not declared: {e}')
            # Use defaults
            self.control_frequency = 100
            self.safety_timeout = 1.0
            self.max_joint_velocity = 5.0

        # Set up QoS profiles for robust communication
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Initialize with error handling
        try:
            self.setup_publishers_subscribers()
            self.setup_timers()
            self.validate_initialization()
        except Exception as e:
            self.get_logger().fatal(f'Initialization failed: {e}')
            self.get_logger().fatal(f'Traceback: {traceback.format_exc()}')
            raise

    def setup_publishers_subscribers(self):
        """Set up publishers and subscribers with error handling"""
        try:
            # Create publishers
            self.status_pub = self.create_publisher(String, 'controller_status', self.qos_profile)

            # Create subscriptions
            self.joint_sub = self.create_subscription(
                JointState, 'joint_states', self.safe_joint_callback, self.qos_profile
            )

        except Exception as e:
            self.get_logger().error(f'Failed to setup communications: {e}')
            raise

    def safe_joint_callback(self, msg):
        """Safe callback with error handling"""
        try:
            # Validate message
            if not self.validate_joint_message(msg):
                self.get_logger().warning('Invalid joint message received')
                return

            # Process message
            self.process_joint_data(msg)

        except Exception as e:
            self.get_logger().error(f'Error in joint callback: {e}')
            self.get_logger().debug(f'Traceback: {traceback.format_exc()}')
            # Don't let callback crash the node

    def validate_joint_message(self, msg):
        """Validate joint message before processing"""
        if not msg.name or not msg.position:
            return False

        if len(msg.name) != len(msg.position):
            return False

        # Check for invalid values
        for pos in msg.position:
            if math.isnan(pos) or math.isinf(pos):
                return False

        return True

    def validate_initialization(self):
        """Validate that node is properly initialized"""
        if not hasattr(self, 'control_frequency'):
            raise RuntimeError('Control frequency not set')

        if self.control_frequency <= 0:
            raise ValueError(f'Invalid control frequency: {self.control_frequency}')
```

## Performance Considerations

When using Python for control applications, performance is critical:

1. **Use NumPy for mathematical operations**: NumPy operations are implemented in C and much faster than pure Python loops
2. **Minimize object allocation**: In control loops, avoid creating new objects unnecessarily
3. **Consider Cython for performance-critical sections**: For computationally intensive tasks, Cython can provide C-level performance
4. **Profile your code**: Use profiling tools to identify bottlenecks

## Summary

Python-based control using `rclpy` provides a powerful and flexible way to implement robotic control systems. The combination of ROS 2's distributed architecture with Python's rich ecosystem enables rapid development of sophisticated robotic applications. From simple publishers and subscribers to complex AI-powered controllers, `rclpy` provides the tools needed to create robust and maintainable robotic systems.

The integration with scientific computing libraries, machine learning frameworks, and the broader Python ecosystem makes it an excellent choice for AI-powered humanoid robots that need to bridge the gap between high-level decision making and low-level control.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the architecture of rclpy and how it interfaces with the ROS 2 middleware. What are the advantages and disadvantages of using Python vs C++ for robotic control?

2. Describe the different communication patterns available in rclpy (topics, services, actions) and when you would use each pattern in a humanoid robot control system.

### Application Questions
3. Design a Python control node for a humanoid robot that:
   - Subscribes to IMU and joint encoder data (200Hz)
   - Implements a balance controller using PID control
   - Publishes joint commands to maintain balance
   - Handles sensor failures gracefully
   Include the complete rclpy implementation with proper error handling.

4. How would you implement a Python node that integrates a deep learning model for perception with ROS 2 control? What are the timing considerations and best practices?

### Hands-on Practice
5. Create a Python control node that subscribes to camera images, performs simple object detection, and publishes velocity commands to move toward the detected object.

6. Implement a lifecycle node in Python that manages the activation and deactivation of a humanoid robot's control systems.

### Critical Thinking
7. What are the performance limitations of using Python for real-time control in humanoid robots? How can you mitigate these limitations?

8. How would you design a Python-based control system that can gracefully handle network interruptions and maintain robot safety?
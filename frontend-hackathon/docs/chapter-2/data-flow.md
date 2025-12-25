---
sidebar_position: 2
title: "Sensor and Actuator Data Flow"
---

# Sensor and Actuator Data Flow

## Introduction

In humanoid robotics, the flow of data between sensors, processing nodes, and actuators forms the backbone of robot operation. Understanding how to design and implement efficient data flows is crucial for creating responsive and capable humanoid robots. This chapter explores the patterns and best practices for managing sensor data acquisition, processing, and actuator command generation in ROS 2 systems.

## Sensor Data Acquisition Pipeline

### Sensor Drivers and Hardware Interfaces

The sensor data pipeline typically begins with sensor drivers that interface with physical hardware. In ROS 2, these drivers publish sensor data to standardized topics using appropriate message types.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import Header
import math
import time

class SensorDriverNode(Node):
    def __init__(self):
        super().__init__('sensor_driver_node')

        # Publishers for different sensor types
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.laser_scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Timer for sensor data publishing
        self.sensor_timer = self.create_timer(0.01, self.publish_sensor_data)  # 100Hz

    def publish_sensor_data(self):
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        # Simulate joint positions (in a real robot, these would come from encoders)
        joint_msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        joint_msg.position = [math.sin(time.time()), math.cos(time.time()), 0.1]
        joint_msg.velocity = [math.cos(time.time()), -math.sin(time.time()), 0.0]
        joint_msg.effort = [0.0, 0.0, 0.0]

        self.joint_state_pub.publish(joint_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate IMU readings
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(time.time() * 0.1)
        imu_msg.orientation.w = math.cos(time.time() * 0.1)

        self.imu_pub.publish(imu_msg)

        # Publish laser scan data
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'

        # Laser scan parameters
        scan_msg.angle_min = -math.pi / 2
        scan_msg.angle_max = math.pi / 2
        scan_msg.angle_increment = math.pi / 180  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0

        # Simulate ranges (in a real robot, these would come from the laser scanner)
        num_ranges = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
        scan_msg.ranges = [5.0 + 2.0 * math.sin(i * 0.1 + time.time()) for i in range(num_ranges)]

        self.laser_scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_driver = SensorDriverNode()

    try:
        rclpy.spin(sensor_driver)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Sensor Data Processing Nodes

Once sensor data is published, processing nodes typically subscribe to these topics and perform various computations:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__('sensor_processing_node')

        # Subscriptions for sensor data
        self.joint_sub = self.create_subscription(
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

        # Publishers for processed data
        self.balance_state_pub = self.create_publisher(Float64MultiArray, 'balance_state', 10)
        self.twist_pub = self.create_publisher(Twist, 'robot_twist', 10)

        # Store sensor data
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_orientation = None
        self.imu_angular_velocity = None

        # Timer for processing
        self.process_timer = self.create_timer(0.02, self.process_sensors)  # 50Hz

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

    def process_sensors(self):
        """Main sensor processing function"""
        if not self.joint_positions or self.imu_orientation is None:
            return

        # Calculate balance state based on joint positions and IMU data
        balance_state = self.calculate_balance_state()

        # Publish balance state
        balance_msg = Float64MultiArray()
        balance_msg.data = balance_state
        self.balance_state_pub.publish(balance_msg)

        # Calculate robot twist (velocity) if possible
        twist = self.estimate_twist()
        if twist:
            self.twist_pub.publish(twist)

    def calculate_balance_state(self):
        """Calculate balance-related metrics"""
        # This is a simplified example - real balance calculations would be more complex
        # Calculate center of mass approximation based on joint positions
        # Calculate stability metrics based on IMU orientation

        # Example: simple balance metric based on IMU roll/pitch
        if self.imu_orientation:
            # Convert quaternion to roll/pitch/yaw approximation
            # In a real implementation, you'd use proper quaternion to euler conversion
            roll = np.arctan2(2.0 * (self.imu_orientation[3] * self.imu_orientation[0] +
                                    self.imu_orientation[1] * self.imu_orientation[2]),
                             1.0 - 2.0 * (self.imu_orientation[0]**2 + self.imu_orientation[1]**2))

            pitch = np.arcsin(2.0 * (self.imu_orientation[3] * self.imu_orientation[1] -
                                    self.imu_orientation[2] * self.imu_orientation[0]))
        else:
            roll = pitch = 0.0

        # Return balance state as array [roll, pitch, center_of_mass_x, center_of_mass_y, etc.]
        return [roll, pitch, 0.0, 0.0]  # Simplified

    def estimate_twist(self):
        """Estimate robot velocity based on sensor data"""
        # This would typically use odometry, IMU integration, or visual odometry
        # For this example, we'll create a mock implementation

        twist = Twist()
        twist.linear.x = 0.1  # m/s
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.05  # rad/s (turning slowly)

        return twist

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessingNode()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actuator Command Pipeline

### Command Generation

Commands to actuators typically originate from higher-level controllers or planners:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class CommandGeneratorNode(Node):
    def __init__(self):
        super().__init__('command_generator_node')

        # Publisher for joint trajectory commands
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_commands',
            10
        )

        # Publisher for direct joint commands (alternative approach)
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )

        # Timer for command generation
        self.command_timer = self.create_timer(0.1, self.generate_commands)  # 10Hz

    def generate_commands(self):
        """Generate actuator commands based on desired behavior"""
        # Example: Generate a simple walking pattern
        self.publish_trajectory_command()
        self.publish_direct_command()

    def publish_trajectory_command(self):
        """Publish joint trajectory command"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']

        point = JointTrajectoryPoint()

        # Generate sinusoidal trajectory for walking
        t = self.get_clock().now().nanoseconds / 1e9  # time in seconds

        point.positions = [
            0.2 * math.sin(t),      # hip
            0.3 * math.sin(t + 0.5), # knee
            0.1 * math.sin(t + 1.0)  # ankle
        ]

        point.velocities = [
            0.2 * math.cos(t),      # hip velocity
            0.3 * math.cos(t + 0.5), # knee velocity
            0.1 * math.cos(t + 1.0)  # ankle velocity
        ]

        point.accelerations = [
            -0.2 * math.sin(t),      # hip acceleration
            -0.3 * math.sin(t + 0.5), # knee acceleration
            -0.1 * math.sin(t + 1.0)  # ankle acceleration
        ]

        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds

        traj_msg.points = [point]
        self.joint_traj_pub.publish(traj_msg)

    def publish_direct_command(self):
        """Publish direct joint commands"""
        cmd_msg = Float64MultiArray()
        t = self.get_clock().now().nanoseconds / 1e9  # time in seconds

        cmd_msg.data = [
            0.1 * math.sin(t * 2),    # hip position
            0.15 * math.sin(t * 2 + 0.3),  # knee position
            0.05 * math.sin(t * 2 + 0.6)   # ankle position
        ]

        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    generator = CommandGeneratorNode()

    try:
        rclpy.spin(generator)
    except KeyboardInterrupt:
        pass
    finally:
        generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Hardware Interface Layer

The hardware interface layer translates high-level commands to low-level actuator commands:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import time

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')

        # Subscription for trajectory commands
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory_commands',
            self.trajectory_callback,
            10
        )

        # Publisher for joint state feedback
        self.state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            'joint_controller_state',
            10
        )

        # Timer for hardware interface loop
        self.hw_timer = self.create_timer(0.001, self.hardware_interface_loop)  # 1kHz

        # Store current state
        self.current_positions = {}
        self.current_velocities = {}
        self.current_efforts = {}
        self.desired_positions = {}
        self.desired_velocities = {}
        self.desired_efforts = {}

        # Initialize with default values
        self.joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']
        for joint in self.joint_names:
            self.current_positions[joint] = 0.0
            self.current_velocities[joint] = 0.0
            self.current_efforts[joint] = 0.0
            self.desired_positions[joint] = 0.0

    def trajectory_callback(self, msg):
        """Handle incoming trajectory commands"""
        if msg.points:
            # For simplicity, just take the first point
            # In a real implementation, you'd handle trajectory execution
            point = msg.points[0]

            for i, joint_name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    self.desired_positions[joint_name] = point.positions[i]
                if i < len(point.velocities):
                    self.desired_velocities[joint_name] = point.velocities[i]
                if i < len(point.effort):
                    self.desired_efforts[joint_name] = point.effort[i]

    def hardware_interface_loop(self):
        """Main hardware interface loop - runs at high frequency"""
        # Simulate sending commands to hardware
        # In a real robot, this would interface with actual hardware

        # Update current positions toward desired positions (simulating actuator response)
        for joint in self.joint_names:
            if joint in self.desired_positions:
                # Simple first-order response model
                current_pos = self.current_positions[joint]
                desired_pos = self.desired_positions[joint]

                # Simulate actuator response with some delay
                self.current_positions[joint] += 0.1 * (desired_pos - current_pos)

                # Calculate velocity (simple numerical derivative)
                self.current_velocities[joint] = (
                    self.current_positions[joint] - current_pos
                ) / 0.001  # dt = 0.001s

        # Publish current state
        state_msg = JointTrajectoryControllerState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'base_link'

        state_msg.joint_names = self.joint_names

        # Set feedback positions, velocities, and efforts
        state_msg.feedback.positions = [self.current_positions[joint] for joint in self.joint_names]
        state_msg.feedback.velocities = [self.current_velocities[joint] for joint in self.joint_names]
        state_msg.feedback.accelerations = [0.0] * len(self.joint_names)  # Simplified

        # Set desired positions, velocities, and efforts
        state_msg.desired.positions = [self.desired_positions.get(joint, 0.0) for joint in self.joint_names]
        state_msg.desired.velocities = [self.desired_velocities.get(joint, 0.0) for joint in self.joint_names]
        state_msg.desired.accelerations = [0.0] * len(self.joint_names)  # Simplified

        # Set error positions, velocities, and efforts
        state_msg.error.positions = [
            self.desired_positions.get(joint, 0.0) - self.current_positions[joint]
            for joint in self.joint_names
        ]
        state_msg.error.velocities = [
            self.desired_velocities.get(joint, 0.0) - self.current_velocities[joint]
            for joint in self.joint_names
        ]
        state_msg.error.accelerations = [0.0] * len(self.joint_names)  # Simplified

        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    hw_interface = HardwareInterfaceNode()

    try:
        rclpy.spin(hw_interface)
    except KeyboardInterrupt:
        pass
    finally:
        hw_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Data Flow Patterns in Humanoid Robotics

### Perception-Action Loop

The perception-action loop is fundamental to humanoid robotics:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import cv2
from cv2 import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PerceptionActionLoopNode(Node):
    def __init__(self):
        super().__init__('perception_action_loop_node')

        # Initialize CV bridge for image processing
        self.bridge = CvBridge()

        # Subscriptions for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for the main perception-action loop
        self.loop_timer = self.create_timer(0.05, self.perception_action_loop)  # 20Hz

        # State variables
        self.current_joint_positions = {}
        self.current_image = None
        self.target_detected = False
        self.target_position = None

    def joint_state_callback(self, msg):
        """Process joint state data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def camera_callback(self, msg):
        """Process camera image data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def perception_action_loop(self):
        """Main perception-action loop"""
        if self.current_image is not None:
            # Process image to detect targets or obstacles
            self.target_detected, self.target_position = self.process_image(self.current_image)

        # Make decisions based on perception results
        cmd_vel = self.make_decision()

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

    def process_image(self, image):
        """Process image to detect objects"""
        # Simple example: detect red objects
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Define range for red color
        lower_red = (0, 50, 50)
        upper_red = (10, 255, 255)
        mask1 = cv.inRange(hsv, lower_red, upper_red)

        lower_red = (170, 50, 50)
        upper_red = (180, 255, 255)
        mask2 = cv.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv.contourArea)
            if cv.contourArea(largest_contour) > 100:  # Minimum area threshold
                # Calculate centroid
                M = cv.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    image_height, image_width = image.shape[:2]
                    # Convert to normalized coordinates (-1 to 1)
                    norm_x = (cx - image_width/2) / (image_width/2)
                    norm_y = (cy - image_height/2) / (image_height/2)
                    return True, (norm_x, norm_y)

        return False, None

    def make_decision(self):
        """Make decisions based on sensor data"""
        cmd_vel = Twist()

        if self.target_detected and self.target_position:
            # Move towards the target
            target_x, target_y = self.target_position

            # Proportional control to move towards target
            cmd_vel.linear.x = max(0.0, min(0.5, 0.2 * (1.0 - abs(target_y))))  # Move forward based on vertical position
            cmd_vel.angular.z = -0.5 * target_x  # Turn to center the target
        else:
            # No target detected, stop or search
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    perception_action_node = PerceptionActionLoopNode()

    try:
        rclpy.spin(perception_action_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_action_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Data Flow Optimization

### Message Filtering and Throttling

To optimize data flow, it's important to filter and throttle messages appropriately:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from message_filters import Subscriber, TimeSynchronizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class OptimizedDataFlowNode(Node):
    def __init__(self):
        super().__init__('optimized_data_flow_node')

        # Create QoS profiles for different data types
        # High frequency sensor data with best effort
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Low frequency processed data with reliable delivery
        processed_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions with appropriate QoS
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            sensor_qos
        )

        # Publisher for processed data
        self.processed_pub = self.create_publisher(
            Float64MultiArray,
            'processed_data',
            processed_qos
        )

        # Timer for processing with variable rate
        self.process_timer = self.create_timer(0.05, self.process_data)  # 20Hz

        # Throttle parameters
        self.last_process_time = self.get_clock().now()
        self.process_interval = 0.05  # 20Hz

        # Store data for processing
        self.joint_data_buffer = []

    def joint_callback(self, msg):
        """Receive joint state data"""
        # Store data in buffer
        self.joint_data_buffer.append({
            'timestamp': self.get_clock().now(),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
            'names': list(msg.name)
        })

        # Limit buffer size to prevent memory issues
        if len(self.joint_data_buffer) > 100:
            self.joint_data_buffer.pop(0)

    def process_data(self):
        """Process buffered data"""
        if not self.joint_data_buffer:
            return

        # Process the most recent data
        latest_data = self.joint_data_buffer[-1]

        # Perform processing (example: calculate average position)
        if latest_data['positions']:
            avg_position = sum(latest_data['positions']) / len(latest_data['positions'])

            # Publish processed result
            result_msg = Float64MultiArray()
            result_msg.data = [avg_position, len(latest_data['positions'])]
            self.processed_pub.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    optimized_node = OptimizedDataFlowNode()

    try:
        rclpy.spin(optimized_node)
    except KeyboardInterrupt:
        pass
    finally:
        optimized_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Data Flow Design

### 1. Appropriate Message Rates
- High-frequency sensors (IMU, encoders): 100-1000Hz
- Medium-frequency sensors (cameras): 10-30Hz
- Low-frequency data (planning results): 1-5Hz

### 2. QoS Policy Selection
- Critical control data: RELIABLE, DURABLE
- High-frequency sensor data: BEST_EFFORT, VOLATILE
- Configuration data: RELIABLE, TRANSIENT_LOCAL

### 3. Data Synchronization
- Use message filters for time-sensitive multi-sensor fusion
- Implement proper timestamp handling
- Consider sensor calibration and delay compensation

### 4. Error Handling
- Implement fallback behaviors when sensor data is unavailable
- Use diagnostic messages to monitor data flow health
- Design graceful degradation strategies

## Summary

Designing efficient sensor and actuator data flows is critical for humanoid robot performance. The pipeline typically follows a pattern from sensor drivers through processing nodes to actuator commands. Each stage should be optimized for the specific requirements of the data being processed, considering factors like frequency, reliability, and computational requirements.

Understanding these patterns and implementing appropriate QoS settings, message filtering, and synchronization mechanisms will result in more robust and responsive humanoid robot systems.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the difference between sensor data acquisition, processing, and actuator command generation. How do these stages interact in a humanoid robot?

2. Describe the appropriate QoS policies for different types of sensor data in a humanoid robot (e.g., IMU data, camera images, joint positions). Justify your choices.

### Application Questions
3. Design a data flow architecture for a humanoid robot that needs to:
   - Process camera data for object detection (30Hz)
   - Integrate IMU and joint encoder data for balance control (200Hz)
   - Execute planned movements from a high-level planner (10Hz)
   Specify the message types, QoS settings, and processing nodes needed.

4. How would you handle sensor data synchronization when different sensors have different update rates and communication latencies?

### Hands-on Practice
5. Implement a sensor fusion node that combines IMU and joint encoder data to estimate the robot's center of mass position.

6. Create a simple data flow that takes camera images, detects a colored object, and generates velocity commands to move toward the object.

### Critical Thinking
7. What are the challenges of managing data flow in a multi-robot system? How would you modify the patterns described in this chapter?

8. How would you design a data flow system that can handle sensor failures gracefully while maintaining robot stability?
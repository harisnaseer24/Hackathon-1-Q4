---
sidebar_position: 2
title: "Bridging AI Agents to ROS Controllers"
---

# Bridging AI Agents to ROS Controllers

## Introduction

The integration of artificial intelligence with robotic systems represents a significant paradigm shift in robotics, where cognitive capabilities are combined with physical embodiment. This chapter explores the patterns and techniques for bridging AI agents to ROS controllers, enabling intelligent decision-making to drive physical robot behavior. The bridge between high-level AI reasoning and low-level robot control is critical for creating autonomous humanoid robots capable of complex, adaptive behaviors.

## Understanding the AI-ROS Integration Architecture

### The Cognitive-Physical Loop

The integration of AI agents with ROS controllers creates a cognitive-physical loop that encompasses:

1. **Perception**: Gathering information from the physical world through sensors
2. **Reasoning**: Processing information and making decisions using AI algorithms
3. **Action**: Executing decisions through robot control systems
4. **Feedback**: Observing the results of actions and adjusting behavior

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ai_reasoning_module import ReasoningEngine  # Hypothetical AI module
import numpy as np

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Initialize AI reasoning engine
        self.reasoning_engine = ReasoningEngine()

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.action_pub = self.create_publisher(String, 'high_level_actions', 10)

        # Subscriptions for sensor data
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10
        )

        # Timer for reasoning cycle
        self.reasoning_timer = self.create_timer(0.1, self.reasoning_cycle)  # 10Hz

        # State variables
        self.latest_image = None
        self.latest_scan = None
        self.latest_joints = None
        self.current_behavior = "idle"

    def image_callback(self, msg):
        """Process camera data for AI perception"""
        # Convert ROS Image to format suitable for AI processing
        image_data = self.convert_ros_image_to_array(msg)
        self.latest_image = image_data
        self.reasoning_engine.update_perceptual_state('camera', image_data)

    def laser_callback(self, msg):
        """Process LIDAR data for AI perception"""
        # Process laser scan data
        scan_data = np.array(msg.ranges)
        self.latest_scan = scan_data
        self.reasoning_engine.update_perceptual_state('lidar', scan_data)

    def joint_callback(self, msg):
        """Process joint state data for AI perception"""
        # Extract joint positions and velocities
        joint_dict = dict(zip(msg.name, msg.position))
        self.latest_joints = joint_dict
        self.reasoning_engine.update_perceptual_state('joints', joint_dict)

    def reasoning_cycle(self):
        """Main AI reasoning cycle"""
        # Update AI world model with latest sensor data
        self.reasoning_engine.update_world_model()

        # Perform reasoning and decision making
        decision = self.reasoning_engine.make_decision()

        # Execute decision through ROS controllers
        self.execute_decision(decision)

    def execute_decision(self, decision):
        """Execute AI decision through ROS control interface"""
        if decision['action'] == 'move_forward':
            cmd = Twist()
            cmd.linear.x = 0.5  # Move forward at 0.5 m/s
            self.cmd_vel_pub.publish(cmd)
        elif decision['action'] == 'turn_left':
            cmd = Twist()
            cmd.angular.z = 0.5  # Turn left at 0.5 rad/s
            self.cmd_vel_pub.publish(cmd)
        elif decision['action'] == 'grasp_object':
            # Publish high-level action for lower-level controller
            action_msg = String()
            action_msg.data = f"grasp:{decision['object_id']}"
            self.action_pub.publish(action_msg)

    def convert_ros_image_to_array(self, img_msg):
        """Convert ROS Image message to numpy array"""
        # This would use cv_bridge in a real implementation
        # For this example, returning a dummy array
        return np.random.rand(img_msg.height, img_msg.width, 3)
```

### Layered Integration Architecture

The AI-ROS bridge typically employs a layered architecture:

1. **Behavior Layer**: High-level AI agents that make strategic decisions
2. **Planning Layer**: Motion and path planning based on AI goals
3. **Control Layer**: Low-level controllers that execute planned motions
4. **Hardware Interface Layer**: Direct communication with robot hardware

## AI Agent Integration Patterns

### Reactive AI Integration

Reactive agents respond directly to environmental stimuli without maintaining complex internal states:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ReactiveAINode(Node):
    def __init__(self):
        super().__init__('reactive_ai_node')

        # Publishers and subscriptions
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Reactive parameters
        self.safety_distance = 0.5  # meters
        self.forward_speed = 0.3    # m/s
        self.turn_speed = 0.5       # rad/s

    def scan_callback(self, msg):
        """Process laser scan and react immediately"""
        # Find closest obstacle in front
        front_ranges = msg.ranges[len(msg.ranges)//2-30:len(msg.ranges)//2+30]
        min_distance = min([r for r in front_ranges if not np.isnan(r) and not np.isinf(r)], default=float('inf'))

        # Generate reactive response
        cmd = Twist()
        if min_distance < self.safety_distance:
            # Too close - turn away
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        else:
            # Safe to move forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
```

### Deliberative AI Integration

Deliberative agents maintain world models and plan ahead:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import heapq
from typing import List, Tuple

class DeliberativeAINode(Node):
    def __init__(self):
        super().__init__('deliberative_ai_node')

        # Publishers and subscriptions
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)

        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Timer for planning cycle
        self.planning_timer = self.create_timer(1.0, self.planning_cycle)

        # World model
        self.world_map = None
        self.robot_pose = None
        self.goals = []
        self.current_plan = []

    def image_callback(self, msg):
        """Process image data for object detection and mapping"""
        # In a real implementation, this would run object detection
        # For this example, we'll just update the map with detected objects
        detected_objects = self.detect_objects(msg)
        self.update_world_map(detected_objects)

    def scan_callback(self, msg):
        """Process LIDAR data for mapping"""
        # Update occupancy grid based on laser scan
        if self.world_map is None:
            self.initialize_map(msg)

        self.update_local_map(msg)

    def detect_objects(self, image_msg):
        """Detect objects in the image (placeholder implementation)"""
        # In a real implementation, this would use computer vision or ML
        # For this example, returning dummy objects
        return [
            {'type': 'person', 'distance': 2.5, 'angle': 0.2},
            {'type': 'obstacle', 'distance': 1.0, 'angle': -0.1}
        ]

    def update_world_map(self, detected_objects):
        """Update world map with detected objects"""
        # This would update the internal representation of the world
        # based on sensor data and object detection results
        pass

    def initialize_map(self, scan_msg):
        """Initialize occupancy grid map"""
        resolution = 0.1  # 10cm resolution
        width = 200  # 20m x 20m map
        height = 200

        self.world_map = OccupancyGrid()
        self.world_map.info.resolution = resolution
        self.world_map.info.width = width
        self.world_map.info.height = height
        self.world_map.info.origin.position.x = -10.0
        self.world_map.info.origin.position.y = -10.0
        self.world_map.data = [-1] * (width * height)  # Unknown initially

    def update_local_map(self, scan_msg):
        """Update local map with laser scan data"""
        # Convert laser scan to occupancy grid updates
        # This is a simplified implementation
        pass

    def planning_cycle(self):
        """Main planning cycle"""
        if not self.goals or not self.world_map:
            return

        # Select next goal
        next_goal = self.goals[0]

        # Plan path to goal using A* or other algorithm
        path = self.plan_path_to_goal(next_goal)

        if path:
            # Execute first step of the plan
            self.follow_path(path)

    def plan_path_to_goal(self, goal):
        """Plan path to goal using A* algorithm"""
        # Simplified A* implementation
        if not self.world_map or not self.robot_pose:
            return None

        # Convert goal and robot pose to grid coordinates
        start = self.pose_to_grid_coords(self.robot_pose)
        goal_coords = self.pose_to_grid_coords(goal)

        # Run A* pathfinding
        path = self.astar_search(start, goal_coords)

        return path

    def astar_search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """A* pathfinding algorithm"""
        # Simplified implementation
        # In a real implementation, this would check occupancy grid for obstacles
        path = [start]

        # For this example, just return a straight line
        # Real implementation would use proper A* with heuristic
        current = start
        while current != goal:
            # Move towards goal
            dx = 1 if goal[0] > current[0] else (-1 if goal[0] < current[0] else 0)
            dy = 1 if goal[1] > current[1] else (-1 if goal[1] < current[1] else 0)

            current = (current[0] + dx, current[1] + dy)
            path.append(current)

            if len(path) > 1000:  # Safety limit
                break

        return path

    def follow_path(self, path):
        """Follow the planned path"""
        if len(path) < 2:
            return

        # Calculate direction to next waypoint
        next_waypoint = path[1]
        current_pos = (int(self.world_map.info.origin.position.x / self.world_map.info.resolution),
                      int(self.world_map.info.origin.position.y / self.world_map.info.resolution))

        # Convert to world coordinates and send command
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward
        # Calculate angular command based on path direction
        self.cmd_vel_pub.publish(cmd)
```

### Learning-Based AI Integration

AI agents that learn and adapt over time:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import random
from collections import deque

class LearningAINode(Node):
    def __init__(self):
        super().__init__('learning_ai_node')

        # Publishers and subscriptions
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Learning parameters
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.1
        self.memory_size = 10000

        # Learning components
        self.q_table = {}  # State-action value table
        self.experience_buffer = deque(maxlen=self.memory_size)
        self.previous_state = None
        self.previous_action = None
        self.previous_reward = None

        # Robot state
        self.joint_positions = {}
        self.imu_orientation = None
        self.scan_ranges = []

        # Learning timer
        self.learning_timer = self.create_timer(0.1, self.learning_cycle)

    def joint_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Update IMU state"""
        self.imu_orientation = [
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ]

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.scan_ranges = list(msg.ranges)

    def get_current_state(self):
        """Get discretized current state for Q-learning"""
        # Discretize continuous state space
        # For this example: balance state (based on IMU) + proximity to obstacles
        if self.imu_orientation is None or not self.scan_ranges:
            return "unknown"

        # Extract pitch from IMU (simplified)
        _, pitch, _ = self.quaternion_to_euler(self.imu_orientation)

        # Find closest obstacle
        valid_ranges = [r for r in self.scan_ranges if not (np.isnan(r) or np.isinf(r))]
        closest_obstacle = min(valid_ranges) if valid_ranges else float('inf')

        # Discretize into buckets
        pitch_bucket = "balanced" if abs(pitch) < 0.1 else ("tilted_left" if pitch < 0 else "tilted_right")
        obstacle_bucket = "far" if closest_obstacle > 1.0 else ("close" if closest_obstacle > 0.5 else "very_close")

        return f"{pitch_bucket}_{obstacle_bucket}"

    def get_possible_actions(self):
        """Define possible actions for the agent"""
        # For this example: different joint configurations or movement commands
        return [
            "move_forward",
            "turn_left",
            "turn_right",
            "stand_up",
            "crouch",
            "balance_left",
            "balance_right"
        ]

    def get_reward(self, current_state, action, next_state):
        """Calculate reward based on state transition"""
        reward = 0.0

        # Penalty for bad balance
        if "tilted" in current_state:
            reward -= 1.0

        # Reward for maintaining balance
        if "balanced" in next_state:
            reward += 0.5

        # Penalty for being too close to obstacles
        if "very_close" in current_state:
            reward -= 2.0

        # Small time penalty to encourage efficiency
        reward -= 0.01

        return reward

    def choose_action(self, state):
        """Choose action using epsilon-greedy policy"""
        possible_actions = self.get_possible_actions()

        # Exploration vs exploitation
        if random.random() < self.exploration_rate:
            # Explore: choose random action
            return random.choice(possible_actions)

        # Exploitation: choose best known action
        if state not in self.q_table:
            self.q_table[state] = {action: 0.0 for action in possible_actions}

        q_values = self.q_table[state]
        best_action = max(q_values, key=q_values.get)
        return best_action

    def update_q_value(self, state, action, reward, next_state):
        """Update Q-value using Q-learning formula"""
        possible_actions = self.get_possible_actions()

        if state not in self.q_table:
            self.q_table[state] = {action: 0.0 for action in possible_actions}

        if next_state not in self.q_table:
            self.q_table[next_state] = {action: 0.0 for action in possible_actions}

        # Q-learning update rule
        current_q = self.q_table[state][action]
        max_next_q = max(self.q_table[next_state].values())

        new_q = current_q + self.learning_rate * (reward + self.discount_factor * max_next_q - current_q)
        self.q_table[state][action] = new_q

    def execute_action(self, action):
        """Execute the chosen action on the robot"""
        cmd = Twist()
        joint_cmd = Float64MultiArray()

        if action == "move_forward":
            cmd.linear.x = 0.3
        elif action == "turn_left":
            cmd.angular.z = 0.5
        elif action == "turn_right":
            cmd.angular.z = -0.5
        elif action == "stand_up":
            # Send joint commands to stand up position
            joint_cmd.data = [0.0, 0.0, 0.0]  # Example joint positions
        elif action == "crouch":
            # Send joint commands to crouch position
            joint_cmd.data = [0.5, -0.5, 0.2]  # Example joint positions
        elif action == "balance_left":
            # Send commands to correct balance to the left
            cmd.angular.z = 0.2
        elif action == "balance_right":
            # Send commands to correct balance to the right
            cmd.angular.z = -0.2

        # Publish commands
        if cmd.linear.x != 0 or cmd.angular.z != 0:
            self.cmd_vel_pub.publish(cmd)
        if joint_cmd.data:
            self.joint_cmd_pub.publish(joint_cmd)

    def learning_cycle(self):
        """Main learning cycle"""
        current_state = self.get_current_state()
        action = self.choose_action(current_state)

        # Execute action
        self.execute_action(action)

        # Calculate reward based on state transition
        if self.previous_state is not None and self.previous_action is not None:
            reward = self.get_reward(self.previous_state, self.previous_action, current_state)
            self.update_q_value(self.previous_state, self.previous_action, reward, current_state)

        # Store transition for learning
        self.previous_state = current_state
        self.previous_action = action

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (simplified)"""
        w, x, y, z = quat

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
```

## Communication Patterns for AI-ROS Integration

### Action-Based Integration

Using ROS 2 actions for goal-oriented AI behavior:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci
from geometry_msgs.action import NavigateToPose
import time

class AIActionBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_action_bridge_node')

        # Action clients for different robot capabilities
        self.navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.manipulate_client = ActionClient(
            self, Fibonacci, 'fibonacci_manipulation'  # Example action
        )

        # Timer for AI decision making
        self.ai_timer = self.create_timer(2.0, self.ai_decision_cycle)

        # AI state
        self.goals_queue = []
        self.current_task = None

    def ai_decision_cycle(self):
        """AI decision making cycle"""
        # In a real implementation, this would use AI reasoning
        # For this example, we'll simulate goal generation

        # Check if current task is complete
        if self.current_task is None or self.is_task_complete():
            # Get next goal from queue or generate new one
            if self.goals_queue:
                next_goal = self.goals_queue.pop(0)
                self.start_new_task(next_goal)

    def is_task_complete(self):
        """Check if current task is complete"""
        # In a real implementation, this would check action server status
        return True  # Simplified

    def start_new_task(self, goal):
        """Start a new task based on AI decision"""
        if goal['type'] == 'navigation':
            self.send_navigation_goal(goal['pose'])
        elif goal['type'] == 'manipulation':
            self.send_manipulation_goal(goal['object'])

    def send_navigation_goal(self, pose):
        """Send navigation goal to action server"""
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.current_task = 'navigation'
        self._send_goal_future = self.navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.current_task = None
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        self.get_logger().info(f'Navigation progress: {feedback_msg.feedback.distance_remaining:.2f}m remaining')

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed with result: {result}')
        self.current_task = None
```

### Service-Based Integration

Using ROS 2 services for AI query-response interactions:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger, SetBool
from std_srvs.srv import Empty
from geometry_msgs.msg import Point

class AIServiceBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_service_bridge_node')

        # Service clients for robot services
        self.localization_client = self.create_client(Trigger, 'localize_robot')
        self.mapping_client = self.create_client(Trigger, 'toggle_mapping')
        self.calibration_client = self.create_client(SetBool, 'calibrate_sensors')

        # Timer for AI service orchestration
        self.service_timer = self.create_timer(5.0, self.ai_service_cycle)

    def ai_service_cycle(self):
        """AI service orchestration cycle"""
        # Decide which services to call based on AI reasoning
        robot_needs_localization = self.evaluate_localization_need()
        robot_needs_calibration = self.evaluate_calibration_need()
        should_toggle_mapping = self.evaluate_mapping_need()

        # Call services as needed
        if robot_needs_localization:
            self.request_localization()
        if robot_needs_calibration:
            self.request_calibration()
        if should_toggle_mapping:
            self.toggle_mapping(True)

    def evaluate_localization_need(self):
        """Evaluate if robot needs localization"""
        # In a real implementation, this would analyze sensor data and pose uncertainty
        # For this example, return True randomly
        import random
        return random.random() < 0.1  # 10% chance

    def evaluate_calibration_need(self):
        """Evaluate if robot needs sensor calibration"""
        # In a real implementation, this would analyze sensor drift or accuracy
        return False

    def evaluate_mapping_need(self):
        """Evaluate if robot should toggle mapping"""
        return False

    def request_localization(self):
        """Request robot localization service"""
        if not self.localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Localization service not available')
            return

        request = Trigger.Request()
        future = self.localization_client.call_async(request)
        future.add_done_callback(self.localization_callback)

    def request_calibration(self):
        """Request sensor calibration service"""
        if not self.calibration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Calibration service not available')
            return

        request = SetBool.Request()
        request.data = True  # Start calibration
        future = self.calibration_client.call_async(request)
        future.add_done_callback(self.calibration_callback)

    def toggle_mapping(self, enable):
        """Toggle mapping service"""
        if not self.mapping_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Mapping service not available')
            return

        request = Trigger.Request()
        future = self.mapping_client.call_async(request)
        future.add_done_callback(self.mapping_callback)

    def localization_callback(self, future):
        """Handle localization service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Localization completed successfully')
            else:
                self.get_logger().warning(f'Localization failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Localization service call failed: {e}')

    def calibration_callback(self, future):
        """Handle calibration service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Calibration completed: {response.message}')
            else:
                self.get_logger().warning(f'Calibration failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Calibration service call failed: {e}')

    def mapping_callback(self, future):
        """Handle mapping service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Mapping toggled successfully')
            else:
                self.get_logger().warning(f'Mapping toggle failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Mapping service call failed: {e}')
```

## Real-time Considerations

### Latency Management

AI-ROS integration must consider real-time constraints:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
import time
from threading import Thread, Lock

class RealTimeAIBridgeNode(Node):
    def __init__(self):
        super().__init__('realtime_ai_bridge_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriptions with appropriate QoS for real-time
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, sensor_qos
        )
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, sensor_qos
        )

        # Real-time control timer (100Hz for control)
        self.control_timer = self.create_timer(0.01, self.real_time_control_cycle)

        # AI processing timer (lower frequency to avoid blocking)
        self.ai_timer = self.create_timer(0.1, self.ai_processing_cycle)

        # Data buffers with timestamps
        self.latest_image = None
        self.image_timestamp = None
        self.latest_joints = None
        self.joint_timestamp = None

        # Control state
        self.current_command = Twist()
        self.ai_decision = None

        # Threading for AI processing to avoid blocking real-time control
        self.ai_lock = Lock()
        self.ai_result = None

    def image_callback(self, msg):
        """Store image with timestamp for real-time access"""
        self.latest_image = msg
        self.image_timestamp = time.time()

    def joint_callback(self, msg):
        """Store joint state with timestamp for real-time access"""
        self.latest_joints = msg
        self.joint_timestamp = time.time()

    def real_time_control_cycle(self):
        """High-frequency control cycle that must meet deadlines"""
        # Always send current command, regardless of AI state
        self.cmd_pub.publish(self.current_command)

        # Check for new AI decisions without blocking
        with self.ai_lock:
            if self.ai_result is not None:
                # Apply new AI decision
                self.apply_ai_decision(self.ai_result)
                self.ai_result = None

    def ai_processing_cycle(self):
        """Lower-frequency AI processing that can take more time"""
        # Run AI processing in separate thread to avoid blocking
        thread = Thread(target=self.run_ai_processing)
        thread.start()

    def run_ai_processing(self):
        """Run AI processing (runs in separate thread)"""
        if self.latest_image is not None:
            # Process image to make AI decision
            decision = self.process_image_for_decision(self.latest_image)

            # Store result safely
            with self.ai_lock:
                self.ai_result = decision

    def process_image_for_decision(self, image_msg):
        """Process image and make AI decision (placeholder)"""
        # In a real implementation, this would run AI inference
        # For this example, return a simple decision
        return {
            'linear_velocity': 0.2,
            'angular_velocity': 0.0,
            'confidence': 0.9
        }

    def apply_ai_decision(self, decision):
        """Apply AI decision to current command"""
        if decision.get('confidence', 0) > 0.5:  # Confidence threshold
            self.current_command.linear.x = decision.get('linear_velocity', 0.0)
            self.current_command.angular.z = decision.get('angular_velocity', 0.0)
```

## Safety and Robustness

### Graceful Degradation

AI-ROS bridges must handle failures gracefully:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time

class SafeAIBridgeNode(Node):
    def __init__(self):
        super().__init__('safe_ai_bridge_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # Subscriptions
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        # Timers
        self.safety_timer = self.create_timer(0.01, self.safety_check)  # 100Hz safety check
        self.control_timer = self.create_timer(0.1, self.control_cycle)  # 10Hz AI control

        # State variables
        self.joint_states = {}
        self.last_ai_update = time.time()
        self.ai_active = True
        self.emergency_stop_triggered = False

        # Safety parameters
        self.ai_timeout = 1.0  # AI must update within 1 second
        self.joint_limit_threshold = 1.5  # Joint position limit (rad)

    def joint_callback(self, msg):
        """Update joint states for safety monitoring"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]

    def safety_check(self):
        """High-frequency safety checks"""
        if self.emergency_stop_triggered:
            # Emergency stop active, only allow safe commands
            self.publish_safe_command()
            return

        # Check for AI timeout
        time_since_ai_update = time.time() - self.last_ai_update
        if time_since_ai_update > self.ai_timeout:
            self.get_logger().warning('AI update timeout, switching to safe mode')
            self.ai_active = False

        # Check for dangerous joint positions
        for joint_name, position in self.joint_states.items():
            if abs(position) > self.joint_limit_threshold:
                self.get_logger().error(f'Dangerous joint position detected: {joint_name} = {position}')
                self.trigger_emergency_stop()
                return

        # If everything is safe, continue with normal operation
        if not self.ai_active:
            self.publish_safe_command()

    def control_cycle(self):
        """AI control cycle"""
        if self.emergency_stop_triggered:
            return

        # Update last AI update time
        self.last_ai_update = time.time()
        self.ai_active = True

        # Generate AI-based commands (placeholder)
        ai_command = self.generate_ai_command()
        if ai_command:
            self.cmd_pub.publish(ai_command)

    def generate_ai_command(self):
        """Generate command based on AI reasoning (placeholder)"""
        # In a real implementation, this would call the AI system
        cmd = Twist()
        cmd.linear.x = 0.2  # Example command
        cmd.angular.z = 0.0
        return cmd

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop_triggered = True
        self.ai_active = False

        # Publish emergency stop signal
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        self.get_logger().error('EMERGENCY STOP TRIGGERED')

    def publish_safe_command(self):
        """Publish safe (zero) command"""
        safe_cmd = Twist()
        self.cmd_pub.publish(safe_cmd)
```

## Performance Optimization

### Efficient Data Handling

For high-performance AI-ROS integration:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
from cv_bridge import CvBridge
from functools import lru_cache

class OptimizedAIBridgeNode(Node):
    def __init__(self):
        super().__init__('optimized_ai_bridge_node')

        # Use compressed images for efficiency
        self.compressed_image_sub = self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed', self.compressed_image_callback, 10
        )

        # Efficient data publishers
        self.ai_output_pub = self.create_publisher(Float64MultiArray, 'ai_control_output', 10)

        # CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Timers
        self.processing_timer = self.create_timer(0.05, self.processing_cycle)  # 20Hz

        # Data buffers
        self.latest_processed_image = None
        self.feature_cache = {}

    def compressed_image_callback(self, msg):
        """Handle compressed image to reduce bandwidth"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Store for processing
            self.latest_processed_image = cv_image

            # Cache features if possible
            self.extract_features_cached(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')

    @lru_cache(maxsize=10)
    def extract_features_cached(self, image_tuple):
        """Cached feature extraction (tuple input for hashability)"""
        # Convert tuple back to array if needed
        image = np.array(image_tuple)

        # Extract features (simplified example)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Extract key features
        features = {
            'edges': edges,
            'mean_color': np.mean(image, axis=(0, 1)),
            'shape': image.shape
        }

        return features

    def processing_cycle(self):
        """Efficient processing cycle"""
        if self.latest_processed_image is not None:
            # Convert to tuple for caching
            image_tuple = tuple(map(tuple, self.latest_processed_image.reshape(-1, self.latest_processed_image.shape[-1])))

            # Extract features with caching
            features = self.extract_features_cached(image_tuple)

            # Process with AI (simplified)
            ai_output = self.process_with_ai(features)

            # Publish efficiently
            if ai_output is not None:
                output_msg = Float64MultiArray()
                output_msg.data = ai_output.flatten().tolist()
                self.ai_output_pub.publish(output_msg)

    def process_with_ai(self, features):
        """Process features with AI model (placeholder)"""
        # In a real implementation, this would call the AI model
        # For efficiency, ensure the AI model is optimized for the task

        # Example: simple processing
        if 'mean_color' in features:
            # Generate control output based on mean color
            output = np.array([features['mean_color'][0] / 255.0,  # Normalize
                              features['mean_color'][1] / 255.0,
                              features['mean_color'][2] / 255.0])
            return output
        return None
```

## Summary

Bridging AI agents to ROS controllers requires careful consideration of architectural patterns, communication mechanisms, real-time constraints, and safety requirements. The integration can take various forms depending on the AI approach (reactive, deliberative, or learning-based) and the specific requirements of the robotic application.

Successful AI-ROS integration involves:
1. Appropriate architectural patterns for the task
2. Efficient communication protocols
3. Real-time performance considerations
4. Safety and robustness measures
5. Performance optimization techniques

The bridge between AI reasoning and physical control is essential for creating intelligent, autonomous humanoid robots that can adapt to dynamic environments and perform complex tasks.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the differences between reactive, deliberative, and learning-based AI integration patterns. When would you use each approach in humanoid robotics?

2. Describe the cognitive-physical loop in AI-ROS integration. What are the key challenges in maintaining this loop effectively?

### Application Questions
3. Design an AI-ROS bridge for a humanoid robot that needs to:
   - Navigate through a crowded environment
   - Recognize and interact with humans
   - Manipulate objects based on verbal commands
   - Maintain balance while performing tasks
   Specify the integration architecture, communication patterns, and safety measures.

4. How would you implement a learning-based AI system that adapts its control strategies based on robot performance feedback? What learning algorithm would you use and how would you integrate it with ROS?

### Hands-on Practice
5. Create a simple AI-ROS bridge that uses camera input to detect objects and generates velocity commands to move toward detected objects while avoiding obstacles.

6. Implement a state machine that manages the transition between different AI control modes (e.g., exploration, manipulation, navigation) based on sensor data and task requirements.

### Critical Thinking
7. What are the main challenges in integrating real-time AI inference with ROS control systems? How can you address latency and computational resource constraints?

8. How would you design an AI-ROS integration system that can handle multiple simultaneous AI tasks (e.g., perception, planning, learning) while ensuring safety and real-time performance?
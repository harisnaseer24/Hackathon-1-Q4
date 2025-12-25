#!/usr/bin/env python3

"""
AI Agent Integration Examples for Chapter 3
This file demonstrates various patterns for integrating AI agents with ROS 2 controllers
covering concepts from Chapter 3: Bridging AI Agents to ROS Controllers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import json
import time
from typing import Dict, Any, Optional
import threading
import queue


class SimpleAIAgentNode(Node):
    """
    A simple AI agent that demonstrates integration with ROS 2
    This example shows how to bridge high-level AI reasoning with ROS control
    """

    def __init__(self):
        super().__init__('simple_ai_agent_node')

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'ai_agent_status', 10)

        # Subscriptions for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )

        # Timer for AI reasoning cycle
        self.ai_timer = self.create_timer(0.2, self.ai_reasoning_cycle)  # 5Hz

        # State variables
        self.scan_data = None
        self.image_data = None
        self.current_goal = None
        self.robot_pose = None
        self.ai_state = "idle"  # idle, navigating, avoiding_obstacles, etc.

        # AI parameters
        self.safety_distance = 0.5  # meters
        self.goal_tolerance = 0.3   # meters

        self.get_logger().info('Simple AI Agent node initialized')

    def scan_callback(self, msg):
        """Process laser scan data for AI reasoning"""
        self.scan_data = np.array(msg.ranges)
        self.get_logger().debug(f'Received scan with {len(self.scan_data)} ranges')

    def image_callback(self, msg):
        """Process image data for AI reasoning"""
        # In a real implementation, this would run object detection
        # For this example, we'll just store the image header timestamp
        self.image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.get_logger().debug(f'Received image at time: {self.image_timestamp}')

    def goal_callback(self, msg):
        """Process new goal for the AI agent"""
        self.current_goal = msg.pose
        self.get_logger().info(f'Received new goal: {msg.pose.position.x}, {msg.pose.position.y}')
        self.ai_state = "navigating"

    def ai_reasoning_cycle(self):
        """Main AI reasoning cycle"""
        if self.ai_state == "idle":
            # If we have a goal, start navigating
            if self.current_goal is not None:
                self.ai_state = "navigating"
        elif self.ai_state == "navigating":
            # Check if we need to avoid obstacles
            if self.should_avoid_obstacles():
                self.ai_state = "avoiding_obstacles"
            # Check if we've reached the goal
            elif self.has_reached_goal():
                self.ai_state = "goal_reached"
                self.publish_status("goal_reached")
        elif self.ai_state == "avoiding_obstacles":
            # Continue obstacle avoidance until clear
            if not self.should_avoid_obstacles():
                self.ai_state = "navigating"
        elif self.ai_state == "goal_reached":
            # Stop and wait for new goal
            self.stop_robot()
            if self.current_goal is None:
                self.ai_state = "idle"

        # Execute appropriate behavior based on state
        if self.ai_state == "navigating":
            self.execute_navigation()
        elif self.ai_state == "avoiding_obstacles":
            self.execute_obstacle_avoidance()
        elif self.ai_state == "goal_reached":
            self.stop_robot()

    def should_avoid_obstacles(self) -> bool:
        """Determine if obstacles need to be avoided"""
        if self.scan_data is None:
            return False

        # Check for obstacles in front (simplified: front 60 degrees)
        front_start = len(self.scan_data) // 2 - 30
        front_end = len(self.scan_data) // 2 + 30
        front_ranges = self.scan_data[front_start:front_end]

        # Filter out invalid ranges
        valid_ranges = [r for r in front_ranges if not (np.isnan(r) or np.isinf(r))]

        if not valid_ranges:
            return False

        # If any valid range is less than safety distance, need to avoid
        return min(valid_ranges) < self.safety_distance

    def has_reached_goal(self) -> bool:
        """Check if robot has reached the goal"""
        if self.current_goal is None or self.robot_pose is None:
            return False

        # Calculate distance to goal (simplified - would need proper TF in real implementation)
        # For this example, we'll just return False to keep it moving
        return False

    def execute_navigation(self):
        """Execute navigation behavior"""
        cmd = Twist()

        # Simple proportional navigation to goal
        # In a real implementation, this would use proper path planning
        cmd.linear.x = 0.3  # Move forward at 0.3 m/s
        cmd.angular.z = 0.0  # No turning for now

        self.cmd_vel_pub.publish(cmd)
        self.publish_status("navigating_to_goal")

    def execute_obstacle_avoidance(self):
        """Execute obstacle avoidance behavior"""
        cmd = Twist()

        if self.scan_data is not None:
            # Simple reactive obstacle avoidance
            # Find the direction with the most clearance
            left_ranges = self.scan_data[:len(self.scan_data)//3]
            front_ranges = self.scan_data[len(self.scan_data)//3:2*len(self.scan_data)//3]
            right_ranges = self.scan_data[2*len(self.scan_data)//3:]

            # Calculate average clearance in each direction
            left_clearance = np.mean([r for r in left_ranges if not (np.isnan(r) or np.isinf(r))] or [0.1])
            front_clearance = np.mean([r for r in front_ranges if not (np.isnan(r) or np.isinf(r))] or [0.1])
            right_clearance = np.mean([r for r in right_ranges if not (np.isnan(r) or np.isinf(r))] or [0.1])

            # Turn toward the direction with most clearance
            if left_clearance > right_clearance:
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.angular.z = -0.5  # Turn right

            # Reduce forward speed when turning
            cmd.linear.x = 0.1

        self.cmd_vel_pub.publish(cmd)
        self.publish_status("avoiding_obstacles")

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.publish_status("stopped")

    def publish_status(self, status: str):
        """Publish AI agent status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


class LearningAIAgentNode(Node):
    """
    An AI agent that learns from experience
    Demonstrates reinforcement learning integration with ROS 2
    """

    def __init__(self):
        super().__init__('learning_ai_agent_node')

        # Publishers and subscriptions
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reward_pub = self.create_publisher(String, 'ai_reward', 10)

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Timer for learning cycle
        self.learning_timer = self.create_timer(0.1, self.learning_cycle)  # 10Hz

        # State variables
        self.scan_data = None
        self.imu_data = None
        self.episode_step = 0
        self.cumulative_reward = 0.0

        # Simple Q-table for learning (discretized state-action space)
        self.q_table = {}  # Maps (state, action) to value
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.1

        # Discretization parameters
        self.distance_bins = 5  # Discretize distance into 5 bins
        self.angle_bins = 5     # Discretize angle into 5 bins

        self.get_logger().info('Learning AI Agent node initialized')

    def scan_callback(self, msg):
        """Process laser scan for learning"""
        self.scan_data = np.array(msg.ranges)

    def imu_callback(self, msg):
        """Process IMU data for learning"""
        self.imu_data = msg

    def learning_cycle(self):
        """Main learning cycle"""
        if self.scan_data is None:
            return

        # Get current state
        state = self.get_discretized_state()

        # Choose action using epsilon-greedy
        action = self.choose_action(state)

        # Execute action
        self.execute_action(action)

        # Calculate reward
        reward = self.calculate_reward()

        # Update Q-value
        self.update_q_value(state, action, reward)

        # Log learning progress
        self.cumulative_reward += reward
        self.episode_step += 1

        if self.episode_step % 100 == 0:
            self.get_logger().info(f'Cumulative reward: {self.cumulative_reward:.2f} over {self.episode_step} steps')

    def get_discretized_state(self) -> str:
        """Convert continuous sensor data to discrete state"""
        if self.scan_data is None:
            return "unknown"

        # Simplified state representation: closest distance in front and robot tilt
        front_distances = self.scan_data[len(self.scan_data)//2-15:len(self.scan_data)//2+15]
        min_front_distance = min([d for d in front_distances if not (np.isnan(d) or np.isinf(d))] or [10.0])

        # Discretize distance into bins
        distance_bin = min(int(min_front_distance * self.distance_bins / 5.0), self.distance_bins - 1)

        # Get tilt from IMU if available
        tilt_bin = 0
        if self.imu_data is not None:
            # Simplified tilt calculation
            tilt = abs(self.imu_data.orientation.z)  # Approximate tilt
            tilt_bin = min(int(tilt * self.angle_bins), self.angle_bins - 1)

        return f"dist_{distance_bin}_tilt_{tilt_bin}"

    def choose_action(self, state: str) -> int:
        """Choose action using epsilon-greedy policy"""
        # Define possible actions: 0=forward, 1=turn_left, 2=turn_right, 3=backward
        actions = [0, 1, 2, 3]

        # Exploration vs exploitation
        if np.random.random() < self.exploration_rate:
            # Explore: choose random action
            return np.random.choice(actions)

        # Exploitation: choose best known action
        if state not in self.q_table:
            # Initialize Q-values for this state
            self.q_table[state] = {action: 0.0 for action in actions}

        # Choose action with highest Q-value
        q_values = self.q_table[state]
        best_action = max(q_values, key=q_values.get)
        return best_action

    def execute_action(self, action: int):
        """Execute the chosen action"""
        cmd = Twist()

        if action == 0:  # Move forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        elif action == 1:  # Turn left
            cmd.linear.x = 0.1
            cmd.angular.z = 0.5
        elif action == 2:  # Turn right
            cmd.linear.x = 0.1
            cmd.angular.z = -0.5
        elif action == 3:  # Move backward
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def calculate_reward(self) -> float:
        """Calculate reward based on current state"""
        reward = 0.0

        if self.scan_data is not None:
            # Reward for moving forward safely
            front_distances = self.scan_data[len(self.scan_data)//2-15:len(self.scan_data)//2+15]
            min_front_distance = min([d for d in front_distances if not (np.isnan(d) or np.isinf(d))] or [0.1])

            if min_front_distance > 0.5:  # Safe distance
                reward += 0.1  # Small reward for safe forward progress
            else:
                reward -= 1.0  # Penalty for being too close to obstacles

        # Small time penalty to encourage efficiency
        reward -= 0.01

        # Publish reward for monitoring
        reward_msg = String()
        reward_msg.data = f"reward:{reward:.3f}"
        self.reward_pub.publish(reward_msg)

        return reward

    def update_q_value(self, state: str, action: int, reward: float):
        """Update Q-value using Q-learning algorithm"""
        if state not in self.q_table:
            self.q_table[state] = {a: 0.0 for a in [0, 1, 2, 3]}

        # Get current Q-value
        current_q = self.q_table[state][action]

        # Calculate maximum Q-value for next state (in a real implementation,
        # we would need to predict the next state)
        # For this example, we'll use a simplified approach
        max_next_q = max(self.q_table[state].values())

        # Q-learning update rule
        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * max_next_q - current_q
        )

        self.q_table[state][action] = new_q


class AIBridgeNode(Node):
    """
    Advanced AI-ROS bridge demonstrating complex integration patterns
    """

    def __init__(self):
        super().__init__('ai_bridge_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'bridge_status', 10)

        # Subscriptions with different QoS for different sensor types
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, sensor_qos)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Service clients for higher-level capabilities
        self.nav_client = self.create_client(NavigateToPose, 'navigate_to_pose')

        # Timer for AI coordination
        self.coordination_timer = self.create_timer(0.5, self.coordinated_decision_cycle)

        # Threading for non-blocking AI processing
        self.ai_result_queue = queue.Queue()
        self.ai_processing_thread = threading.Thread(target=self.background_ai_processing, daemon=True)
        self.ai_processing_thread.start()

        # State management
        self.sensor_fusion_data = {}
        self.ai_decisions = []
        self.active_goals = []

        self.get_logger().info('AI Bridge node initialized')

    def scan_callback(self, msg):
        """Process scan data with fusion"""
        self.sensor_fusion_data['laser'] = {
            'ranges': np.array(msg.ranges),
            'timestamp': time.time(),
            'frame_id': msg.header.frame_id
        }
        self.trigger_ai_processing()

    def image_callback(self, msg):
        """Process image data with fusion"""
        self.sensor_fusion_data['camera'] = {
            'height': msg.height,
            'width': msg.width,
            'encoding': msg.encoding,
            'timestamp': time.time(),
            'frame_id': msg.header.frame_id
        }
        self.trigger_ai_processing()

    def trigger_ai_processing(self):
        """Trigger background AI processing"""
        # In a real implementation, this would send sensor data to the AI thread
        # For this example, we'll just log that processing was triggered
        self.get_logger().debug('AI processing triggered by sensor update')

    def background_ai_processing(self):
        """Background thread for AI processing"""
        while rclpy.ok():
            # In a real implementation, this would run heavy AI computations
            # For this example, we'll just simulate processing
            time.sleep(0.1)

    def coordinated_decision_cycle(self):
        """Coordinate multiple AI decision sources"""
        # Fuse sensor data
        fused_state = self.fuse_sensor_data()

        # Generate decisions from multiple sources
        navigation_decision = self.make_navigation_decision(fused_state)
        safety_decision = self.make_safety_decision(fused_state)
        task_decision = self.make_task_decision(fused_state)

        # Coordinate decisions
        final_command = self.coordinate_decisions(
            navigation_decision, safety_decision, task_decision
        )

        # Execute command
        if final_command:
            self.cmd_pub.publish(final_command)

    def fuse_sensor_data(self) -> Dict[str, Any]:
        """Fuse data from multiple sensors"""
        fused_data = {
            'timestamp': time.time(),
            'has_laser_data': 'laser' in self.sensor_fusion_data,
            'has_camera_data': 'camera' in self.sensor_fusion_data,
            'environment_model': {}
        }

        # Process laser data
        if 'laser' in self.sensor_fusion_data:
            laser_data = self.sensor_fusion_data['laser']
            # Calculate simple environment features
            valid_ranges = [r for r in laser_data['ranges'] if not (np.isnan(r) or np.isinf(r))]
            if valid_ranges:
                fused_data['environment_model']['min_distance'] = min(valid_ranges)
                fused_data['environment_model']['mean_distance'] = np.mean(valid_ranges)

        # Process camera data
        if 'camera' in self.sensor_fusion_data:
            cam_data = self.sensor_fusion_data['camera']
            fused_data['environment_model']['image_available'] = True
            fused_data['environment_model']['resolution'] = (cam_data['width'], cam_data['height'])

        return fused_data

    def make_navigation_decision(self, state: Dict[str, Any]) -> Optional[Twist]:
        """Make navigation decision based on fused state"""
        if 'environment_model' in state:
            env_model = state['environment_model']
            if env_model.get('min_distance', float('inf')) > 0.5:  # Safe to move
                cmd = Twist()
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
                return cmd
        return None

    def make_safety_decision(self, state: Dict[str, Any]) -> Optional[Twist]:
        """Make safety-related decisions"""
        if 'environment_model' in state:
            env_model = state['environment_model']
            min_dist = env_model.get('min_distance', float('inf'))

            if min_dist < 0.3:  # Emergency stop threshold
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().warn('Safety decision: Emergency stop triggered')
                return cmd
        return None

    def make_task_decision(self, state: Dict[str, Any]) -> Optional[Twist]:
        """Make task-specific decisions"""
        # Placeholder for task-level decisions
        return None

    def coordinate_decisions(self, nav_decision: Optional[Twist],
                           safety_decision: Optional[Twist],
                           task_decision: Optional[Twist]) -> Optional[Twist]:
        """Coordinate multiple decision sources with priority"""
        # Safety decisions have highest priority
        if safety_decision is not None:
            return safety_decision

        # Task decisions have medium priority
        if task_decision is not None:
            return task_decision

        # Navigation decisions have lowest priority
        return nav_decision


def main(args=None):
    """
    Main function to run the AI integration examples
    """
    rclpy.init(args=args)

    # Create AI agent nodes
    simple_agent = SimpleAIAgentNode()
    learning_agent = LearningAIAgentNode()
    bridge_node = AIBridgeNode()

    # Create multi-threaded executor to handle all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simple_agent)
    executor.add_node(learning_agent)
    executor.add_node(bridge_node)

    try:
        print("AI Agent Integration Examples running...")
        print("This example demonstrates:")
        print("- Simple AI agent with reactive behaviors")
        print("- Learning AI agent with Q-learning")
        print("- Advanced AI-ROS bridge with sensor fusion")
        print("Check the logs for AI decision making and learning progress.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        simple_agent.destroy_node()
        learning_agent.destroy_node()
        bridge_node.destroy_node()
        rclpy.shutdown()
        print("AI Agent Integration Examples finished.")


if __name__ == '__main__':
    main()
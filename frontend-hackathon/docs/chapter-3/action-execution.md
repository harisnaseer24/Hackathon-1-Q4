---
sidebar_position: 3
title: "Action Execution from Decision Logic"
---

# Action Execution from Decision Logic

## Introduction

The transformation of AI decisions into physical robot actions is a critical component of autonomous robotic systems. This chapter explores the patterns and mechanisms for executing actions based on decision logic, focusing on how AI reasoning translates into concrete robot behaviors. In humanoid robotics, this process must be robust, safe, and responsive to the dynamic nature of human environments.

Action execution involves multiple layers of decision-making, from high-level goals determined by AI systems to low-level joint commands executed by robot controllers. The challenge lies in maintaining the connection between abstract decision logic and physical action while ensuring safety, efficiency, and adaptability.

## The Decision-to-Action Pipeline

### From Decision to Execution

The process of translating AI decisions into robot actions follows a structured pipeline:

1. **Decision Making**: AI system generates a high-level decision or goal
2. **Action Planning**: Convert high-level goals into executable action sequences
3. **Motion Planning**: Generate detailed trajectories for robot limbs
4. **Control Execution**: Execute low-level commands to robot actuators
5. **Feedback Integration**: Monitor execution and adjust as needed

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
import time
from typing import Optional

class DecisionToActionNode(Node):
    def __init__(self):
        super().__init__('decision_to_action_node')

        # Publishers for different control levels
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'action_status', 10)

        # Subscriptions for feedback
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.pose_callback, 10)

        # Action clients for complex behaviors
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_action_client = ActionClient(self, ManipulateObject, 'manipulate_object')

        # Timer for decision processing
        self.decision_timer = self.create_timer(0.1, self.decision_processing_cycle)

        # State variables
        self.current_decision = None
        self.current_action = None
        self.action_status = "idle"
        self.joint_states = {}
        self.robot_pose = None

        # Decision queue
        self.decision_queue = []

    def joint_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]

    def pose_callback(self, msg):
        """Update robot pose information"""
        self.robot_pose = msg.pose

    def add_decision(self, decision):
        """Add a decision to the processing queue"""
        self.decision_queue.append(decision)
        self.get_logger().info(f'Added decision to queue: {decision}')

    def decision_processing_cycle(self):
        """Main cycle for processing decisions and initiating actions"""
        if not self.decision_queue:
            return

        # Get next decision
        decision = self.decision_queue.pop(0)
        self.current_decision = decision

        # Process decision and generate appropriate action
        action = self.process_decision(decision)
        if action:
            self.execute_action(action)

    def process_decision(self, decision):
        """Process decision and return appropriate action"""
        decision_type = decision.get('type', 'unknown')

        if decision_type == 'navigate':
            return {
                'action_type': 'navigation',
                'target_pose': decision['target'],
                'priority': decision.get('priority', 1)
            }
        elif decision_type == 'manipulate':
            return {
                'action_type': 'manipulation',
                'object_id': decision['object_id'],
                'action': decision['manipulation_action'],
                'priority': decision.get('priority', 1)
            }
        elif decision_type == 'communicate':
            return {
                'action_type': 'communication',
                'message': decision['message'],
                'recipient': decision.get('recipient', 'all'),
                'priority': decision.get('priority', 0)
            }
        else:
            self.get_logger().warning(f'Unknown decision type: {decision_type}')
            return None

    def execute_action(self, action):
        """Execute the generated action"""
        action_type = action['action_type']
        self.current_action = action
        self.action_status = "executing"

        if action_type == 'navigation':
            self.execute_navigation_action(action)
        elif action_type == 'manipulation':
            self.execute_manipulation_action(action)
        elif action_type == 'communication':
            self.execute_communication_action(action)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            self.action_status = "failed"

    def execute_navigation_action(self, action):
        """Execute navigation action using action client"""
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            self.action_status = "failed"
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = action['target_pose']

        self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.navigation_goal_response_callback)

    def execute_manipulation_action(self, action):
        """Execute manipulation action using action client"""
        if not self.manipulation_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Manipulation action server not available')
            self.action_status = "failed"
            return

        goal_msg = ManipulateObject.Goal()
        goal_msg.object_id = action['object_id']
        goal_msg.action = action['action']

        self.manipulation_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.manipulation_feedback_callback
        ).add_done_callback(self.manipulation_goal_response_callback)

    def execute_communication_action(self, action):
        """Execute communication action"""
        # Publish communication message
        comm_msg = String()
        comm_msg.data = f"[{action['recipient']}] {action['message']}"
        self.status_pub.publish(comm_msg)

        self.action_status = "completed"
        self.get_logger().info(f'Communication action completed: {action["message"]}')

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation action feedback"""
        self.get_logger().info(f'Navigating... {feedback_msg.feedback.distance_remaining:.2f}m remaining')

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.action_status = "failed"
            return

        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.action_status = "completed"
            self.get_logger().info('Navigation completed successfully')
        else:
            self.action_status = "failed"
            self.get_logger().error(f'Navigation failed with status: {status}')

        # Publish completion status
        status_msg = String()
        status_msg.data = f"navigation_{self.action_status}"
        self.status_pub.publish(status_msg)
```

## Action Planning and Sequencing

### Hierarchical Action Structures

Complex robot behaviors often require sequences of actions that must be orchestrated:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from enum import Enum
from typing import List, Dict, Any
import asyncio

class ActionStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

class ActionStep:
    """Represents a single step in an action sequence"""
    def __init__(self, name: str, action_type: str, parameters: Dict[str, Any],
                 dependencies: List[str] = None):
        self.name = name
        self.action_type = action_type
        self.parameters = parameters
        self.dependencies = dependencies or []
        self.status = ActionStatus.PENDING
        self.start_time = None
        self.end_time = None

class ActionSequence:
    """Represents a sequence of actions with dependencies"""
    def __init__(self, name: str, steps: List[ActionStep]):
        self.name = name
        self.steps = steps
        self.status = ActionStatus.PENDING
        self.current_step_idx = 0

    def get_ready_steps(self) -> List[ActionStep]:
        """Get steps that are ready to execute (dependencies satisfied)"""
        ready_steps = []
        for step in self.steps:
            if step.status == ActionStatus.PENDING:
                all_deps_met = all(
                    self.get_step_by_name(dep).status == ActionStatus.COMPLETED
                    for dep in step.dependencies
                )
                if all_deps_met:
                    ready_steps.append(step)
        return ready_steps

    def get_step_by_name(self, name: str) -> ActionStep:
        """Get a step by its name"""
        for step in self.steps:
            if step.name == name:
                return step
        return None

class ActionSequencerNode(Node):
    def __init__(self):
        super().__init__('action_sequencer_node')

        # Publishers
        self.status_pub = self.create_publisher(String, 'action_sequence_status', 10)

        # Timer for action sequencing
        self.sequencer_timer = self.create_timer(0.1, self.sequencer_cycle)

        # Active sequences
        self.active_sequences = {}
        self.sequence_counter = 0

    def create_pick_and_place_sequence(self) -> ActionSequence:
        """Create a pick-and-place action sequence"""
        steps = [
            ActionStep(
                name="approach_object",
                action_type="navigation",
                parameters={"target_pose": {"x": 1.0, "y": 0.0, "theta": 0.0}},
                dependencies=[]
            ),
            ActionStep(
                name="align_with_object",
                action_type="manipulation",
                parameters={"action": "align", "object_id": "target_object"},
                dependencies=["approach_object"]
            ),
            ActionStep(
                name="grasp_object",
                action_type="manipulation",
                parameters={"action": "grasp", "object_id": "target_object"},
                dependencies=["align_with_object"]
            ),
            ActionStep(
                name="lift_object",
                action_type="manipulation",
                parameters={"action": "lift", "height": 0.2},
                dependencies=["grasp_object"]
            ),
            ActionStep(
                name="navigate_to_destination",
                action_type="navigation",
                parameters={"target_pose": {"x": 2.0, "y": 1.0, "theta": 0.0}},
                dependencies=["lift_object"]
            ),
            ActionStep(
                name="place_object",
                action_type="manipulation",
                parameters={"action": "place", "object_id": "target_object"},
                dependencies=["navigate_to_destination"]
            )
        ]

        return ActionSequence("pick_and_place", steps)

    def start_sequence(self, sequence: ActionSequence) -> str:
        """Start executing an action sequence"""
        seq_id = f"seq_{self.sequence_counter}"
        self.sequence_counter += 1

        sequence.id = seq_id
        self.active_sequences[seq_id] = sequence

        self.get_logger().info(f'Started sequence: {sequence.name} (ID: {seq_id})')
        return seq_id

    def sequencer_cycle(self):
        """Main cycle for managing action sequences"""
        for seq_id, sequence in list(self.active_sequences.items()):
            if sequence.status in [ActionStatus.COMPLETED, ActionStatus.FAILED, ActionStatus.CANCELLED]:
                # Remove completed sequences
                del self.active_sequences[seq_id]
                continue

            # Process ready steps in the sequence
            ready_steps = sequence.get_ready_steps()

            for step in ready_steps:
                if step.status == ActionStatus.PENDING:
                    self.execute_action_step(sequence, step)

            # Update sequence status based on step statuses
            self.update_sequence_status(sequence)

            # Publish sequence status
            status_msg = String()
            status_msg.data = f"sequence_{seq_id}:{sequence.status.value}"
            self.status_pub.publish(status_msg)

    def execute_action_step(self, sequence: ActionSequence, step: ActionStep):
        """Execute a single action step"""
        step.status = ActionStatus.RUNNING
        step.start_time = self.get_clock().now()

        self.get_logger().info(f'Executing step: {step.name} in sequence {sequence.name}')

        # In a real implementation, this would call the appropriate action client
        # For this example, we'll simulate execution
        self.simulate_action_execution(sequence, step)

    def simulate_action_execution(self, sequence: ActionSequence, step: ActionStep):
        """Simulate action execution (in real implementation, this would call actual action servers)"""
        # Simulate different execution times based on action type
        import random
        if step.action_type == "navigation":
            execution_time = random.uniform(2.0, 5.0)  # 2-5 seconds for navigation
        elif step.action_type == "manipulation":
            execution_time = random.uniform(1.0, 3.0)  # 1-3 seconds for manipulation
        else:
            execution_time = 1.0  # 1 second for other actions

        # For simulation purposes, we'll mark as completed after a delay
        # In real implementation, we'd wait for action server feedback
        timer = self.create_timer(execution_time, lambda: self.complete_action_step(sequence, step))

    def complete_action_step(self, sequence: ActionSequence, step: ActionStep):
        """Complete an action step"""
        step.status = ActionStatus.COMPLETED
        step.end_time = self.get_clock().now()

        self.get_logger().info(f'Step completed: {step.name} in sequence {sequence.name}')

    def update_sequence_status(self, sequence: ActionSequence):
        """Update the overall status of a sequence"""
        step_statuses = [step.status for step in sequence.steps]

        if all(status == ActionStatus.COMPLETED for status in step_statuses):
            sequence.status = ActionStatus.COMPLETED
        elif any(status == ActionStatus.FAILED for status in step_statuses):
            sequence.status = ActionStatus.FAILED
        elif any(status == ActionStatus.CANCELLED for status in step_statuses):
            sequence.status = ActionStatus.CANCELLED
        elif any(status == ActionStatus.RUNNING for status in step_statuses):
            sequence.status = ActionStatus.RUNNING
        else:
            sequence.status = ActionStatus.PENDING
```

## Decision Logic Implementation

### Rule-Based Decision Systems

Rule-based systems provide deterministic decision making:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from typing import Dict, List, Any
import re

class RuleBasedDecisionNode(Node):
    def __init__(self):
        super().__init__('rule_based_decision_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'decision_status', 10)

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Timer for decision making
        self.decision_timer = self.create_timer(0.1, self.rule_based_decision_cycle)

        # Robot state
        self.scan_data = None
        self.image_data = None
        self.obstacle_detected = False
        self.target_detected = False
        self.target_direction = 0.0  # angle to target (-pi to pi)

        # Decision rules
        self.decision_rules = [
            # Rule 1: Emergency stop if obstacle is too close
            {
                'condition': lambda: self.min_scan_distance() < 0.3,
                'action': lambda: self.emergency_stop(),
                'priority': 100,
                'name': 'emergency_stop'
            },
            # Rule 2: Avoid obstacles if they are in the path
            {
                'condition': lambda: self.obstacle_in_path() and self.min_scan_distance() < 0.8,
                'action': lambda: self.avoid_obstacles(),
                'priority': 90,
                'name': 'obstacle_avoidance'
            },
            # Rule 3: Move toward detected target
            {
                'condition': lambda: self.target_detected,
                'action': lambda: self.move_toward_target(),
                'priority': 50,
                'name': 'target_approach'
            },
            # Rule 4: Wander if no specific target
            {
                'condition': lambda: not self.target_detected and not self.obstacle_in_path(),
                'action': lambda: self.wander(),
                'priority': 10,
                'name': 'wandering'
            }
        ]

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.scan_data = np.array(msg.ranges)
        self.update_obstacle_detection()

    def image_callback(self, msg):
        """Process camera image for target detection"""
        # In a real implementation, this would run computer vision algorithms
        # For this example, we'll simulate target detection
        self.update_target_detection(msg)

    def update_obstacle_detection(self):
        """Update obstacle detection based on scan data"""
        if self.scan_data is not None and len(self.scan_data) > 0:
            # Check for obstacles in front (simplified: front 60 degrees)
            front_indices = slice(len(self.scan_data)//2 - 30, len(self.scan_data)//2 + 30)
            front_distances = self.scan_data[front_indices]
            valid_distances = [d for d in front_distances if not (np.isnan(d) or np.isinf(d))]

            if valid_distances:
                self.obstacle_detected = min(valid_distances) < 1.0
            else:
                self.obstacle_detected = False

    def update_target_detection(self, image_msg):
        """Update target detection based on image data (simulated)"""
        # In a real implementation, this would run object detection
        # For this example, simulate target detection
        import random
        self.target_detected = random.random() < 0.3  # 30% chance of target
        if self.target_detected:
            self.target_direction = random.uniform(-1.0, 1.0)  # Random direction

    def min_scan_distance(self):
        """Get minimum valid scan distance"""
        if self.scan_data is None or len(self.scan_data) == 0:
            return float('inf')

        valid_distances = [d for d in self.scan_data if not (np.isnan(d) or np.isinf(d))]
        return min(valid_distances) if valid_distances else float('inf')

    def obstacle_in_path(self):
        """Check if obstacle is in the robot's path"""
        if self.scan_data is None or len(self.scan_data) == 0:
            return False

        # Check front arc (simplified)
        front_range = self.scan_data[len(self.scan_data)//2 - 15 : len(self.scan_data)//2 + 15]
        valid_front = [d for d in front_range if not (np.isnan(d) or np.isinf(d))]

        if not valid_front:
            return False

        return min(valid_front) < 0.8  # Obstacle within 80cm

    def rule_based_decision_cycle(self):
        """Main cycle for rule-based decision making"""
        # Evaluate all rules and sort by priority
        active_rules = []
        for rule in self.decision_rules:
            if rule['condition']():
                active_rules.append(rule)

        # Sort by priority (highest first)
        active_rules.sort(key=lambda x: x['priority'], reverse=True)

        # Execute highest priority rule
        if active_rules:
            rule = active_rules[0]
            self.get_logger().debug(f'Executing rule: {rule["name"]} (priority: {rule["priority"]})')
            rule['action']()

            # Publish decision status
            status_msg = String()
            status_msg.data = f'executing_rule: {rule["name"]}'
            self.status_pub.publish(status_msg)
        else:
            # No active rules - stop robot
            self.emergency_stop()

    def emergency_stop(self):
        """Stop robot immediately"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def avoid_obstacles(self):
        """Generate commands to avoid obstacles"""
        cmd = Twist()

        # Determine turn direction based on obstacle distribution
        if self.scan_data is not None:
            left_distances = self.scan_data[:len(self.scan_data)//2]
            right_distances = self.scan_data[len(self.scan_data)//2:]

            left_clear = np.mean([d for d in left_distances if not (np.isnan(d) or np.isinf(d))] or [1.0])
            right_clear = np.mean([d for d in right_distances if not (np.isnan(d) or np.isinf(d))] or [1.0])

            if left_clear > right_clear:
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.angular.z = -0.5  # Turn right

        cmd.linear.x = 0.0  # Stop forward motion when avoiding obstacles
        self.cmd_pub.publish(cmd)

    def move_toward_target(self):
        """Move toward detected target"""
        cmd = Twist()

        # Move forward
        cmd.linear.x = 0.3

        # Turn toward target
        cmd.angular.z = self.target_direction * 0.5  # Scale the turn command

        self.cmd_pub.publish(cmd)

    def wander(self):
        """Wander behavior when no specific target"""
        cmd = Twist()

        # Move forward
        cmd.linear.x = 0.2

        # Random gentle turns
        import random
        cmd.angular.z = random.uniform(-0.2, 0.2)

        self.cmd_pub.publish(cmd)
```

### Learning-Based Decision Systems

Machine learning systems can adapt decision making over time:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import pickle
import os
from typing import Dict, List, Tuple

class LearningBasedDecisionNode(Node):
    def __init__(self):
        super().__init__('learning_decision_node')

        # Publishers and subscriptions
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Timer for learning-based decision making
        self.learning_timer = self.create_timer(0.1, self.learning_decision_cycle)

        # Robot state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_orientation = None
        self.scan_ranges = []

        # Learning components
        self.policy_network = self.initialize_policy_network()
        self.experience_buffer = []
        self.training_mode = True

        # Learning parameters
        self.learning_rate = 0.001
        self.discount_factor = 0.95
        self.epsilon = 0.1  # Exploration rate

    def initialize_policy_network(self):
        """Initialize or load a policy network"""
        # For this example, we'll use a simple linear model
        # In a real implementation, this would be a neural network
        model_path = 'policy_model.pkl'

        if os.path.exists(model_path):
            # Load existing model
            with open(model_path, 'rb') as f:
                return pickle.load(f)
        else:
            # Initialize new model (simple linear weights for this example)
            # State size: 6 joint positions + 6 joint velocities + 4 IMU values + 100 scan readings
            state_size = 6 + 6 + 4 + 100  # Adjust based on actual sensor configuration
            action_size = 2  # linear.x and angular.z velocities
            return {
                'weights': np.random.normal(0, 0.1, (action_size, state_size)),
                'bias': np.zeros(action_size)
            }

    def joint_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update IMU state"""
        self.imu_orientation = [
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ]

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.scan_ranges = list(msg.ranges)

    def get_state_vector(self) -> np.ndarray:
        """Convert robot state to feature vector for learning"""
        # Create state vector from sensor data
        state_parts = []

        # Joint positions (first 6 joints, assuming humanoid)
        joint_names = ['hip_joint', 'knee_joint', 'ankle_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
        for name in joint_names:
            pos = self.joint_positions.get(name, 0.0)
            state_parts.append(pos)

        # Joint velocities (first 6 joints)
        for name in joint_names:
            vel = self.joint_velocities.get(name, 0.0)
            state_parts.append(vel)

        # IMU orientation (if available)
        if self.imu_orientation:
            state_parts.extend(self.imu_orientation)
        else:
            state_parts.extend([0.0, 0.0, 0.0, 1.0])  # Default quaternion

        # Laser scan (first 100 readings, subsampled if necessary)
        if self.scan_ranges:
            scan_data = self.scan_ranges[:100]
            # Pad with infinity if not enough readings
            if len(scan_data) < 100:
                scan_data.extend([float('inf')] * (100 - len(scan_data)))
            # Normalize finite values
            normalized_scan = []
            for r in scan_data:
                if np.isfinite(r):
                    normalized_scan.append(min(r, 10.0) / 10.0)  # Normalize to [0, 1] for values up to 10m
                else:
                    normalized_scan.append(1.0)  # Treat infinite as maximum distance
            state_parts.extend(normalized_scan)
        else:
            state_parts.extend([1.0] * 100)  # Default: maximum distances

        return np.array(state_parts, dtype=np.float32)

    def get_reward(self, prev_state: np.ndarray, action: np.ndarray, new_state: np.ndarray) -> float:
        """Calculate reward based on state transition and action"""
        reward = 0.0

        # Positive reward for moving forward (if applicable)
        if action[0] > 0:  # linear.x > 0
            reward += 0.1

        # Negative reward for getting too close to obstacles
        scan_start_idx = 6 + 6 + 4  # Skip joint pos/vel and IMU
        scan_readings = new_state[scan_start_idx:scan_start_idx+100]
        min_distance = min(scan_readings) * 10.0  # Denormalize to meters
        if min_distance < 0.5:
            reward -= 2.0  # Big penalty for being too close

        # Positive reward for maintaining balance (simplified)
        imu_start_idx = 6 + 6  # Skip joint positions and velocities
        imu_vals = new_state[imu_start_idx:imu_start_idx+4]
        roll, pitch, _ = self.quaternion_to_rpy(imu_vals)
        if abs(roll) < 0.2 and abs(pitch) < 0.2:
            reward += 0.5  # Reward for staying upright

        # Small negative reward for excessive turning
        if abs(action[1]) > 0.5:  # angular.z
            reward -= 0.1

        return reward

    def quaternion_to_rpy(self, quat):
        """Convert quaternion to roll-pitch-yaw (simplified)"""
        x, y, z, w = quat
        # Simplified conversion (not completely accurate but sufficient for reward calc)
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw

    def choose_action(self, state: np.ndarray) -> np.ndarray:
        """Choose action based on current policy"""
        if np.random.random() < self.epsilon and self.training_mode:
            # Exploration: random action
            return np.random.uniform(-1, 1, size=2)  # [linear.x, angular.z]
        else:
            # Exploitation: use current policy
            weights = self.policy_network['weights']
            bias = self.policy_network['bias']

            # Simple linear policy: action = weights * state + bias
            action = weights.dot(state) + bias

            # Clip actions to reasonable ranges
            action[0] = np.clip(action[0], -1.0, 1.0)  # linear.x
            action[1] = np.clip(action[1], -1.0, 1.0)  # angular.z

            return action

    def update_policy(self, state: np.ndarray, action: np.ndarray, reward: float, next_state: np.ndarray, done: bool):
        """Update policy based on experience"""
        if not self.training_mode:
            return

        # For this simple example, we'll use a basic gradient update
        # In a real implementation, this would use more sophisticated RL algorithms

        # Calculate TD error
        current_value = self.policy_network['weights'].dot(state) + self.policy_network['bias']
        if done:
            target = reward
        else:
            next_value = self.policy_network['weights'].dot(next_state) + self.policy_network['bias']
            target = reward + self.discount_factor * np.max(next_value)

        td_error = target - current_value

        # Update weights using gradient descent
        grad = np.outer(td_error, state)
        self.policy_network['weights'] += self.learning_rate * grad
        self.policy_network['bias'] += self.learning_rate * td_error

    def learning_decision_cycle(self):
        """Main cycle for learning-based decision making"""
        current_state = self.get_state_vector()

        # Choose action based on current policy
        action = self.choose_action(current_state)

        # Execute action
        cmd = Twist()
        cmd.linear.x = float(action[0]) * 0.5  # Scale to reasonable speed
        cmd.angular.z = float(action[1]) * 0.5  # Scale to reasonable turning
        self.cmd_pub.publish(cmd)

        # If we have previous experience, update policy
        if hasattr(self, 'prev_state') and hasattr(self, 'prev_action'):
            reward = self.get_reward(self.prev_state, self.prev_action, current_state)
            self.update_policy(self.prev_state, self.prev_action, reward, current_state, False)

        # Store current experience for next iteration
        self.prev_state = current_state
        self.prev_action = action

    def save_model(self):
        """Save the trained model"""
        with open('policy_model.pkl', 'wb') as f:
            pickle.dump(self.policy_network, f)
        self.get_logger().info('Model saved successfully')

    def on_shutdown(self):
        """Cleanup function"""
        if self.training_mode:
            self.save_model()
```

## Safety and Error Handling

### Fail-Safe Mechanisms

Critical for humanoid robots operating in human environments:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Time
import time
from enum import Enum

class SafetyLevel(Enum):
    NORMAL = 1
    WARNING = 2
    DANGER = 3
    EMERGENCY = 4

class SafeActionExecutionNode(Node):
    def __init__(self):
        super().__init__('safe_action_execution_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, 'safety_status', 10)

        # Subscriptions
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.01, self.safety_check)  # 100Hz safety check

        # Robot state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_orientation = None
        self.imu_angular_velocity = None

        # Safety parameters
        self.joint_position_limits = {
            'hip_joint': (-1.5, 1.5),
            'knee_joint': (0.0, 2.5),
            'ankle_joint': (-0.8, 0.8),
            'shoulder_joint': (-2.0, 2.0),
            'elbow_joint': (0.0, 2.5)
        }
        self.joint_velocity_limit = 5.0  # rad/s
        self.tilt_threshold = 0.5  # rad (about 28 degrees)
        self.angular_velocity_threshold = 1.0  # rad/s

        # Safety state
        self.safety_level = SafetyLevel.NORMAL
        self.emergency_stop_active = False
        self.last_normal_command = Twist()
        self.safety_engaged_time = None

        # Action execution state
        self.active_action = None
        self.action_start_time = None

    def joint_callback(self, msg):
        """Update joint state with safety checks"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos = msg.position[i]
                self.joint_positions[name] = pos

                # Check position limits
                if name in self.joint_position_limits:
                    min_limit, max_limit = self.joint_position_limits[name]
                    if pos < min_limit or pos > max_limit:
                        self.get_logger().error(f'JOINT LIMIT EXCEEDED: {name} = {pos}')
                        self.trigger_safety_response(SafetyLevel.DANGER)

            if i < len(msg.velocity):
                vel = msg.velocity[i]
                self.joint_velocities[name] = vel

                # Check velocity limits
                if abs(vel) > self.joint_velocity_limit:
                    self.get_logger().error(f'JOINT VELOCITY EXCEEDED: {name} = {vel}')
                    self.trigger_safety_response(SafetyLevel.WARNING)

    def imu_callback(self, msg):
        """Update IMU state with safety checks"""
        self.imu_orientation = [
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ]
        self.imu_angular_velocity = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]

        # Check orientation limits
        if self.imu_orientation:
            roll, pitch, _ = self.quaternion_to_rpy(self.imu_orientation)
            if abs(roll) > self.tilt_threshold or abs(pitch) > self.tilt_threshold:
                self.get_logger().error(f'DANGEROUS TILT DETECTED: roll={roll}, pitch={pitch}')
                self.trigger_safety_response(SafetyLevel.DANGER)

        # Check angular velocity limits
        if self.imu_angular_velocity:
            max_ang_vel = max(abs(v) for v in self.imu_angular_velocity)
            if max_ang_vel > self.angular_velocity_threshold:
                self.get_logger().error(f'DANGEROUS ANGULAR VELOCITY: {max_ang_vel}')
                self.trigger_safety_response(SafetyLevel.WARNING)

    def quaternion_to_rpy(self, quat):
        """Convert quaternion to roll-pitch-yaw (simplified)"""
        x, y, z, w = quat
        # Simplified conversion for safety checks
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def safety_check(self):
        """Main safety monitoring function"""
        current_time = time.time()

        # If emergency stop is active, only allow safe actions
        if self.emergency_stop_active:
            self.publish_safe_command()
            return

        # Check for safety conditions
        new_safety_level = self.assess_safety_level()

        # Update safety level and trigger appropriate response
        if new_safety_level != self.safety_level:
            self.safety_level = new_safety_level

            # Publish safety status
            status_msg = String()
            status_msg.data = f"safety_level:{self.safety_level.name.lower()}"
            self.safety_status_pub.publish(status_msg)

        # Take action based on safety level
        if self.safety_level == SafetyLevel.EMERGENCY:
            self.trigger_emergency_stop()
        elif self.safety_level == SafetyLevel.DANGER:
            self.reduce_speed_safely()
        elif self.safety_level == SafetyLevel.WARNING:
            self.caution_mode()
        else:  # NORMAL
            self.normal_operation()

    def assess_safety_level(self) -> SafetyLevel:
        """Assess current safety level based on sensor data"""
        # Check for extreme joint positions (immediate danger)
        for joint_name, pos in self.joint_positions.items():
            if joint_name in self.joint_position_limits:
                min_limit, max_limit = self.joint_position_limits[joint_name]
                if pos < min_limit * 1.1 or pos > max_limit * 1.1:  # 10% beyond limits
                    return SafetyLevel.EMERGENCY

        # Check for dangerous tilt
        if self.imu_orientation:
            roll, pitch, _ = self.quaternion_to_rpy(self.imu_orientation)
            if abs(roll) > self.tilt_threshold * 1.5 or abs(pitch) > self.tilt_threshold * 1.5:
                return SafetyLevel.EMERGENCY

        # Check for extreme angular velocities
        if self.imu_angular_velocity:
            max_ang_vel = max(abs(v) for v in self.imu_angular_velocity)
            if max_ang_vel > self.angular_velocity_threshold * 2.0:
                return SafetyLevel.EMERGENCY

        # Check for approaching limits
        for joint_name, pos in self.joint_positions.items():
            if joint_name in self.joint_position_limits:
                min_limit, max_limit = self.joint_position_limits[joint_name]
                tolerance = (max_limit - min_limit) * 0.1  # 10% tolerance
                if pos < min_limit + tolerance or pos > max_limit - tolerance:
                    return SafetyLevel.DANGER

        # Check for dangerous tilt approaching
        if self.imu_orientation:
            roll, pitch, _ = self.quaternion_to_rpy(self.imu_orientation)
            if abs(roll) > self.tilt_threshold * 0.8 or abs(pitch) > self.tilt_threshold * 0.8:
                return SafetyLevel.DANGER

        # Check for high joint velocities
        for joint_name, vel in self.joint_velocities.items():
            if abs(vel) > self.joint_velocity_limit * 0.8:
                return SafetyLevel.WARNING

        # Check for high angular velocities
        if self.imu_angular_velocity:
            max_ang_vel = max(abs(v) for v in self.imu_angular_velocity)
            if max_ang_vel > self.angular_velocity_threshold * 0.8:
                return SafetyLevel.WARNING

        # Normal operation
        return SafetyLevel.NORMAL

    def trigger_safety_response(self, level: SafetyLevel):
        """Trigger appropriate safety response"""
        if level.value > self.safety_level.value:
            self.safety_level = level

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.safety_engaged_time = time.time()
            self.get_logger().fatal('EMERGENCY STOP ACTIVATED')

            # Publish emergency stop signal
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)

    def reduce_speed_safely(self):
        """Reduce robot speed while maintaining control"""
        # In a real implementation, this would gradually reduce speed
        # For now, we'll just log the action
        if self.safety_level == SafetyLevel.DANGER:
            self.get_logger().warn('SAFETY MODE: Reducing speed')

    def caution_mode(self):
        """Operate in caution mode with reduced speeds"""
        if self.safety_level == SafetyLevel.WARNING:
            self.get_logger().info('CAUTION MODE: Operating with reduced speed')

    def normal_operation(self):
        """Normal operation mode"""
        if self.safety_level == SafetyLevel.NORMAL:
            # Resume normal operation
            pass

    def publish_safe_command(self):
        """Publish safe (zero) command"""
        safe_cmd = Twist()
        self.cmd_pub.publish(safe_cmd)

    def normal_operation(self):
        """Resume normal operation (called when safety clears)"""
        if self.emergency_stop_active:
            # Safety system is engaged, don't resume normal operation
            return

        # In a real implementation, this would resume normal action execution
        # For this example, we'll just log
        pass

    def reset_emergency_stop(self):
        """Reset emergency stop (should be called externally in safe conditions)"""
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            self.safety_level = SafetyLevel.NORMAL
            self.get_logger().info('Emergency stop reset - returning to normal operation')

            # Publish that emergency stop is cleared
            stop_msg = Bool()
            stop_msg.data = False
            self.emergency_stop_pub.publish(stop_msg)
```

## Performance Considerations

### Efficient Action Execution

For real-time humanoid control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from threading import Thread, Lock
from collections import deque
import time
import numpy as np

class EfficientActionExecutionNode(Node):
    def __init__(self):
        super().__init__('efficient_action_execution_node')

        # Use appropriate QoS for performance
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers with performance-optimized QoS
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', qos_profile)
        self.body_cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        # Subscriptions with performance-optimized QoS
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos_profile
        )

        # High-frequency control timer (1000Hz for critical control)
        self.control_timer = self.create_timer(0.001, self.high_freq_control_cycle)

        # Lower-frequency action planning (100Hz for action planning)
        self.action_timer = self.create_timer(0.01, self.action_planning_cycle)

        # State with minimal memory allocation
        self.joint_positions = {}  # Use dictionary for sparse updates
        self.joint_velocities = {}
        self.desired_positions = {}
        self.desired_velocities = {}

        # Pre-allocated arrays to avoid allocation during control
        self.control_command = Float64MultiArray()
        self.joint_names_ordered = ['hip_joint', 'knee_joint', 'ankle_joint',
                                   'shoulder_joint', 'elbow_joint', 'wrist_joint']
        self.position_buffer = [0.0] * len(self.joint_names_ordered)
        self.velocity_buffer = [0.0] * len(self.joint_names_ordered)

        # Action execution queues
        self.action_queue = deque()
        self.active_action = None

        # Timing statistics
        self.control_cycle_times = deque(maxlen=100)
        self.action_cycle_times = deque(maxlen=50)

    def joint_state_callback(self, msg):
        """High-performance joint state callback"""
        # Update only the joints that have new values (sparse update)
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def high_freq_control_cycle(self):
        """High-frequency control cycle (1000Hz)"""
        start_time = time.perf_counter()

        # Execute current action with minimal computation
        if self.active_action:
            # Use pre-allocated buffers
            cmd_data = self.execute_active_action()
            if cmd_data:
                # Publish without creating new objects unnecessarily
                self.control_command.data = cmd_data
                self.joint_cmd_pub.publish(self.control_command)

        # Track performance
        end_time = time.perf_counter()
        self.control_cycle_times.append(end_time - start_time)

        # Log performance warnings
        if len(self.control_cycle_times) == 100:  # Buffer full
            avg_time = sum(self.control_cycle_times) / len(self.control_cycle_times)
            if avg_time > 0.0005:  # More than half our deadline
                self.get_logger().warning(f'Control cycle averaging {avg_time*1000:.2f}ms (deadline: 1.0ms)')

    def action_planning_cycle(self):
        """Action planning cycle (100Hz)"""
        start_time = time.perf_counter()

        # Process action queue
        if self.active_action is None and self.action_queue:
            self.active_action = self.action_queue.popleft()

        # Update active action
        if self.active_action:
            self.update_active_action()

        # Track performance
        end_time = time.perf_counter()
        self.action_cycle_times.append(end_time - start_time)

    def execute_active_action(self):
        """Execute the active action with optimized computation"""
        if not self.active_action:
            return None

        action_type = self.active_action.get('type', 'idle')

        if action_type == 'move_to_pose':
            return self.execute_move_to_pose_action(self.active_action)
        elif action_type == 'follow_trajectory':
            return self.execute_follow_trajectory_action(self.active_action)
        elif action_type == 'balance':
            return self.execute_balance_action(self.active_action)
        else:
            return None

    def execute_move_to_pose_action(self, action):
        """Execute move-to-pose action efficiently"""
        target_pose = action.get('target', {})
        joint_targets = target_pose.get('joints', {})

        # Update desired positions with minimal computation
        for joint_name, target_pos in joint_targets.items():
            if joint_name in self.joint_positions:
                # Simple PD control (efficient computation)
                current_pos = self.joint_positions[joint_name]

                # Compute error and simple control (no integrator for speed)
                error = target_pos - current_pos
                control_output = 100.0 * error  # Simple P-only control for speed

                # Update desired position
                self.desired_positions[joint_name] = current_pos + control_output * 0.001  # dt = 1ms

        # Pack results into pre-allocated buffer
        for i, joint_name in enumerate(self.joint_names_ordered):
            self.position_buffer[i] = self.desired_positions.get(joint_name,
                                                               self.joint_positions.get(joint_name, 0.0))

        return self.position_buffer.copy()  # Return copy to avoid reference issues

    def execute_follow_trajectory_action(self, action):
        """Execute trajectory following action efficiently"""
        # In a real implementation, this would interpolate through trajectory points
        # For this example, we'll just return the current position buffer
        return self.position_buffer.copy()

    def execute_balance_action(self, action):
        """Execute balance control action efficiently"""
        # In a real implementation, this would run balance control algorithms
        # For this example, return a stable stance
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Stable stance

    def update_active_action(self):
        """Update the active action state"""
        if not self.active_action:
            return

        # Check completion conditions
        completion_check = self.active_action.get('completion_check', 'none')

        if completion_check == 'position_reached':
            if self.check_position_reached(self.active_action):
                self.complete_active_action()
        elif completion_check == 'time_elapsed':
            start_time = self.active_action.get('start_time', 0)
            duration = self.active_action.get('duration', 1.0)
            current_time = time.time()

            if current_time - start_time > duration:
                self.complete_active_action()

    def check_position_reached(self, action):
        """Check if target position has been reached"""
        target_pose = action.get('target', {})
        joint_targets = target_pose.get('joints', {})

        tolerance = action.get('tolerance', 0.01)  # 10mrad tolerance

        for joint_name, target_pos in joint_targets.items():
            current_pos = self.joint_positions.get(joint_name, 0.0)
            if abs(current_pos - target_pos) > tolerance:
                return False

        return True

    def complete_active_action(self):
        """Complete the active action"""
        if self.active_action:
            self.get_logger().debug(f'Action completed: {self.active_action.get("type", "unknown")}')
            self.active_action = None

    def queue_action(self, action):
        """Queue an action for execution"""
        action['start_time'] = time.time()
        self.action_queue.append(action)
        self.get_logger().info(f'Action queued: {action.get("type", "unknown")}')

    def get_performance_stats(self):
        """Get performance statistics"""
        if self.control_cycle_times:
            avg_control_time = sum(self.control_cycle_times) / len(self.control_cycle_times)
            max_control_time = max(self.control_cycle_times)
        else:
            avg_control_time = max_control_time = 0.0

        if self.action_cycle_times:
            avg_action_time = sum(self.action_cycle_times) / len(self.action_cycle_times)
        else:
            avg_action_time = 0.0

        return {
            'control_cycle_avg_ms': avg_control_time * 1000,
            'control_cycle_max_ms': max_control_time * 1000,
            'action_cycle_avg_ms': avg_action_time * 1000,
            'action_queue_size': len(self.action_queue)
        }
```

## Integration with Higher-Level Systems

### Decision Logic Coordination

Integrating with mission planning and other AI systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import json
import uuid
from datetime import datetime
from typing import Dict, Any, Optional

class DecisionLogicCoordinatorNode(Node):
    def __init__(self):
        super().__init__('decision_logic_coordinator_node')

        # Publishers for coordination
        self.task_status_pub = self.create_publisher(String, 'task_status', 10)
        self.coordination_pub = self.create_publisher(String, 'coordination_messages', 10)

        # Subscriptions for coordination
        self.task_assignment_sub = self.create_subscription(
            String, 'task_assignments', self.task_assignment_callback, 10
        )
        self.status_request_sub = self.create_subscription(
            String, 'status_requests', self.status_request_callback, 10
        )

        # Timer for coordination cycle
        self.coordination_timer = self.create_timer(1.0, self.coordination_cycle)

        # Coordinator state
        self.assigned_tasks = {}
        self.task_results = {}
        self.system_status = "operational"
        self.last_coordination_time = datetime.now()

        # Callback groups for multithreading
        self.task_cb_group = MutuallyExclusiveCallbackGroup()
        self.coordination_cb_group = MutuallyExclusiveCallbackGroup()

    def task_assignment_callback(self, msg):
        """Handle task assignments from higher-level systems"""
        try:
            task_data = json.loads(msg.data)
            task_id = task_data.get('task_id', str(uuid.uuid4()))
            task_type = task_data.get('type', 'unknown')
            task_params = task_data.get('parameters', {})

            # Register the new task
            self.assigned_tasks[task_id] = {
                'type': task_type,
                'parameters': task_params,
                'status': 'received',
                'assigned_time': datetime.now().isoformat()
            }

            self.get_logger().info(f'Received task assignment: {task_type} (ID: {task_id})')

            # Publish acknowledgment
            ack_msg = String()
            ack_msg.data = json.dumps({
                'type': 'task_acknowledgment',
                'task_id': task_id,
                'status': 'received',
                'timestamp': datetime.now().isoformat()
            })
            self.coordination_pub.publish(ack_msg)

            # Start task execution
            self.execute_assigned_task(task_id)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in task assignment: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing task assignment: {e}')

    def status_request_callback(self, msg):
        """Handle status requests from coordination system"""
        try:
            request_data = json.loads(msg.data)
            request_type = request_data.get('type', 'status_query')

            if request_type == 'status_query':
                status_response = self.generate_status_report()

                response_msg = String()
                response_msg.data = json.dumps(status_response)
                self.coordination_pub.publish(response_msg)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in status request: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing status request: {e}')

    def generate_status_report(self) -> Dict[str, Any]:
        """Generate comprehensive status report"""
        return {
            'type': 'status_report',
            'system_id': self.get_name(),
            'status': self.system_status,
            'active_tasks': len([t for t in self.assigned_tasks.values() if t['status'] in ['executing', 'received']]),
            'completed_tasks': len([t for t in self.task_results.values()]),
            'last_coordination_time': self.last_coordination_time.isoformat(),
            'timestamp': datetime.now().isoformat(),
            'resource_usage': self.get_resource_usage()
        }

    def get_resource_usage(self) -> Dict[str, float]:
        """Get current resource usage"""
        # In a real implementation, this would query system resources
        # For this example, return dummy values
        return {
            'cpu_percent': 25.0,
            'memory_percent': 45.0,
            'disk_percent': 30.0,
            'network_io': 1.2  # MB/s
        }

    def execute_assigned_task(self, task_id: str):
        """Execute an assigned task"""
        if task_id not in self.assigned_tasks:
            return

        task = self.assigned_tasks[task_id]
        task_type = task['type']

        # Update task status
        task['status'] = 'executing'
        task['start_time'] = datetime.now().isoformat()

        # Publish task status
        self.publish_task_status(task_id, 'executing')

        # Execute based on task type
        success = False
        result = None

        if task_type == 'navigation':
            success, result = self.execute_navigation_task(task)
        elif task_type == 'manipulation':
            success, result = self.execute_manipulation_task(task)
        elif task_type == 'inspection':
            success, result = self.execute_inspection_task(task)
        elif task_type == 'monitoring':
            success, result = self.execute_monitoring_task(task)
        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            success = False
            result = {'error': f'Unknown task type: {task_type}'}

        # Complete the task
        self.complete_task(task_id, success, result)

    def execute_navigation_task(self, task: Dict[str, Any]) -> tuple[bool, Dict[str, Any]]:
        """Execute navigation task"""
        try:
            target_pose = task['parameters'].get('target_pose', {})
            self.get_logger().info(f'Executing navigation to: {target_pose}')

            # In a real implementation, this would call navigation action server
            # For this example, simulate execution
            import time
            time.sleep(0.5)  # Simulate navigation time

            # Simulate success
            result = {
                'success': True,
                'final_pose': target_pose,
                'distance_traveled': 2.5,
                'execution_time': 0.5
            }
            return True, result

        except Exception as e:
            self.get_logger().error(f'Navigation task failed: {e}')
            return False, {'error': str(e)}

    def execute_manipulation_task(self, task: Dict[str, Any]) -> tuple[bool, Dict[str, Any]]:
        """Execute manipulation task"""
        try:
            object_id = task['parameters'].get('object_id', '')
            action = task['parameters'].get('action', 'grasp')
            self.get_logger().info(f'Executing {action} on object: {object_id}')

            # In a real implementation, this would call manipulation action server
            # For this example, simulate execution
            import time
            time.sleep(0.3)  # Simulate manipulation time

            # Simulate success
            result = {
                'success': True,
                'object_manipulated': object_id,
                'action_performed': action,
                'execution_time': 0.3
            }
            return True, result

        except Exception as e:
            self.get_logger().error(f'Manipulation task failed: {e}')
            return False, {'error': str(e)}

    def execute_inspection_task(self, task: Dict[str, Any]) -> tuple[bool, Dict[str, Any]]:
        """Execute inspection task"""
        try:
            area = task['parameters'].get('area', 'unknown')
            self.get_logger().info(f'Inspecting area: {area}')

            # In a real implementation, this would use sensors to inspect
            # For this example, simulate execution
            import time
            time.sleep(0.4)  # Simulate inspection time

            # Simulate inspection results
            result = {
                'success': True,
                'area_inspected': area,
                'findings': ['no_anomalies'],
                'images_captured': 3,
                'execution_time': 0.4
            }
            return True, result

        except Exception as e:
            self.get_logger().error(f'Inspection task failed: {e}')
            return False, {'error': str(e)}

    def execute_monitoring_task(self, task: Dict[str, Any]) -> tuple[bool, Dict[str, Any]]:
        """Execute monitoring task"""
        try:
            duration = task['parameters'].get('duration', 60)  # seconds
            self.get_logger().info(f'Monitoring for {duration} seconds')

            # In a real implementation, this would run in background
            # For this example, simulate execution
            import time
            time.sleep(0.1)  # Simulate setup time

            # Simulate monitoring results
            result = {
                'success': True,
                'monitoring_duration': duration,
                'events_detected': 0,
                'status_updates': 5,
                'execution_time': 0.1
            }
            return True, result

        except Exception as e:
            self.get_logger().error(f'Monitoring task failed: {e}')
            return False, {'error': str(e)}

    def complete_task(self, task_id: str, success: bool, result: Dict[str, Any]):
        """Complete a task and update status"""
        if task_id not in self.assigned_tasks:
            return

        task = self.assigned_tasks[task_id]

        # Update task status
        task['status'] = 'completed' if success else 'failed'
        task['end_time'] = datetime.now().isoformat()
        task['result'] = result

        # Store result
        self.task_results[task_id] = result

        # Publish completion status
        self.publish_task_status(task_id, task['status'])

        # Publish result to coordination system
        result_msg = String()
        result_msg.data = json.dumps({
            'type': 'task_result',
            'task_id': task_id,
            'success': success,
            'result': result,
            'timestamp': datetime.now().isoformat()
        })
        self.coordination_pub.publish(result_msg)

        # Clean up completed task after some time
        import threading
        cleanup_timer = threading.Timer(30.0, self.cleanup_completed_task, args=[task_id])
        cleanup_timer.start()

    def cleanup_completed_task(self, task_id: str):
        """Clean up completed task from memory"""
        if task_id in self.assigned_tasks:
            del self.assigned_tasks[task_id]

    def publish_task_status(self, task_id: str, status: str):
        """Publish task status update"""
        status_msg = String()
        status_msg.data = json.dumps({
            'type': 'task_status_update',
            'task_id': task_id,
            'status': status,
            'timestamp': datetime.now().isoformat()
        })
        self.task_status_pub.publish(status_msg)

    def coordination_cycle(self):
        """Regular coordination cycle"""
        self.last_coordination_time = datetime.now()

        # Check for timeout tasks
        current_time = datetime.now()
        for task_id, task in list(self.assigned_tasks.items()):
            if task['status'] == 'executing':
                start_time_str = task.get('start_time', current_time.isoformat())
                start_time = datetime.fromisoformat(start_time_str.replace('Z', '+00:00'))

                # Check if task has timed out (after 5 minutes for this example)
                if (current_time - start_time).total_seconds() > 300:
                    self.get_logger().warning(f'Task timed out: {task_id}')
                    self.complete_task(task_id, False, {'error': 'Task timed out'})

        # Publish periodic status
        if len(self.assigned_tasks) > 0:
            status_msg = String()
            status_msg.data = f"active_tasks: {len([t for t in self.assigned_tasks.values() if t['status'] == 'executing'])}"
            self.coordination_pub.publish(status_msg)

    def get_task_progress(self, task_id: str) -> Optional[float]:
        """Get progress of a specific task"""
        # In a real implementation, this would track actual progress
        # For this example, return based on status
        if task_id in self.assigned_tasks:
            status = self.assigned_tasks[task_id]['status']
            if status == 'completed':
                return 1.0
            elif status == 'executing':
                return 0.5  # Halfway
            else:
                return 0.0
        return None
```

## Summary

Action execution from decision logic is a critical component of autonomous humanoid robots, requiring careful consideration of safety, efficiency, and coordination. The process involves multiple layers:

1. **Decision Making**: High-level AI systems generate goals and plans
2. **Action Planning**: Converting goals into executable action sequences
3. **Execution Control**: Low-level control systems execute actions safely
4. **Feedback Integration**: Monitoring and adapting to environmental changes
5. **Coordination**: Integration with higher-level systems and other robots

Successful implementation requires balancing real-time performance with safety considerations, using appropriate architectures for the specific application, and ensuring robust error handling and fail-safe mechanisms.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the decision-to-action pipeline in humanoid robotics. What are the key components and how do they interact?

2. Compare rule-based decision systems with learning-based decision systems. What are the advantages and disadvantages of each approach for humanoid robotics?

### Application Questions
3. Design an action execution system for a humanoid robot that needs to:
   - Navigate through a dynamic environment with moving obstacles
   - Pick up objects based on visual recognition
   - Respond to voice commands from humans
   - Maintain balance while performing tasks
   Specify the decision logic, action planning, and execution control needed.

4. How would you implement a hierarchical action execution system that can handle both high-level mission planning and low-level safety responses simultaneously?

### Hands-on Practice
5. Create a simple action execution node that accepts goal poses and executes navigation actions with proper feedback and error handling.

6. Implement a rule-based decision system that chooses between different behaviors based on sensor inputs (obstacle detection, target recognition, etc.).

### Critical Thinking
7. What are the main challenges in ensuring real-time performance for action execution in humanoid robots? How would you address timing constraints and computational complexity?

8. How would you design an action execution system that can gracefully handle partial failures (e.g., one joint fails but others continue operating) while maintaining safety?
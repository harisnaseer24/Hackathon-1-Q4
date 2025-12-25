#!/usr/bin/env python3

"""
Sample AI Decision Logic Implementations for Chapter 3
This file demonstrates various AI decision-making approaches for humanoid robotics
following concepts from Chapter 3: Bridging AI Agents to ROS Controllers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, LaserScan, Image
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass
from enum import Enum
import time
import json


class DecisionType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    BALANCE = "balance"
    INTERACTION = "interaction"
    MONITORING = "monitoring"


@dataclass
class DecisionContext:
    """Context for decision making"""
    sensor_data: Dict[str, Any]
    robot_state: Dict[str, Any]
    environment_state: Dict[str, Any]
    goals: List[Any]
    constraints: Dict[str, Any]
    timestamp: float


@dataclass
class DecisionResult:
    """Result of a decision process"""
    decision_type: DecisionType
    action: Any
    confidence: float
    reasoning_trace: List[str]
    execution_plan: List[Any]


class AIDecisionEngineNode(Node):
    """
    AI Decision Engine demonstrating multiple decision-making approaches
    """

    def __init__(self):
        super().__init__('ai_decision_engine_node')

        # Publishers
        self.decision_pub = self.create_publisher(String, 'ai_decision', 10)
        self.action_pub = self.create_publisher(Float64MultiArray, 'ai_actions', 10)
        self.debug_pub = self.create_publisher(String, 'decision_debug', 10)

        # Subscriptions
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # Timer for decision making
        self.decision_timer = self.create_timer(0.2, self.decision_cycle)  # 5Hz decision cycle

        # State variables
        self.joint_states = {}
        self.scan_data = None
        self.imu_data = None
        self.current_goal = None
        self.robot_pose = None

        # Decision engines
        self.reactive_engine = ReactiveDecisionEngine()
        self.planning_engine = PlanningDecisionEngine()
        self.learning_engine = LearningDecisionEngine()
        self.hybrid_engine = HybridDecisionEngine()

        # Decision history for learning
        self.decision_history = []

        self.get_logger().info('AI Decision Engine initialized')

    def joint_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def scan_callback(self, msg):
        """Update laser scan information"""
        self.scan_data = np.array(msg.ranges)

    def imu_callback(self, msg):
        """Update IMU information"""
        self.imu_data = msg

    def goal_callback(self, msg):
        """Update goal information"""
        self.current_goal = msg.pose

    def decision_cycle(self):
        """Main decision cycle"""
        # Gather current context
        context = self.gather_decision_context()

        # Select appropriate decision engine based on situation
        if self.is_emergency_situation(context):
            decision_result = self.reactive_engine.make_decision(context)
        elif self.requires_long_term_planning(context):
            decision_result = self.planning_engine.make_decision(context)
        elif self.can_learn_from_this_situation(context):
            decision_result = self.learning_engine.make_decision(context)
        else:
            decision_result = self.hybrid_engine.make_decision(context)

        # Execute decision
        self.execute_decision(decision_result)

        # Store for learning and analysis
        self.decision_history.append(decision_result)

    def gather_decision_context(self) -> DecisionContext:
        """Gather all relevant information for decision making"""
        return DecisionContext(
            sensor_data={
                'scan': self.scan_data,
                'imu': self.imu_data,
                'joints': self.joint_states
            },
            robot_state={
                'pose': self.robot_pose,
                'goal': self.current_goal
            },
            environment_state=self.understand_environment(),
            goals=self.get_current_goals(),
            constraints=self.get_constraints(),
            timestamp=time.time()
        )

    def understand_environment(self) -> Dict[str, Any]:
        """Understand the current environment state"""
        env_state = {}

        if self.scan_data is not None:
            # Analyze obstacles
            valid_ranges = [r for r in self.scan_data if not (np.isnan(r) or np.isinf(r))]
            if valid_ranges:
                env_state['obstacle_distance'] = min(valid_ranges)
                env_state['free_space_ratio'] = len([r for r in valid_ranges if r > 1.0]) / len(valid_ranges)

        if self.imu_data is not None:
            # Analyze balance state
            roll, pitch, _ = self.quaternion_to_rpy([
                self.imu_data.orientation.x,
                self.imu_data.orientation.y,
                self.imu_data.orientation.z,
                self.imu_data.orientation.w
            ])
            env_state['tilt'] = {'roll': roll, 'pitch': pitch}

        return env_state

    def get_current_goals(self) -> List[Any]:
        """Get current goals for the robot"""
        goals = []
        if self.current_goal is not None:
            goals.append(('navigate_to_pose', self.current_goal))
        return goals

    def get_constraints(self) -> Dict[str, Any]:
        """Get current operational constraints"""
        return {
            'safety': {'max_speed': 0.5, 'min_distance': 0.3},
            'balance': {'max_tilt': 0.3},
            'power': {'remaining': 1.0}  # 100% battery
        }

    def is_emergency_situation(self, context: DecisionContext) -> bool:
        """Check if current situation requires immediate reactive response"""
        env_state = context.environment_state
        sensor_data = context.sensor_data

        # Check for imminent collision
        if 'obstacle_distance' in env_state and env_state['obstacle_distance'] < 0.2:
            return True

        # Check for dangerous tilt
        if 'tilt' in env_state and (abs(env_state['tilt']['roll']) > 0.5 or abs(env_state['tilt']['pitch']) > 0.5):
            return True

        return False

    def requires_long_term_planning(self, context: DecisionContext) -> bool:
        """Check if situation requires long-term planning"""
        goals = context.goals
        return len(goals) > 0 and any(g[0] == 'navigate_to_pose' for g in goals)

    def can_learn_from_this_situation(self, context: DecisionContext) -> bool:
        """Check if situation is suitable for learning-based decisions"""
        # For this example, return True if we have sufficient sensor data
        return context.sensor_data.get('scan') is not None

    def execute_decision(self, decision_result: DecisionResult):
        """Execute the decision result"""
        # Publish decision for debugging
        decision_msg = String()
        decision_msg.data = json.dumps({
            'type': decision_result.decision_type.value,
            'action': str(decision_result.action),
            'confidence': decision_result.confidence,
            'timestamp': time.time()
        })
        self.decision_pub.publish(decision_msg)

        # Execute action based on type
        if decision_result.decision_type == DecisionType.NAVIGATION:
            self.execute_navigation_action(decision_result.action)
        elif decision_result.decision_type == DecisionType.BALANCE:
            self.execute_balance_action(decision_result.action)
        elif decision_result.decision_type == DecisionType.INTERACTION:
            self.execute_interaction_action(decision_result.action)

    def execute_navigation_action(self, action):
        """Execute navigation action"""
        if isinstance(action, Twist):
            # If action is already a Twist message, publish directly
            self.cmd_vel_pub.publish(action)
        else:
            # Convert action to appropriate format
            cmd = Twist()
            if isinstance(action, dict):
                cmd.linear.x = action.get('linear_x', 0.0)
                cmd.angular.z = action.get('angular_z', 0.0)
            self.cmd_vel_pub.publish(cmd)

    def execute_balance_action(self, action):
        """Execute balance action"""
        # In a real implementation, this would send joint commands
        # For this example, we'll just log the action
        self.get_logger().info(f'Balance action: {action}')

    def execute_interaction_action(self, action):
        """Execute interaction action"""
        # In a real implementation, this would control arms, head, etc.
        # For this example, we'll just log the action
        self.get_logger().info(f'Interaction action: {action}')


class ReactiveDecisionEngine:
    """
    Reactive decision engine that responds immediately to environmental changes
    Uses simple if-then rules for fast decision making
    """

    def __init__(self):
        self.reactive_rules = [
            self.emergency_stop_rule,
            self.obstacle_avoidance_rule,
            self.balance_correction_rule,
            self.goal_seeking_rule
        ]

    def make_decision(self, context: DecisionContext) -> DecisionResult:
        """Make reactive decision based on current context"""
        # Evaluate rules in order of priority
        for rule in self.reactive_rules:
            decision = rule(context)
            if decision is not None:
                return decision

        # Default: stop if no rule applies
        return DecisionResult(
            decision_type=DecisionType.NAVIGATION,
            action=Twist(),  # Stop
            confidence=0.5,
            reasoning_trace=["No reactive rule matched, stopping"],
            execution_plan=[]
        )

    def emergency_stop_rule(self, context: DecisionContext) -> Optional[DecisionResult]:
        """Rule for emergency stop when immediate danger detected"""
        env_state = context.environment_state
        sensor_data = context.sensor_data

        if 'obstacle_distance' in env_state and env_state['obstacle_distance'] < 0.2:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            return DecisionResult(
                decision_type=DecisionType.NAVIGATION,
                action=cmd,
                confidence=1.0,
                reasoning_trace=["Obstacle too close", "Emergency stop activated"],
                execution_plan=[]
            )

        if 'tilt' in env_state and (abs(env_state['tilt']['roll']) > 0.5 or abs(env_state['tilt']['pitch']) > 0.5):
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            return DecisionResult(
                decision_type=DecisionType.BALANCE,
                action=cmd,
                confidence=1.0,
                reasoning_trace=["Dangerous tilt detected", "Emergency stop for balance"],
                execution_plan=[]
            )

        return None

    def obstacle_avoidance_rule(self, context: DecisionContext) -> Optional[DecisionResult]:
        """Rule for avoiding obstacles"""
        sensor_data = context.sensor_data

        if sensor_data.get('scan') is not None:
            scan_data = sensor_data['scan']

            # Check front sectors for obstacles
            front_left_idx = len(scan_data) // 2 - 15
            front_center_idx = len(scan_data) // 2
            front_right_idx = len(scan_data) // 2 + 15

            if 0 <= front_left_idx < len(scan_data) and 0 <= front_right_idx < len(scan_data):
                front_left_dist = scan_data[front_left_idx]
                front_center_dist = scan_data[front_center_idx]
                front_right_dist = scan_data[front_right_idx]

                valid_distances = [d for d in [front_left_dist, front_center_dist, front_right_dist]
                                  if not (np.isnan(d) or np.isinf(d))]

                if valid_distances and min(valid_distances) < 0.5:  # Obstacle within 50cm
                    cmd = Twist()

                    # Determine turn direction based on clearance
                    left_clear = front_left_dist if not (np.isnan(front_left_dist) or np.isinf(front_left_dist)) else 1.0
                    right_clear = front_right_dist if not (np.isnan(front_right_dist) or np.isinf(front_right_dist)) else 1.0

                    if left_clear > right_clear:
                        cmd.angular.z = 0.3  # Turn left
                    else:
                        cmd.angular.z = -0.3  # Turn right

                    cmd.linear.x = 0.1  # Move slowly while turning

                    return DecisionResult(
                        decision_type=DecisionType.NAVIGATION,
                        action=cmd,
                        confidence=0.8,
                        reasoning_trace=[
                            f"Obstacle detected: left={left_clear:.2f}m, center={front_center_dist:.2f}m, right={right_clear:.2f}m",
                            "Avoiding obstacle"
                        ],
                        execution_plan=[]
                    )

        return None

    def balance_correction_rule(self, context: DecisionContext) -> Optional[DecisionResult]:
        """Rule for balance correction"""
        env_state = context.environment_state

        if 'tilt' in env_state:
            tilt = env_state['tilt']

            # If tilted beyond threshold, correct balance
            if abs(tilt['roll']) > 0.1 or abs(tilt['pitch']) > 0.1:
                cmd = Twist()

                # Generate corrective motion based on tilt
                cmd.angular.z = -tilt['roll'] * 2.0  # Counter-roll
                cmd.linear.x = -tilt['pitch'] * 0.5  # Counter-pitch

                return DecisionResult(
                    decision_type=DecisionType.BALANCE,
                    action=cmd,
                    confidence=0.7,
                    reasoning_trace=[
                        f"Tilt detected: roll={tilt['roll']:.3f}, pitch={tilt['pitch']:.3f}",
                        "Applying balance correction"
                    ],
                    execution_plan=[]
                )

        return None

    def goal_seeking_rule(self, context: DecisionContext) -> Optional[DecisionResult]:
        """Rule for moving toward goal"""
        robot_state = context.robot_state

        if robot_state.get('goal') is not None and robot_state.get('pose') is not None:
            goal = robot_state['goal']
            pose = robot_state['pose']

            # Calculate direction to goal
            dx = goal.position.x - pose.position.x
            dy = goal.position.y - pose.position.y
            distance_to_goal = math.sqrt(dx*dx + dy*dy)

            if distance_to_goal > 0.3:  # Beyond tolerance
                cmd = Twist()

                # Simple proportional navigation
                cmd.linear.x = min(0.3, distance_to_goal * 0.5)  # Max 0.3 m/s

                # Calculate angle to goal
                angle_to_goal = math.atan2(dy, dx)
                # Assuming robot orientation is in pose.orientation (simplified)
                # In real implementation, would use proper TF
                current_yaw = 0.0  # Simplified
                angle_error = angle_to_goal - current_yaw

                # Normalize angle
                while angle_error > math.pi:
                    angle_error -= 2 * math.pi
                while angle_error < -math.pi:
                    angle_error += 2 * math.pi

                cmd.angular.z = max(-0.5, min(0.5, angle_error * 1.0))  # Limit angular velocity

                return DecisionResult(
                    decision_type=DecisionType.NAVIGATION,
                    action=cmd,
                    confidence=0.6,
                    reasoning_trace=[
                        f"Moving toward goal at ({goal.position.x:.2f}, {goal.position.y:.2f})",
                        f"Distance: {distance_to_goal:.2f}m, angle error: {angle_error:.3f}rad"
                    ],
                    execution_plan=[]
                )

        return None


class PlanningDecisionEngine:
    """
    Planning-based decision engine that creates detailed action sequences
    Uses search algorithms and optimization for complex multi-step decisions
    """

    def __init__(self):
        self.planning_algorithms = {
            'a_star': self.a_star_plan,
            'rrt': self.rrt_plan,
            'potential_field': self.potential_field_plan
        }

    def make_decision(self, context: DecisionContext) -> DecisionResult:
        """Make planning-based decision"""
        robot_state = context.robot_state
        sensor_data = context.sensor_data

        if robot_state.get('goal') is not None:
            # Plan path to goal using appropriate algorithm
            plan = self.plan_path_to_goal(robot_state['pose'], robot_state['goal'], sensor_data)

            if plan:
                # Execute first step of plan
                next_action = self.get_next_action_from_plan(plan, robot_state['pose'])

                return DecisionResult(
                    decision_type=DecisionType.NAVIGATION,
                    action=next_action,
                    confidence=0.9,
                    reasoning_trace=[
                        "Generated path plan to goal",
                        f"Plan contains {len(plan)} waypoints",
                        "Executing next action in plan"
                    ],
                    execution_plan=plan
                )

        return DecisionResult(
            decision_type=DecisionType.NAVIGATION,
            action=Twist(),  # Stop if no plan could be made
            confidence=0.3,
            reasoning_trace=["Could not generate plan to goal"],
            execution_plan=[]
        )

    def plan_path_to_goal(self, start_pose, goal_pose, sensor_data) -> Optional[List[Pose]]:
        """Plan path from start to goal using sensor data"""
        # For this example, we'll use a simple A* implementation
        # In a real implementation, this would use more sophisticated planning
        if sensor_data.get('scan') is not None:
            # Use laser scan data to create a simple grid map for planning
            return self.a_star_plan(start_pose, goal_pose, sensor_data['scan'])

        return None

    def a_star_plan(self, start_pose, goal_pose, scan_data) -> Optional[List[Pose]]:
        """A* path planning implementation (simplified)"""
        # This is a simplified version - a real implementation would be more complex
        # Convert poses to grid coordinates and run A*

        # For this example, return a straight line as a simple "plan"
        plan = []

        # Start and goal positions
        start_x, start_y = start_pose.position.x, start_pose.position.y
        goal_x, goal_y = goal_pose.position.x, goal_pose.position.y

        # Generate waypoints along straight line
        steps = 10  # Number of waypoints
        for i in range(steps + 1):
            t = i / steps
            wp = Pose()
            wp.position.x = start_x + t * (goal_x - start_x)
            wp.position.y = start_y + t * (goal_y - start_y)
            # Set orientation toward goal
            angle = math.atan2(goal_y - start_y, goal_x - start_x)
            wp.orientation.z = math.sin(angle / 2)
            wp.orientation.w = math.cos(angle / 2)

            plan.append(wp)

        return plan

    def get_next_action_from_plan(self, plan: List[Pose], current_pose) -> Twist:
        """Get the next action from the plan"""
        if not plan:
            return Twist()  # Stop if no plan

        # Get next waypoint
        next_waypoint = plan[0]  # Simplified - would normally follow the plan

        # Generate command to move toward next waypoint
        cmd = Twist()

        # Calculate direction to waypoint
        dx = next_waypoint.position.x - current_pose.position.x
        dy = next_waypoint.position.y - current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Move toward waypoint
        cmd.linear.x = min(0.3, distance * 0.5)  # Proportional to distance

        # Calculate angular command
        angle_to_waypoint = math.atan2(dy, dx)
        # Simplified orientation calculation
        cmd.angular.z = angle_to_waypoint * 0.5

        return cmd


class LearningDecisionEngine:
    """
    Learning-based decision engine that improves decisions over time
    Uses reinforcement learning or other learning approaches
    """

    def __init__(self):
        # For this example, we'll use a simple Q-learning approach
        # In a real implementation, this might use deep RL
        self.q_table = {}
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.1

    def make_decision(self, context: DecisionContext) -> DecisionResult:
        """Make learning-based decision"""
        # Discretize the state space
        state = self.discretize_state(context)

        # Get possible actions
        possible_actions = self.get_possible_actions(context)

        # Choose action using epsilon-greedy
        if np.random.random() < self.exploration_rate:
            # Explore: choose random action
            action = np.random.choice(possible_actions)
        else:
            # Exploit: choose best known action
            action = self.get_best_action(state, possible_actions)

        # Create action command
        action_cmd = self.create_action_command(action, context)

        return DecisionResult(
            decision_type=DecisionType.NAVIGATION,
            action=action_cmd,
            confidence=0.7,  # Learning-based decisions have moderate confidence initially
            reasoning_trace=[
                f"Learning-based decision in state: {state}",
                f"Selected action: {action}",
                "Applying learned policy"
            ],
            execution_plan=[]
        )

    def discretize_state(self, context: DecisionContext) -> str:
        """Convert continuous state to discrete state representation"""
        env_state = context.environment_state
        robot_state = context.robot_state

        # Create a simple state representation
        obstacle_dist_category = "far"
        if 'obstacle_distance' in env_state:
            dist = env_state['obstacle_distance']
            if dist < 0.3:
                obstacle_dist_category = "very_close"
            elif dist < 0.6:
                obstacle_dist_category = "close"
            elif dist < 1.0:
                obstacle_dist_category = "near"

        goal_dist_category = "unknown"
        if robot_state.get('goal') and robot_state.get('pose'):
            goal_dx = robot_state['goal'].position.x - robot_state['pose'].position.x
            goal_dy = robot_state['goal'].position.y - robot_state['pose'].position.y
            goal_dist = math.sqrt(goal_dx*goal_dx + goal_dy*goal_dy)

            if goal_dist < 0.5:
                goal_dist_category = "very_close"
            elif goal_dist < 1.5:
                goal_dist_category = "close"
            elif goal_dist < 3.0:
                goal_dist_category = "near"
            else:
                goal_dist_category = "far"

        return f"obs_{obstacle_dist_category}_goal_{goal_dist_category}"

    def get_possible_actions(self, context: DecisionContext) -> List[str]:
        """Get possible actions for the current context"""
        return ["move_forward", "turn_left", "turn_right", "move_backward", "stop"]

    def get_best_action(self, state: str, possible_actions: List[str]) -> str:
        """Get the best action for a given state"""
        if state not in self.q_table:
            # Initialize Q-values for this state
            self.q_table[state] = {action: 0.0 for action in possible_actions}

        # Return action with highest Q-value
        q_values = self.q_table[state]
        return max(q_values, key=q_values.get)

    def create_action_command(self, action: str, context: DecisionContext) -> Twist:
        """Create Twist command from action string"""
        cmd = Twist()

        if action == "move_forward":
            cmd.linear.x = 0.3
        elif action == "turn_left":
            cmd.angular.z = 0.5
        elif action == "turn_right":
            cmd.angular.z = -0.5
        elif action == "move_backward":
            cmd.linear.x = -0.2
        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def update_q_value(self, state: str, action: str, reward: float, next_state: str):
        """Update Q-value based on experience"""
        if state not in self.q_table:
            self.q_table[state] = {}

        if action not in self.q_table[state]:
            self.q_table[state][action] = 0.0

        # Get max Q-value for next state
        if next_state in self.q_table:
            max_next_q = max(self.q_table[next_state].values()) if self.q_table[next_state] else 0.0
        else:
            max_next_q = 0.0

        # Update Q-value using Q-learning formula
        current_q = self.q_table[state][action]
        new_q = current_q + self.learning_rate * (
            reward + self.discount_factor * max_next_q - current_q
        )

        self.q_table[state][action] = new_q


class HybridDecisionEngine:
    """
    Hybrid decision engine that combines multiple approaches
    Uses meta-reasoning to select the best approach for each situation
    """

    def __init__(self):
        self.reactive_engine = ReactiveDecisionEngine()
        self.planning_engine = PlanningDecisionEngine()
        self.learning_engine = LearningDecisionEngine()

    def make_decision(self, context: DecisionContext) -> DecisionResult:
        """Make hybrid decision by selecting best approach for situation"""
        # Assess the situation to determine best decision approach
        situation_analysis = self.analyze_situation(context)

        # Select appropriate engine based on situation
        if situation_analysis['urgency'] > 0.8:
            # High urgency: use reactive
            return self.reactive_engine.make_decision(context)
        elif situation_analysis['complexity'] > 0.7 and situation_analysis['time_available'] > 2.0:
            # Complex task with time: use planning
            return self.planning_engine.make_decision(context)
        elif situation_analysis['novelty'] > 0.5:
            # Novel situation: use learning
            return self.learning_engine.make_decision(context)
        else:
            # Default: use reactive for safety
            return self.reactive_engine.make_decision(context)

    def analyze_situation(self, context: DecisionContext) -> Dict[str, float]:
        """Analyze the current situation to determine best decision approach"""
        env_state = context.environment_state
        robot_state = context.robot_state

        analysis = {
            'urgency': 0.0,
            'complexity': 0.0,
            'time_available': 5.0,  # Default assumption
            'novelty': 0.0
        }

        # Assess urgency based on immediate threats
        if 'obstacle_distance' in env_state and env_state['obstacle_distance'] < 0.3:
            analysis['urgency'] = 0.9
        elif 'tilt' in env_state and (abs(env_state['tilt']['roll']) > 0.3 or abs(env_state['tilt']['pitch']) > 0.3):
            analysis['urgency'] = 0.8
        else:
            analysis['urgency'] = 0.2  # Low urgency for normal navigation

        # Assess complexity based on goal and environment
        if robot_state.get('goal'):
            analysis['complexity'] = 0.6  # Navigation is moderately complex
        else:
            analysis['complexity'] = 0.3  # Simple monitoring/interaction

        # Assess novelty based on familiarity with environment
        # For this example, assume moderate novelty
        analysis['novelty'] = 0.4

        return analysis


def main(args=None):
    """
    Main function to run the AI decision logic examples
    """
    rclpy.init(args=args)

    decision_engine = AIDecisionEngineNode()

    try:
        print("AI Decision Logic Examples running...")
        print("This example demonstrates:")
        print("- Reactive decision making for immediate responses")
        print("- Planning-based decisions for complex tasks")
        print("- Learning-based decisions that improve over time")
        print("- Hybrid approaches that combine multiple strategies")
        print("Check the logs for decision-making behavior.")
        print("Press Ctrl+C to stop.")
        rclpy.spin(decision_engine)
    except KeyboardInterrupt:
        pass
    finally:
        decision_engine.destroy_node()
        rclpy.shutdown()
        print("AI Decision Logic Examples finished.")


if __name__ == '__main__':
    main()
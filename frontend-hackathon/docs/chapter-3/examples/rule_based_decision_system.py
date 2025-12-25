#!/usr/bin/env python3

"""
Rule-Based Decision System Example for Chapter 3
This example demonstrates a rule-based AI system that makes decisions based on sensor inputs
and executes corresponding robot actions, following concepts from Chapter 3: Bridging AI Agents to ROS Controllers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import Dict, List, Tuple, Optional
import math


class RuleBasedDecisionNode(Node):
    """
    A rule-based decision system that demonstrates deterministic AI behavior
    for humanoid robot control based on sensor inputs and predefined rules
    """

    def __init__(self):
        super().__init__('rule_based_decision_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'decision_status', 10)

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )

        # Timer for rule evaluation
        self.rule_timer = self.create_timer(0.1, self.rule_evaluation_cycle)  # 10Hz

        # Robot state
        self.scan_data = None
        self.odom_data = None
        self.current_goal = None
        self.robot_pose = None
        self.robot_twist = None

        # Decision state
        self.current_behavior = 'idle'  # idle, navigating, avoiding_obstacles, etc.
        self.behavior_start_time = None

        # Rule-based parameters
        self.safety_distance = 0.5  # meters
        self.goal_tolerance = 0.3   # meters
        self.obstacle_detection_angle = 30  # degrees (half-angle for front detection)

        # Rule definitions: (condition_func, action_func, priority)
        self.rules = [
            (self.is_emergency_situation, self.execute_emergency_stop, 100),
            (self.is_goal_reached, self.execute_goal_reached_behavior, 90),
            (self.is_obstacle_imminent, self.execute_emergency_avoidance, 80),
            (self.is_obstacle_nearby, self.execute_obstacle_avoidance, 70),
            (self.has_valid_goal, self.execute_navigation, 60),
            (self.is_robot_stuck, self.execute_escape_behavior, 50),
            (self.is_idle_state, self.execute_idle_behavior, 10),
        ]

        self.get_logger().info('Rule-based decision system initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg
        self.robot_pose = msg.pose.pose
        self.robot_twist = msg.twist.twist

    def goal_callback(self, msg):
        """Process new goal"""
        self.current_goal = msg.pose
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def rule_evaluation_cycle(self):
        """Main rule evaluation cycle"""
        # Evaluate rules in priority order and execute first matching rule
        for condition_func, action_func, priority in sorted(self.rules, key=lambda x: x[2], reverse=True):
            if condition_func():
                action_func()
                break  # Execute only the highest priority matching rule

    # Condition functions (return True if rule should fire)
    def is_emergency_situation(self) -> bool:
        """Check if there's an emergency situation requiring immediate stop"""
        if self.scan_data is None:
            return False

        # Check for very close obstacles (less than 20cm)
        valid_distances = [d for d in self.scan_data if not (np.isnan(d) or np.isinf(d))]
        if valid_distances and min(valid_distances) < 0.2:
            self.get_logger().warn('EMERGENCY: Very close obstacle detected')
            return True

        return False

    def is_goal_reached(self) -> bool:
        """Check if robot has reached the goal"""
        if self.current_goal is None or self.robot_pose is None:
            return False

        # Calculate distance to goal
        dx = self.current_goal.position.x - self.robot_pose.position.x
        dy = self.current_goal.position.y - self.robot_pose.position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)

        return distance_to_goal <= self.goal_tolerance

    def is_obstacle_imminent(self) -> bool:
        """Check if obstacle is imminent (requires immediate action)"""
        if self.scan_data is None:
            return False

        # Check front arc for imminent obstacles
        front_start_idx = int((0 - self.scan_angle_min) / self.scan_angle_increment - self.obstacle_detection_angle)
        front_end_idx = int((0 - self.scan_angle_min) / self.scan_angle_increment + self.obstacle_detection_angle)

        if 0 <= front_start_idx < len(self.scan_data) and 0 <= front_end_idx < len(self.scan_data):
            front_distances = self.scan_data[front_start_idx:front_end_idx]
            valid_distances = [d for d in front_distances if not (np.isnan(d) or np.isinf(d))]

            if valid_distances and min(valid_distances) < self.safety_distance * 0.5:  # Half safety distance
                return True

        return False

    def is_obstacle_nearby(self) -> bool:
        """Check if obstacle is nearby (requires avoidance action)"""
        if self.scan_data is None:
            return False

        # Check front arc for nearby obstacles
        front_start_idx = int((0 - self.scan_angle_min) / self.scan_angle_increment - self.obstacle_detection_angle)
        front_end_idx = int((0 - self.scan_angle_min) / self.scan_angle_increment + self.obstacle_detection_angle)

        if 0 <= front_start_idx < len(self.scan_data) and 0 <= front_end_idx < len(self.scan_data):
            front_distances = self.scan_data[front_start_idx:front_end_idx]
            valid_distances = [d for d in front_distances if not (np.isnan(d) or np.isinf(d))]

            if valid_distances and min(valid_distances) < self.safety_distance:
                return True

        return False

    def has_valid_goal(self) -> bool:
        """Check if there's a valid goal to pursue"""
        return self.current_goal is not None and self.robot_pose is not None

    def is_robot_stuck(self) -> bool:
        """Check if robot appears to be stuck"""
        if self.robot_twist is None:
            return False

        # Check if robot is not moving despite having a goal
        linear_speed = math.sqrt(
            self.robot_twist.linear.x**2 +
            self.robot_twist.linear.y**2 +
            self.robot_twist.linear.z**2
        )

        # If robot has a goal but isn't moving much for a while
        if (self.current_goal is not None and
            linear_speed < 0.05 and  # Moving very slowly
            self.current_behavior == 'navigating'):  # And supposed to be navigating
            return True

        return False

    def is_idle_state(self) -> bool:
        """Check if robot should be in idle state"""
        return True  # This is a catch-all rule with lowest priority

    # Action functions (execute the corresponding behavior)
    def execute_emergency_stop(self):
        """Execute emergency stop behavior"""
        cmd = Twist()
        # Stop all motion immediately
        self.cmd_vel_pub.publish(cmd)

        self.current_behavior = 'emergency_stop'
        self.publish_status('emergency_stop')
        self.get_logger().warn('EMERGENCY STOP EXECUTED')

    def execute_goal_reached_behavior(self):
        """Execute behavior when goal is reached"""
        cmd = Twist()
        # Stop at goal
        self.cmd_vel_pub.publish(cmd)

        self.current_behavior = 'goal_reached'
        self.publish_status('goal_reached')
        self.get_logger().info('Goal reached successfully')

        # Clear the goal
        self.current_goal = None

    def execute_emergency_avoidance(self):
        """Execute immediate obstacle avoidance"""
        cmd = Twist()

        # Immediate avoidance: stop and turn
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn right to avoid obstacle

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = 'avoiding_imminent_obstacle'
        self.publish_status('avoiding_imminent_obstacle')

    def execute_obstacle_avoidance(self):
        """Execute obstacle avoidance behavior"""
        cmd = Twist()

        if self.scan_data is not None:
            # Find direction with maximum clearance
            left_distances = self.scan_data[:len(self.scan_data)//3]
            right_distances = self.scan_data[2*len(self.scan_data)//3:]

            left_clearance = np.mean([d for d in left_distances if not (np.isnan(d) or np.isinf(d))] or [0.1])
            right_clearance = np.mean([d for d in right_distances if not (np.isnan(d) or np.isinf(d))] or [0.1])

            # Turn toward clearer direction
            if left_clearance > right_clearance:
                cmd.angular.z = 0.3  # Turn left
            else:
                cmd.angular.z = -0.3  # Turn right

            # Move forward slowly while avoiding
            cmd.linear.x = 0.1

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = 'avoiding_obstacles'
        self.publish_status('avoiding_obstacles')

    def execute_navigation(self):
        """Execute navigation toward goal"""
        cmd = Twist()

        if self.current_goal and self.robot_pose:
            # Simple proportional navigation
            dx = self.current_goal.position.x - self.robot_pose.position.x
            dy = self.current_goal.position.y - self.robot_pose.position.y

            # Calculate distance and angle to goal
            distance = math.sqrt(dx*dx + dy*dy)
            angle_to_goal = math.atan2(dy, dx)

            # Get current robot orientation (simplified - in real system would use proper TF)
            # For this example, assume robot is facing along x-axis initially
            robot_yaw = 0.0  # Would normally get from orientation in real system

            # Calculate angular error
            angle_error = angle_to_goal - robot_yaw
            # Normalize angle to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # Simple proportional control
            cmd.linear.x = min(0.3, distance * 0.5)  # Max 0.3 m/s, proportional to distance
            cmd.angular.z = angle_error * 1.0  # Proportional to angle error

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = 'navigating'
        self.publish_status('navigating_to_goal')

    def execute_escape_behavior(self):
        """Execute behavior when robot appears stuck"""
        cmd = Twist()

        # Try backing up slightly
        cmd.linear.x = -0.2  # Move backward
        cmd.angular.z = 0.1  # Add slight turn to break symmetry

        self.cmd_vel_pub.publish(cmd)
        self.current_behavior = 'escaping_stuck'
        self.publish_status('escaping_stuck')
        self.get_logger().info('Attempting to escape stuck condition')

    def execute_idle_behavior(self):
        """Execute idle behavior"""
        cmd = Twist()
        # Stop robot in idle state
        self.cmd_vel_pub.publish(cmd)

        if self.current_behavior != 'idle':
            # Only log when transitioning to idle
            self.get_logger().info('Entering idle state')

        self.current_behavior = 'idle'
        self.publish_status('idle')

    def publish_status(self, status: str):
        """Publish decision status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def get_robot_distance_to_goal(self) -> float:
        """Get distance from robot to current goal"""
        if self.current_goal is None or self.robot_pose is None:
            return float('inf')

        dx = self.current_goal.position.x - self.robot_pose.position.x
        dy = self.current_goal.position.y - self.robot_pose.position.y
        return math.sqrt(dx*dx + dy*dy)


class ComplexRuleBasedSystemNode(Node):
    """
    A more complex rule-based system with multiple decision layers
    and sophisticated rule combinations
    """

    def __init__(self):
        super().__init__('complex_rule_based_system_node')

        # Publishers and subscriptions
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.behavior_pub = self.create_publisher(String, 'active_behavior', 10)

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # Timer for complex rule evaluation
        self.complex_rule_timer = self.create_timer(0.05, self.complex_rule_cycle)  # 20Hz

        # State variables
        self.scan_data = None
        self.odom_data = None
        self.current_goal = None
        self.environment_state = {}
        self.decision_context = {}

        # Rule sets for different behaviors
        self.safety_rules = []
        self.navigation_rules = []
        self.task_rules = []
        self.social_rules = []  # For humanoid-specific social behaviors

        # Initialize rule sets
        self.initialize_rules()

    def initialize_rules(self):
        """Initialize all rule sets"""
        # Safety rules (highest priority)
        self.safety_rules = [
            {
                'name': 'collision_avoidance',
                'condition': self.check_collision_risk,
                'action': self.execute_collision_avoidance,
                'priority': 100,
                'preconditions': ['scan_available']
            },
            {
                'name': 'fall_prevention',
                'condition': self.check_balance_risk,
                'action': self.execute_balance_correction,
                'priority': 95,
                'preconditions': ['imu_available']
            }
        ]

        # Navigation rules
        self.navigation_rules = [
            {
                'name': 'goal_seeking',
                'condition': self.check_goal_progress,
                'action': self.execute_goal_navigation,
                'priority': 80,
                'preconditions': ['goal_available', 'pose_known']
            },
            {
                'name': 'path_following',
                'condition': self.check_path_deviation,
                'action': self.execute_path_following,
                'priority': 75,
                'preconditions': ['path_available']
            }
        ]

        # Task rules
        self.task_rules = [
            {
                'name': 'object_interaction',
                'condition': self.check_object_proximity,
                'action': self.execute_object_interaction,
                'priority': 70,
                'preconditions': ['object_detected']
            }
        ]

        # Social rules (humanoid-specific)
        self.social_rules = [
            {
                'name': 'personal_space_respect',
                'condition': self.check_human_proximity,
                'action': self.execute_personal_space_maintenance,
                'priority': 65,
                'preconditions': ['human_detected']
            }
        ]

    def complex_rule_cycle(self):
        """Execute complex rule evaluation with multiple rule sets"""
        # Update environment state
        self.update_environment_state()

        # Evaluate rule sets in priority order
        active_behavior = None

        # Check safety rules first
        for rule in sorted(self.safety_rules, key=lambda x: x['priority'], reverse=True):
            if self.check_rule_preconditions(rule) and rule['condition']():
                rule['action']()
                active_behavior = rule['name']
                break

        # If no safety behavior, check navigation rules
        if not active_behavior:
            for rule in sorted(self.navigation_rules, key=lambda x: x['priority'], reverse=True):
                if self.check_rule_preconditions(rule) and rule['condition']():
                    rule['action']()
                    active_behavior = rule['name']
                    break

        # If no navigation behavior, check task rules
        if not active_behavior:
            for rule in sorted(self.task_rules, key=lambda x: x['priority'], reverse=True):
                if self.check_rule_preconditions(rule) and rule['condition']():
                    rule['action']()
                    active_behavior = rule['name']
                    break

        # If no task behavior, check social rules
        if not active_behavior:
            for rule in sorted(self.social_rules, key=lambda x: x['priority'], reverse=True):
                if self.check_rule_preconditions(rule) and rule['condition']():
                    rule['action']()
                    active_behavior = rule['name']
                    break

        # Publish active behavior
        if active_behavior:
            behavior_msg = String()
            behavior_msg.data = active_behavior
            self.behavior_pub.publish(behavior_msg)

    def check_rule_preconditions(self, rule) -> bool:
        """Check if rule preconditions are met"""
        for precondition in rule['preconditions']:
            if not self.decision_context.get(precondition, False):
                return False
        return True

    def update_environment_state(self):
        """Update environment state for decision making"""
        self.decision_context = {}

        # Update availability flags
        self.decision_context['scan_available'] = self.scan_data is not None
        self.decision_context['goal_available'] = self.current_goal is not None
        self.decision_context['pose_known'] = self.odom_data is not None

        # Update derived state
        if self.scan_data is not None:
            # Calculate environment features
            valid_ranges = [r for r in self.scan_data if not (np.isnan(r) or np.isinf(r))]
            if valid_ranges:
                self.environment_state['closest_obstacle'] = min(valid_ranges)
                self.environment_state['free_space_ratio'] = len([r for r in valid_ranges if r > 1.0]) / len(valid_ranges)

        if self.odom_data is not None:
            self.environment_state['speed'] = math.sqrt(
                self.odom_data.twist.twist.linear.x**2 +
                self.odom_data.twist.twist.linear.y**2
            )

    # Safety rule conditions and actions
    def check_collision_risk(self) -> bool:
        """Check for collision risk"""
        closest = self.environment_state.get('closest_obstacle', float('inf'))
        return closest < 0.4  # Risk if closer than 40cm

    def execute_collision_avoidance(self):
        """Execute collision avoidance"""
        cmd = Twist()
        # Implement collision avoidance behavior
        cmd.linear.x = 0.0  # Stop forward motion
        cmd.angular.z = 0.3  # Turn away from obstacle
        self.cmd_pub.publish(cmd)

    def check_balance_risk(self) -> bool:
        """Check for balance risk (simplified)"""
        # In a real humanoid system, this would check IMU data
        return False  # Simplified for this example

    def execute_balance_correction(self):
        """Execute balance correction"""
        cmd = Twist()
        # Implement balance correction
        self.cmd_pub.publish(cmd)

    # Navigation rule conditions and actions
    def check_goal_progress(self) -> bool:
        """Check if making progress toward goal"""
        if self.current_goal is None or self.odom_data is None:
            return False

        # Calculate distance to goal
        dx = self.current_goal.position.x - self.odom_data.pose.pose.position.x
        dy = self.current_goal.position.y - self.odom_data.pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance > 0.5  # Need to navigate if more than 0.5m from goal

    def execute_goal_navigation(self):
        """Execute goal-directed navigation"""
        cmd = Twist()
        # Implement goal navigation
        cmd.linear.x = 0.2  # Move forward
        self.cmd_pub.publish(cmd)

    def check_path_deviation(self) -> bool:
        """Check if deviating from planned path"""
        # Simplified - would check against planned path in real implementation
        return False

    def execute_path_following(self):
        """Execute path following"""
        cmd = Twist()
        # Implement path following
        self.cmd_pub.publish(cmd)

    # Task rule conditions and actions
    def check_object_proximity(self) -> bool:
        """Check if near an object to interact with"""
        # Simplified - would check for specific objects in real implementation
        return False

    def execute_object_interaction(self):
        """Execute object interaction"""
        cmd = Twist()
        # Implement object interaction
        self.cmd_pub.publish(cmd)

    # Social rule conditions and actions
    def check_human_proximity(self) -> bool:
        """Check if near a human (would use people detection in real system)"""
        # Simplified - would check for humans in real implementation
        return False

    def execute_personal_space_maintenance(self):
        """Execute personal space maintenance"""
        cmd = Twist()
        # Implement personal space behavior
        self.cmd_pub.publish(cmd)


def main(args=None):
    """
    Main function to run the rule-based decision system examples
    """
    rclpy.init(args=args)

    # Create nodes
    simple_rule_node = RuleBasedDecisionNode()
    complex_rule_node = ComplexRuleBasedSystemNode()

    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simple_rule_node)
    executor.add_node(complex_rule_node)

    try:
        print("Rule-Based Decision System Examples running...")
        print("These examples demonstrate:")
        print("- Simple reactive rule-based decision making")
        print("- Complex multi-layered rule systems")
        print("- Priority-based rule execution")
        print("- Safety-focused decision making")
        print("Check the logs for decision-making behavior.")
        print("Press Ctrl+C to stop.")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        simple_rule_node.destroy_node()
        complex_rule_node.destroy_node()
        rclpy.shutdown()
        print("Rule-Based Decision System Examples finished.")


if __name__ == '__main__':
    main()
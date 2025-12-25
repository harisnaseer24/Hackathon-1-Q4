---
sidebar_position: 3
title: "Control Patterns for Humanoid Robots"
---

# Control Patterns for Humanoid Robots

## Introduction

Humanoid robots present unique control challenges due to their complex kinematic structure, underactuation, and the need to maintain dynamic balance while performing tasks. This chapter explores the fundamental control patterns used in humanoid robotics and how they can be implemented using ROS 2. Understanding these patterns is essential for developing stable and capable humanoid robots.

## Fundamental Control Concepts

### Hierarchical Control Architecture

Humanoid control systems typically employ a hierarchical architecture with multiple control layers, each operating at different time scales:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class HierarchicalControlNode(Node):
    def __init__(self):
        super().__init__('hierarchical_control_node')

        # Publishers for different control levels
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.body_cmd_pub = self.create_publisher(Twist, 'body_motion_commands', 10)

        # Subscriptions for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Timers for different control frequencies
        self.high_freq_timer = self.create_timer(0.001, self.joint_control_loop)  # 1kHz
        self.mid_freq_timer = self.create_timer(0.01, self.balance_control_loop)  # 100Hz
        self.low_freq_timer = self.create_timer(0.1, self.motion_control_loop)    # 10Hz

        # Initialize state variables
        self.current_joint_positions = {}
        self.current_joint_velocities = {}
        self.imu_orientation = None
        self.imu_angular_velocity = None
        self.desired_body_motion = Twist()

    def joint_state_callback(self, msg):
        """Update joint state information"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update IMU information"""
        self.imu_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.imu_angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

    def joint_control_loop(self):
        """High-frequency joint control loop (1kHz)"""
        # Calculate desired joint torques based on higher-level commands
        # and current state feedback

        # Example: Simple PD control for joint positions
        joint_commands = Float64MultiArray()
        desired_positions = self.calculate_joint_positions_from_balance()

        for joint_name, desired_pos in desired_positions.items():
            current_pos = self.current_joint_positions.get(joint_name, 0.0)
            current_vel = self.current_joint_velocities.get(joint_name, 0.0)

            # PD control: tau = Kp * error + Kd * velocity_error
            position_error = desired_pos - current_pos
            velocity_error = 0.0 - current_vel  # Desired velocity is 0

            Kp = 100.0  # Proportional gain
            Kd = 10.0   # Derivative gain

            torque = Kp * position_error + Kd * velocity_error
            joint_commands.data.append(torque)

        self.joint_cmd_pub.publish(joint_commands)

    def balance_control_loop(self):
        """Mid-frequency balance control loop (100Hz)"""
        # Calculate desired joint positions for balance
        # based on IMU data and desired motion
        if self.imu_orientation is not None:
            # Calculate balance corrections based on orientation
            balance_correction = self.calculate_balance_correction()
            self.apply_balance_correction(balance_correction)

    def motion_control_loop(self):
        """Low-frequency motion control loop (10Hz)"""
        # Plan and execute gross body motions
        # such as walking, reaching, or turning
        motion_commands = self.plan_motion()
        self.body_cmd_pub.publish(motion_commands)

    def calculate_joint_positions_from_balance(self):
        """Calculate desired joint positions based on balance requirements"""
        # This would involve inverse kinematics and balance control algorithms
        # For this example, we'll return a simple default stance
        return {
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0,
            'shoulder_joint': 0.0,
            'elbow_joint': 0.0
        }

    def calculate_balance_correction(self):
        """Calculate balance correction based on IMU data"""
        if self.imu_orientation is None:
            return [0.0, 0.0, 0.0]

        # Convert quaternion to roll/pitch/yaw
        # Simplified conversion for demonstration
        w, x, y, z = self.imu_orientation
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))

        # Calculate correction based on tilt
        max_correction = 0.1  # radians
        roll_correction = max(-max_correction, min(max_correction, -roll * 2.0))
        pitch_correction = max(-max_correction, min(max_correction, -pitch * 2.0))

        return [roll_correction, pitch_correction, 0.0]  # [x, y, z corrections]

    def apply_balance_correction(self, correction):
        """Apply balance correction to joint commands"""
        # This would modify the desired joint positions based on balance needs
        pass

    def plan_motion(self):
        """Plan gross body motion"""
        # This would implement motion planning algorithms
        # For now, return a simple default motion command
        twist = Twist()
        twist.linear.x = 0.1  # Move forward slowly
        twist.angular.z = 0.0  # No turning
        return twist

def main(args=None):
    rclpy.init(args=args)
    controller = HierarchicalControlNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Joint-Level Control Patterns

### PID Control for Joint Position

PID (Proportional-Integral-Derivative) control is fundamental for precise joint control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

class JointPIDControllerNode(Node):
    def __init__(self):
        super().__init__('joint_pid_controller_node')

        # Publishers and subscribers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # PID controller parameters
        self.kp = 100.0  # Proportional gain
        self.ki = 10.0   # Integral gain
        self.kd = 5.0    # Derivative gain

        # PID state variables
        self.integral_error = {}
        self.previous_error = {}
        self.previous_time = None

        # Desired positions (for this example, fixed targets)
        self.desired_positions = {
            'hip_joint': 0.1,
            'knee_joint': 0.2,
            'ankle_joint': 0.05
        }

        # Current joint positions
        self.current_positions = {}

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

    def control_loop(self):
        """Main PID control loop"""
        current_time = time.time()
        if self.previous_time is None:
            self.previous_time = current_time
            return

        dt = current_time - self.previous_time
        self.previous_time = current_time

        joint_commands = Float64MultiArray()

        for joint_name, desired_pos in self.desired_positions.items():
            current_pos = self.current_positions.get(joint_name, 0.0)

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
            output = (self.kp * error +
                     self.ki * self.integral_error[joint_name] +
                     self.kd * derivative)

            joint_commands.data.append(output)

        self.joint_cmd_pub.publish(joint_commands)

def main(args=None):
    rclpy.init(args=args)
    controller = JointPIDControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Impedance Control

Impedance control allows for compliant behavior, important for safe human-robot interaction:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class ImpedanceControllerNode(Node):
    def __init__(self):
        super().__init__('impedance_controller_node')

        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'joint_impedance_commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.control_timer = self.create_timer(0.001, self.impedance_control_loop)  # 1kHz

        # Impedance parameters (stiffness and damping matrices)
        # For simplicity, using diagonal matrices
        self.stiffness = np.array([100.0, 150.0, 80.0])  # N⋅m/rad for each joint
        self.damping = np.array([10.0, 15.0, 8.0])      # N⋅m⋅s/rad for each joint

        # Desired equilibrium positions and velocities
        self.desired_positions = np.array([0.1, 0.2, 0.05])
        self.desired_velocities = np.array([0.0, 0.0, 0.0])
        self.desired_accelerations = np.array([0.0, 0.0, 0.0])

        # Current state
        self.current_positions = np.zeros(3)
        self.current_velocities = np.zeros(3)

    def joint_state_callback(self, msg):
        """Update current joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(self.current_positions):
                self.current_positions[i] = msg.position[i]
            if i < len(msg.velocity) and i < len(self.current_velocities):
                self.current_velocities[i] = msg.velocity[i]

    def impedance_control_loop(self):
        """Main impedance control loop"""
        # Calculate position and velocity errors
        pos_error = self.desired_positions - self.current_positions
        vel_error = self.desired_velocities - self.current_velocities

        # Calculate impedance forces: F = K * (x_d - x) + D * (v_d - v)
        impedance_force = (self.stiffness * pos_error +
                          self.damping * vel_error)

        # Publish impedance control commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = impedance_force.tolist()
        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = ImpedanceControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Balance Control Patterns

### Zero Moment Point (ZMP) Control

ZMP control is a classical approach for humanoid balance:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import numpy as np
import math

class ZMPControllerNode(Node):
    def __init__(self):
        super().__init__('zmp_controller_node')

        # Publishers
        self.zmp_ref_pub = self.create_publisher(Point, 'zmp_reference', 10)
        self.com_corr_pub = self.create_publisher(Point, 'center_of_mass_correction', 10)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'balance_joint_commands', 10)

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Timer for balance control
        self.balance_timer = self.create_timer(0.01, self.balance_control_loop)  # 100Hz

        # Robot parameters (simplified)
        self.com_height = 0.8  # Center of mass height in meters
        self.gravity = 9.81

        # Current state
        self.current_com = np.array([0.0, 0.0, self.com_height])
        self.current_com_vel = np.array([0.0, 0.0, 0.0])
        self.current_com_acc = np.array([0.0, 0.0, 0.0])

        # Support polygon (simplified as a rectangle)
        self.support_polygon = {
            'x_min': -0.1, 'x_max': 0.1,
            'y_min': -0.05, 'y_max': 0.05
        }

    def joint_state_callback(self, msg):
        """Update state based on joint positions"""
        # In a real implementation, this would calculate CoM from joint positions
        # For this example, we'll simulate the CoM based on some joint angles
        pass

    def imu_callback(self, msg):
        """Update state based on IMU data"""
        # Extract orientation and angular velocity
        # This would be used to estimate CoM state
        pass

    def balance_control_loop(self):
        """Main balance control loop using ZMP"""
        # Calculate current ZMP from CoM state
        current_zmp = self.calculate_zmp(self.current_com, self.current_com_acc)

        # Calculate desired ZMP (typically within support polygon)
        desired_zmp = self.calculate_desired_zmp()

        # Calculate CoM correction to achieve desired ZMP
        com_correction = self.calculate_com_correction(current_zmp, desired_zmp)

        # Publish ZMP reference and CoM correction
        zmp_msg = Point()
        zmp_msg.x = desired_zmp[0]
        zmp_msg.y = desired_zmp[1]
        zmp_msg.z = 0.0  # ZMP is in the ground plane
        self.zmp_ref_pub.publish(zmp_msg)

        com_corr_msg = Point()
        com_corr_msg.x = com_correction[0]
        com_corr_msg.y = com_correction[1]
        com_corr_msg.z = com_correction[2]
        self.com_corr_pub.publish(com_corr_msg)

        # Generate joint commands based on balance correction
        joint_commands = self.generate_balance_joint_commands(com_correction)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.joint_cmd_pub.publish(cmd_msg)

    def calculate_zmp(self, com_pos, com_acc):
        """Calculate ZMP from CoM position and acceleration"""
        # ZMP_x = CoM_x - (CoM_z - ZMP_z) / g * CoM_acc_x
        # ZMP_y = CoM_y - (CoM_z - ZMP_z) / g * CoM_acc_y
        zmp_x = com_pos[0] - (com_pos[2]) / self.gravity * com_acc[0]
        zmp_y = com_pos[1] - (com_pos[2]) / self.gravity * com_acc[1]
        return np.array([zmp_x, zmp_y])

    def calculate_desired_zmp(self):
        """Calculate desired ZMP within support polygon"""
        # For this example, keep ZMP in the center of the support polygon
        center_x = (self.support_polygon['x_min'] + self.support_polygon['x_max']) / 2.0
        center_y = (self.support_polygon['y_min'] + self.support_polygon['y_max']) / 2.0
        return np.array([center_x, center_y])

    def calculate_com_correction(self, current_zmp, desired_zmp):
        """Calculate CoM correction to move ZMP to desired location"""
        # Simple proportional control to move CoM
        correction_gain = 2.0  # Adjust based on robot dynamics
        pos_error = desired_zmp - current_zmp
        com_correction = correction_gain * pos_error
        return np.array([com_correction[0], com_correction[1], 0.0])  # Only x,y correction

    def generate_balance_joint_commands(self, com_correction):
        """Generate joint commands to achieve CoM correction"""
        # This would implement inverse kinematics to move CoM
        # For this example, return a simple command
        # In practice, this would use complex inverse kinematics
        return [com_correction[0] * 10.0, com_correction[1] * 10.0, 0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    controller = ZMPControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Walking Pattern Generation

### Inverse Kinematics for Walking

Walking requires coordinated movement of legs and body:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class WalkingControllerNode(Node):
    def __init__(self):
        super().__init__('walking_controller_node')

        # Publishers
        self.foot_trajectory_pub = self.create_publisher(Pose, 'foot_trajectory', 10)
        self.joint_trajectory_pub = self.create_publisher(Float64MultiArray, 'walking_joint_commands', 10)

        # Timer for walking control
        self.walk_timer = self.create_timer(0.02, self.walking_control_loop)  # 50Hz

        # Walking parameters
        self.step_length = 0.3  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds
        self.step_phase = 0.0  # 0.0 to 1.0

        # Robot parameters
        self.hip_offset = 0.1  # distance from body to hip
        self.thigh_length = 0.4  # length of thigh
        self.shin_length = 0.4  # length of shin

    def walking_control_loop(self):
        """Main walking control loop"""
        # Update step phase
        self.step_phase += 0.02 / self.step_duration  # dt = 0.02s
        if self.step_phase > 1.0:
            self.step_phase = 0.0

        # Calculate foot trajectory for current phase
        foot_pose = self.calculate_foot_trajectory(self.step_phase)

        # Publish foot trajectory
        self.foot_trajectory_pub.publish(foot_pose)

        # Calculate joint angles using inverse kinematics
        joint_commands = self.inverse_kinematics(foot_pose.position)

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_commands
        self.joint_trajectory_pub.publish(cmd_msg)

    def calculate_foot_trajectory(self, phase):
        """Calculate foot trajectory for a single step"""
        pose = Pose()

        # Calculate x position (forward movement)
        x = self.step_length * phase

        # Calculate z position (foot lift)
        if phase < 0.5:
            # Upward arc
            z = self.step_height * math.sin(math.pi * phase * 2)
        else:
            # Downward arc
            z = self.step_height * math.sin(math.pi * (1 - phase) * 2)

        # Keep y at 0 for straight walking
        y = 0.0

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Set orientation to keep foot level
        pose.orientation.w = 1.0  # No rotation
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        return pose

    def inverse_kinematics(self, foot_position):
        """Calculate joint angles for desired foot position (2D planar model)"""
        # Simplified 2D inverse kinematics for leg
        # foot_position is relative to hip
        x = foot_position.x
        y = foot_position.z  # z in pose becomes y in 2D leg model

        # Hip is at (0, 0) in leg coordinate system
        # Calculate distance from hip to foot
        distance = math.sqrt(x**2 + y**2)

        # Check if position is reachable
        leg_length = self.thigh_length + self.shin_length
        if distance > leg_length:
            # Position not reachable, extend leg fully
            angle1 = math.atan2(y, x)
            angle2 = 0.0  # Both joints aligned
        else:
            # Use law of cosines to find knee angle
            cos_knee_angle = (self.thigh_length**2 + self.shin_length**2 - distance**2) / (2 * self.thigh_length * self.shin_length)
            cos_knee_angle = max(-1, min(1, cos_knee_angle))  # Clamp to valid range
            knee_angle = math.pi - math.acos(cos_knee_angle)

            # Calculate hip angle
            hip_angle_offset = math.atan2(y, x)
            hip_angle_correction = math.atan2(self.shin_length * math.sin(knee_angle),
                                            self.thigh_length + self.shin_length * math.cos(knee_angle))
            hip_angle = hip_angle_offset - hip_angle_correction

            angle1 = hip_angle
            angle2 = knee_angle

        # Calculate ankle angle to keep foot level (simplified)
        ankle_angle = -(angle1 + angle2)

        # Return joint angles [hip, knee, ankle]
        return [angle1, angle2, ankle_angle]

def main(args=None):
    rclpy.init(args=args)
    controller = WalkingControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Control Patterns

### Model Predictive Control (MPC)

MPC is an advanced control technique that can handle constraints:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from scipy.optimize import minimize

class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, 'mpc_joint_commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.mpc_timer = self.create_timer(0.1, self.mpc_control_loop)  # 10Hz

        # MPC parameters
        self.prediction_horizon = 10  # Number of steps to predict
        self.control_horizon = 5      # Number of steps to optimize
        self.dt = 0.1                 # Time step (10Hz)

        # State and control dimensions
        self.state_dim = 6   # position, velocity for 3 joints
        self.control_dim = 3 # torques for 3 joints

        # Current state
        self.current_state = np.zeros(self.state_dim)

        # System matrices (simplified linear model)
        self.A = self.create_system_matrix()
        self.B = self.create_input_matrix()

        # Cost matrices
        self.Q = np.eye(self.state_dim) * 10.0  # State cost
        self.R = np.eye(self.control_dim) * 1.0  # Control cost

    def create_system_matrix(self):
        """Create discrete-time system matrix A for x[k+1] = A*x[k] + B*u[k]"""
        # Simplified double integrator model for each joint
        # [pos, vel] -> [pos + vel*dt, vel + acc*dt]
        dt = self.dt
        A = np.eye(self.state_dim)
        for i in range(0, self.state_dim, 2):  # Position indices
            if i + 1 < self.state_dim:  # Velocity index
                A[i, i + 1] = dt  # Position update from velocity
        return A

    def create_input_matrix(self):
        """Create input matrix B for x[k+1] = A*x[k] + B*u[k]"""
        # Simplified model where control input affects acceleration
        B = np.zeros((self.state_dim, self.control_dim))
        for i in range(1, self.state_dim, 2):  # Velocity indices
            B[i, (i-1)//2] = self.dt  # Acceleration affects velocity
        return B

    def joint_state_callback(self, msg):
        """Update current state from joint measurements"""
        # In a real system, this would convert joint positions and velocities
        # to the state representation used by the MPC controller
        for i in range(min(len(msg.position), self.control_dim)):
            self.current_state[i * 2] = msg.position[i]  # Position
        for i in range(min(len(msg.velocity), self.control_dim)):
            self.current_state[i * 2 + 1] = msg.velocity[i]  # Velocity

    def mpc_control_loop(self):
        """Main MPC control loop"""
        # Define the desired state (e.g., zero position and velocity)
        desired_state = np.zeros(self.state_dim)

        # Solve MPC optimization problem
        optimal_control = self.solve_mpc(self.current_state, desired_state)

        # Publish the first control input
        cmd_msg = Float64MultiArray()
        cmd_msg.data = optimal_control[:self.control_dim].tolist()  # First control step
        self.joint_cmd_pub.publish(cmd_msg)

    def solve_mpc(self, current_state, desired_state):
        """Solve the MPC optimization problem"""
        # Define cost function to minimize
        def cost_function(u_flat):
            # Reshape flattened control sequence
            U = u_flat.reshape((self.control_horizon, self.control_dim))

            # Simulate the system forward
            x = current_state.copy()
            total_cost = 0.0

            for k in range(self.prediction_horizon):
                # Apply control if within control horizon
                if k < self.control_horizon:
                    u = U[k]
                else:
                    u = np.zeros(self.control_dim)  # Zero control after control horizon

                # Predict next state
                x = self.A @ x + self.B @ u

                # Add state cost
                state_error = x - desired_state
                total_cost += state_error.T @ self.Q @ state_error

                # Add control cost if within control horizon
                if k < self.control_horizon:
                    total_cost += u.T @ self.R @ u

            return total_cost

        # Initial guess for control sequence
        initial_u = np.zeros(self.control_horizon * self.control_dim)

        # Define bounds for control inputs (optional)
        # For this example, no bounds
        bounds = [(-100, 100) for _ in range(self.control_horizon * self.control_dim)]

        # Solve optimization problem
        result = minimize(cost_function, initial_u, method='SLSQP', bounds=bounds)

        if result.success:
            return result.x
        else:
            # Return zero control if optimization fails
            self.get_logger().warn('MPC optimization failed, using zero control')
            return np.zeros(self.control_horizon * self.control_dim)

def main(args=None):
    rclpy.init(args=args)
    controller = MPCControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Fault Tolerance

### Emergency Stop and Recovery

Safety mechanisms are crucial for humanoid robots:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
import numpy as np

class SafetyControllerNode(Node):
    def __init__(self):
        super().__init__('safety_controller_node')

        # Publishers
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_cmd_pub = self.create_publisher(Float64MultiArray, 'safety_joint_commands', 10)
        self.stop_cmd_pub = self.create_publisher(Twist, 'stop_cmd', 10)

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.005, self.safety_check)  # 200Hz

        # Safety parameters
        self.joint_position_limits = {
            'hip_joint': (-1.5, 1.5),
            'knee_joint': (0.0, 2.5),
            'ankle_joint': (-0.8, 0.8)
        }

        self.joint_velocity_limits = 5.0  # rad/s
        self.tilt_threshold = 0.5  # rad (about 28 degrees)

        # Current state
        self.current_positions = {}
        self.current_velocities = {}
        self.imu_orientation = None

        # Safety state
        self.emergency_stop_active = False
        self.last_safe_positions = {}

    def joint_state_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.current_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """Update IMU state"""
        self.imu_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def safety_check(self):
        """Main safety monitoring function"""
        if self.emergency_stop_active:
            # Already in emergency stop state, just publish stop commands
            self.publish_stop_commands()
            return

        # Check various safety conditions
        dangerous_condition = False

        # Check joint position limits
        for joint_name, pos in self.current_positions.items():
            if joint_name in self.joint_position_limits:
                min_limit, max_limit = self.joint_position_limits[joint_name]
                if pos < min_limit or pos > max_limit:
                    self.get_logger().error(f'Joint {joint_name} exceeded position limits: {pos}')
                    dangerous_condition = True

        # Check joint velocity limits
        for joint_name, vel in self.current_velocities.items():
            if abs(vel) > self.joint_velocity_limits:
                self.get_logger().error(f'Joint {joint_name} exceeded velocity limits: {vel}')
                dangerous_condition = True

        # Check IMU for dangerous tilt
        if self.imu_orientation is not None:
            # Convert quaternion to pitch angle (simplified)
            _, pitch, _ = self.quaternion_to_euler(self.imu_orientation)
            if abs(pitch) > self.tilt_threshold:
                self.get_logger().error(f'Dangerous tilt detected: pitch = {pitch}')
                dangerous_condition = True

        # Trigger emergency stop if dangerous condition detected
        if dangerous_condition:
            self.trigger_emergency_stop()

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles (simplified)"""
        # Extract pitch from quaternion
        w, x, y, z = quat
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop_active = True
        self.get_logger().error('EMERGENCY STOP ACTIVATED')

        # Store current safe positions for recovery
        self.last_safe_positions = self.current_positions.copy()

        # Publish emergency stop signal
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Publish zero commands to actuators
        self.publish_stop_commands()

    def publish_stop_commands(self):
        """Publish commands to stop all motion"""
        # Publish zero joint commands
        zero_cmd = Float64MultiArray()
        zero_cmd.data = [0.0, 0.0, 0.0]  # Zero torques for all joints
        self.safety_cmd_pub.publish(zero_cmd)

        # Publish stop command for base motion
        stop_twist = Twist()
        self.stop_cmd_pub.publish(stop_twist)

    def reset_emergency_stop(self):
        """Reset emergency stop (to be called externally)"""
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop reset')

def main(args=None):
    rclpy.init(args=args)
    safety_controller = SafetyControllerNode()

    try:
        rclpy.spin(safety_controller)
    except KeyboardInterrupt:
        pass
    finally:
        safety_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Patterns

### State Machine for Control Modes

Using a state machine to manage different control modes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from enum import Enum

class ControlMode(Enum):
    IDLE = "idle"
    STANDING = "standing"
    WALKING = "walking"
    BALANCE = "balance"
    EMERGENCY_STOP = "emergency_stop"

class StateMachineControllerNode(Node):
    def __init__(self):
        super().__init__('state_machine_controller_node')

        # Publishers
        self.mode_pub = self.create_publisher(String, 'control_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # Timer for state machine
        self.state_timer = self.create_timer(0.05, self.state_machine_loop)  # 20Hz

        # Initialize state
        self.current_mode = ControlMode.IDLE
        self.previous_mode = None
        self.state_entry_time = self.get_clock().now()

        # Mode transition triggers
        self.requested_mode = None

    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        # This would be used to make mode transition decisions
        pass

    def state_machine_loop(self):
        """Main state machine loop"""
        # Update previous mode if it changed
        if self.current_mode != self.previous_mode:
            self.state_entry_time = self.get_clock().now()
            self.previous_mode = self.current_mode
            self.on_mode_enter(self.current_mode)

        # Check for mode transitions
        new_mode = self.evaluate_transitions()
        if new_mode != self.current_mode:
            self.transition_to_mode(new_mode)

        # Execute current mode behavior
        self.execute_current_mode()

        # Publish current mode
        mode_msg = String()
        mode_msg.data = self.current_mode.value
        self.mode_pub.publish(mode_msg)

    def evaluate_transitions(self):
        """Evaluate conditions for mode transitions"""
        current_time = self.get_clock().now()
        time_in_state = (current_time - self.state_entry_time).nanoseconds / 1e9

        # Example transition logic
        if self.requested_mode is not None:
            # External request takes priority
            return self.requested_mode

        # Idle mode transitions
        if self.current_mode == ControlMode.IDLE:
            # Transition to standing if conditions are met
            return ControlMode.STANDING

        # Standing mode transitions
        elif self.current_mode == ControlMode.STANDING:
            # Check for balance issues
            if self.detect_balance_issue():
                return ControlMode.BALANCE
            # Check for walking command
            elif self.should_start_walking():
                return ControlMode.WALKING

        # Walking mode transitions
        elif self.current_mode == ControlMode.WALKING:
            # Check for balance issues
            if self.detect_balance_issue():
                return ControlMode.BALANCE
            # Check if walking should stop
            elif not self.should_continue_walking():
                return ControlMode.STANDING

        # Balance mode transitions
        elif self.current_mode == ControlMode.BALANCE:
            # Check if balance is recovered
            if self.is_balanced() and time_in_state > 2.0:  # Stay in balance for at least 2 seconds
                if self.should_walk():
                    return ControlMode.WALKING
                else:
                    return ControlMode.STANDING

        # Emergency stop - only exit via external command
        elif self.current_mode == ControlMode.EMERGENCY_STOP:
            # Can only exit emergency stop via external reset
            pass

        # Default: stay in current mode
        return self.current_mode

    def detect_balance_issue(self):
        """Detect if robot is losing balance"""
        # This would check IMU, joint positions, etc.
        # For this example, return a simulated value
        import random
        return random.random() < 0.05  # 5% chance of balance issue

    def should_start_walking(self):
        """Check if robot should start walking"""
        # This would check for walk commands or other conditions
        return False

    def should_continue_walking(self):
        """Check if robot should continue walking"""
        return False

    def should_walk(self):
        """Check if robot should transition to walking"""
        return False

    def is_balanced(self):
        """Check if robot is balanced"""
        # This would check actual balance metrics
        import random
        return random.random() > 0.1  # 90% chance of being balanced

    def transition_to_mode(self, new_mode):
        """Handle transition to a new mode"""
        self.get_logger().info(f'Transitioning from {self.current_mode.value} to {new_mode.value}')
        self.on_mode_exit(self.current_mode)
        self.current_mode = new_mode

    def on_mode_enter(self, mode):
        """Called when entering a new mode"""
        self.get_logger().info(f'Entering mode: {mode.value}')
        # Mode-specific initialization would go here

    def on_mode_exit(self, mode):
        """Called when exiting a mode"""
        self.get_logger().info(f'Exiting mode: {mode.value}')
        # Mode-specific cleanup would go here

    def execute_current_mode(self):
        """Execute behavior for current mode"""
        if self.current_mode == ControlMode.IDLE:
            self.execute_idle_mode()
        elif self.current_mode == ControlMode.STANDING:
            self.execute_standing_mode()
        elif self.current_mode == ControlMode.WALKING:
            self.execute_walking_mode()
        elif self.current_mode == ControlMode.BALANCE:
            self.execute_balance_mode()
        elif self.current_mode == ControlMode.EMERGENCY_STOP:
            self.execute_emergency_stop_mode()

    def execute_idle_mode(self):
        """Execute idle mode behavior"""
        # Minimal activity, perhaps breathing motion
        pass

    def execute_standing_mode(self):
        """Execute standing mode behavior"""
        # Maintain standing position with balance control
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_walking_mode(self):
        """Execute walking mode behavior"""
        # Generate walking commands
        cmd = Twist()
        cmd.linear.x = 0.2  # Walk forward slowly
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_balance_mode(self):
        """Execute balance mode behavior"""
        # Focus on balance recovery
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_emergency_stop_mode(self):
        """Execute emergency stop mode behavior"""
        # Stop all motion
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def request_mode_change(self, new_mode):
        """Request a mode change (external interface)"""
        self.requested_mode = new_mode

def main(args=None):
    rclpy.init(args=args)
    controller = StateMachineControllerNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Control patterns for humanoid robots involve multiple layers of control working together, from high-level motion planning down to low-level joint control. Key patterns include:

1. **Hierarchical Control**: Different control layers operating at different frequencies
2. **Joint-Level Control**: PID, impedance, and other low-level control techniques
3. **Balance Control**: ZMP-based and other balance maintenance strategies
4. **Walking Control**: Inverse kinematics and gait generation
5. **Advanced Control**: MPC and other optimization-based approaches
6. **Safety Systems**: Emergency stops and fault tolerance
7. **State Machines**: Managing different operational modes

Understanding and implementing these patterns properly is essential for creating stable, safe, and capable humanoid robots.

## Exercises and Self-Assessment

### Conceptual Understanding
1. Explain the hierarchical control architecture in humanoid robots. What are the typical control frequencies for each level and why?

2. Compare and contrast PID control, impedance control, and model predictive control. What are the advantages and disadvantages of each approach for humanoid robotics?

### Application Questions
3. Design a control architecture for a humanoid robot that needs to:
   - Stand up from a seated position
   - Walk forward for 10 steps
   - Turn 90 degrees
   - Sit down
   Describe the control modes, transitions, and key control patterns needed.

4. How would you implement a balance controller that can handle external disturbances (e.g., someone pushing the robot)?

### Hands-on Practice
5. Implement a simple PD controller for a single joint and tune the gains to achieve stable position control.

6. Create a state machine that transitions between standing, walking, and sitting modes based on sensor feedback.

### Critical Thinking
7. What are the main challenges in implementing whole-body control for humanoid robots? How do different control patterns address these challenges?

8. How would you design a control system that can adapt to different terrains (flat ground, stairs, uneven surfaces)?
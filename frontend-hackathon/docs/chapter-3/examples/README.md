# Chapter 3: AI Agent Integration Examples

This directory contains executable code examples that demonstrate the integration of AI agents with ROS 2 controllers, as covered in Chapter 3: Bridging AI Agents to ROS Controllers.

## Available Examples

### 1. ai_agent_integration.py
A comprehensive example showing multiple patterns for integrating AI agents with ROS 2:
- Simple reactive AI agent with obstacle avoidance
- Learning-based AI agent using Q-learning
- Advanced AI-ROS bridge with sensor fusion
- Decision coordination between multiple AI sources

**To run:**
```bash
# Make sure you have ROS 2 and the required dependencies installed
cd frontend-hackathon
python3 docs/chapter-3/examples/ai_agent_integration.py
```

### 2. rule_based_decision_system.py
A rule-based decision system that demonstrates how to implement deterministic decision making for robot control.

**To run:**
```bash
python3 docs/chapter-3/examples/rule_based_decision_system.py
```

### 3. behavior_tree_agent.py
An implementation of a behavior tree-based AI agent for complex task execution.

**To run:**
```bash
python3 docs/chapter-3/examples/behavior_tree_agent.py
```

### 4. neural_network_bridge.py
Example of integrating neural network-based AI with ROS 2 control systems.

**To run:**
```bash
python3 docs/chapter-3/examples/neural_network_bridge.py
```

## Requirements

To run these examples, you need:

- ROS 2 (Humble Hawksbill or later recommended)
- Python 3.8 or later
- Standard Python packages:
  - numpy
  - rclpy
  - opencv-python (for vision examples)
  - tensorflow or pytorch (for neural network examples)

Install dependencies:
```bash
pip3 install numpy opencv-python tensorflow
```

## Dependencies

These examples depend on:
- Standard ROS 2 message types (std_msgs, geometry_msgs, sensor_msgs)
- ROS 2 action interfaces (action_msgs)
- Basic navigation and manipulation interfaces (if running with robot)

## Running with a Robot Simulation

To see these examples in action with a simulated robot:

1. Launch a robot simulation (e.g., TurtleBot3):
```bash
# In a separate terminal
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

2. Run the AI agent example:
```bash
python3 docs/chapter-3/examples/ai_agent_integration.py
```

3. The AI agent will connect to the simulated robot and begin executing behaviors.

## Troubleshooting

- If you get import errors, make sure ROS 2 is properly sourced: `source /opt/ros/humble/setup.bash`
- If the robot doesn't respond, check that the topic names match your robot's configuration
- For vision examples, make sure camera topics are available
- Check the console output for error messages and warnings

## Extending the Examples

These examples are designed to be extended for specific applications:
- Modify the decision logic in `ai_agent_integration.py` for your specific task
- Add new sensor types by extending the sensor fusion in `AIBridgeNode`
- Implement custom reward functions in the learning agent
- Add new behavior trees for complex task execution
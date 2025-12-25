---
sidebar_position: 1
---

# Chapter 4: URDF Structure and Purpose

## Overview

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links, joints, and their relationships. For humanoid robots, URDF is crucial as it provides the foundation for simulation, visualization, and kinematic analysis.

## What is URDF?

URDF stands for Unified Robot Description Format. It's an XML-based format that allows you to describe a robot's physical properties including:

- **Links**: Rigid parts of the robot (e.g., torso, arms, legs, head)
- **Joints**: Connections between links (e.g., hinges, prismatic joints)
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: How the robot interacts with the environment
- **Inertial properties**: Mass, center of mass, and moments of inertia

## URDF in Humanoid Robotics

For humanoid robots, URDF is particularly important because:

1. **Complex Kinematics**: Humanoid robots have complex kinematic chains with multiple degrees of freedom
2. **Simulation**: URDF enables accurate physics simulation in environments like Gazebo
3. **Visualization**: URDF models can be visualized in RViz for debugging and development
4. **Motion Planning**: Planning algorithms use URDF for collision checking and inverse kinematics

## Basic URDF Structure

A basic URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="child_link">
    <!-- Similar structure as base_link -->
  </link>
</robot>
```

## Key Components

### Links

Links represent rigid bodies in the robot. Each link can have:

- **Visual**: How the link appears in simulation/visualization
- **Collision**: How the link interacts with the environment
- **Inertial**: Physical properties for dynamics simulation

### Joints

Joints connect links and define how they can move relative to each other. Joint types include:

- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Continuous rotational movement (like a wheel)
- **Prismatic**: Linear sliding movement
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

### Materials and Colors

You can define materials for visualization:

```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<link name="my_link">
  <visual>
    <material name="blue"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
</link>
```

## URDF for Humanoid Robots

Humanoid robots require special considerations in their URDF:

### Multi-Chain Structure

Humanoid robots have multiple kinematic chains (arms, legs, torso, head) that need to be properly connected:

```
torso
├── head
├── left_arm
│   ├── left_forearm
│   └── left_hand
├── right_arm
│   ├── right_forearm
│   └── right_hand
├── left_leg
│   ├── left_lower_leg
│   └── left_foot
└── right_leg
    ├── right_lower_leg
    └── right_foot
```

### Proper Mass Distribution

For stable simulation, humanoid URDFs need realistic mass distributions:

- Head: ~5% of total mass
- Torso: ~50% of total mass
- Arms: ~13% each
- Legs: ~18% each

### Joint Limits

Humanoid joints should have realistic limits to prevent unnatural poses:

- Shoulder: ±90° in multiple axes
- Elbow: -120° to 0°
- Hip: ±45° for abduction/adduction
- Knee: 0° to -120° (flexion)

## Best Practices

1. **Start Simple**: Begin with a basic skeleton and add complexity gradually
2. **Use Xacro**: For complex robots, use Xacro (XML Macros) to avoid repetition
3. **Validate**: Use `check_urdf` to validate your URDF files
4. **Test in Simulation**: Verify the model works correctly in Gazebo or other simulators
5. **Realistic Dimensions**: Use actual robot dimensions when available

## URDF Tools

ROS provides several tools for working with URDF:

- `check_urdf`: Validates URDF files
- `urdf_to_graphiz`: Generates visual representations of the kinematic tree
- `joint_state_publisher`: Publishes joint states for visualization
- `robot_state_publisher`: Publishes transforms for the robot model

## Example: Simple Humanoid Torso

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Next Steps

After creating your URDF model, you'll need to:

1. **Validate** the model using ROS tools
2. **Test** in simulation environments
3. **Add** transmission definitions for actuators
4. **Include** Gazebo-specific tags for simulation
5. **Create** launch files for easy loading

URDF is the foundation for humanoid robot simulation and visualization. A well-constructed URDF model is essential for effective robot development and testing in ROS environments.
---
sidebar_position: 2
---

# Chapter 4: Links, Joints, and Kinematics

## Overview

In URDF (Unified Robot Description Format), links and joints form the fundamental building blocks of robot models. Understanding how to properly define these elements is crucial for creating accurate and functional humanoid robot models. This chapter covers the detailed structure of links and joints, and how they relate to robot kinematics.

## Links: The Building Blocks of Robot Models

### What is a Link?

A link in URDF represents a rigid body part of the robot. Each link has:

- **Physical properties**: Mass, center of mass, and inertia tensor
- **Visual representation**: How the link appears in visualization
- **Collision properties**: How the link interacts with the environment

### Link Structure

A basic link definition includes three main components:

```xml
<link name="link_name">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Shape definition -->
    </geometry>
    <material name="material_name"/>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Shape definition -->
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

### Visual Element

The visual element defines how the link appears in visualization tools like RViz:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Options: box, cylinder, sphere, or mesh -->
    <box size="0.1 0.1 0.1"/>
    <!-- <cylinder radius="0.1" length="0.2"/> -->
    <!-- <sphere radius="0.1"/> -->
    <!-- <mesh filename="package://my_robot/meshes/link.stl"/> -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Element

The collision element defines how the link interacts with the environment in simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Similar to visual, but often simplified for performance -->
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Element

The inertial element defines the physical properties for dynamics simulation:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

## Joints: Connecting the Links

### What is a Joint?

A joint in URDF defines the connection between two links and specifies how they can move relative to each other. Joints have types, limits, and kinematic properties.

### Joint Structure

A basic joint definition includes:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Joint Types

#### 1. Fixed Joint

A fixed joint has no degrees of freedom - the connected links move together as one:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_mount"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>
```

#### 2. Revolute Joint

A revolute joint allows rotation around a single axis with defined limits:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.356" upper="0" effort="30" velocity="1.0"/>
</joint>
```

#### 3. Continuous Joint

A continuous joint allows unlimited rotation around an axis (like a wheel):

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <origin xyz="0 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

#### 4. Prismatic Joint

A prismatic joint allows linear translation along an axis:

```xml
<joint name="slider_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="50" velocity="0.5"/>
</joint>
```

#### 5. Floating and Planar Joints

- **Floating**: 6 degrees of freedom (not commonly used in humanoid robots)
- **Planar**: Movement in a plane (rarely used in humanoid robots)

### Joint Properties

#### Origin

The origin element specifies the position and orientation of the joint relative to the parent link:

```xml
<origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>
```

- `xyz`: Position offset (x, y, z) in meters
- `rpy`: Orientation offset (roll, pitch, yaw) in radians

#### Axis

The axis element defines the joint's axis of motion:

```xml
<axis xyz="0 0 1"/>  <!-- Rotates around Z-axis -->
<axis xyz="1 0 0"/>  <!-- Rotates around X-axis -->
```

#### Limits

The limit element defines the joint's range of motion:

```xml
<limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
```

- `lower`: Minimum joint value (radians for revolute, meters for prismatic)
- `upper`: Maximum joint value
- `effort`: Maximum effort (torque for revolute, force for prismatic)
- `velocity`: Maximum velocity

## Kinematics for Humanoid Robots

### Forward Kinematics

Forward kinematics calculates the position and orientation of the end effector given joint angles. For humanoid robots, this is essential for:

- Reaching targets with hands
- Placing feet for stable walking
- Head orientation for vision systems

### Inverse Kinematics

Inverse kinematics calculates the required joint angles to achieve a desired end-effector position. This is crucial for:

- Walking pattern generation
- Object manipulation
- Balance maintenance

### Kinematic Chains in Humanoid Robots

Humanoid robots have multiple kinematic chains:

#### Arm Chain
```
torso → shoulder → upper_arm → elbow → forearm → hand
```

#### Leg Chain
```
torso → hip → thigh → knee → shin → foot
```

#### Head Chain
```
torso → neck → head
```

## Practical Example: Humanoid Arm

Let's create a complete example of a humanoid arm with proper links and joints:

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Shoulder (connected to torso) -->
  <link name="shoulder">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder joint (YAW) -->
  <joint name="shoulder_yaw" type="revolute">
    <parent link="shoulder"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <!-- Upper Arm -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.356" upper="0" effort="40" velocity="2"/>
  </joint>

  <!-- Forearm -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Wrist joint -->
  <joint name="wrist_joint" type="revolute">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="3"/>
  </joint>

  <!-- Hand -->
  <link name="hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

</robot>
```

## Kinematic Considerations for Humanoid Robots

### Degrees of Freedom (DOF)

Humanoid robots typically have high DOF:

- **Human-like arm**: 7 DOF (shoulder: 3, elbow: 1, wrist: 3)
- **Human-like leg**: 6 DOF (hip: 3, knee: 1, ankle: 2)
- **Full humanoid**: 28+ DOF for full body

### Redundancy

Humanoid robots often have redundant DOF, meaning multiple joint configurations can achieve the same end-effector position. This allows for:

- Obstacle avoidance
- Balance maintenance
- Natural movement patterns

### Singularity Avoidance

Near kinematic singularities (where the Jacobian matrix becomes singular), small changes in end-effector position require large joint changes. Good URDF design includes:

- Joint limits that avoid singular configurations
- Proper joint placement to minimize singular regions

## Best Practices for Links and Joints

1. **Consistent Naming**: Use descriptive names that indicate function (e.g., `left_elbow_joint`)
2. **Proper Coordinate Frames**: Follow the right-hand rule for joint axes
3. **Realistic Inertial Properties**: Use actual robot measurements when possible
4. **Appropriate Joint Limits**: Set limits based on physical constraints
5. **Collision-Free Configurations**: Ensure joint limits don't cause self-collision
6. **Mass Distribution**: Distribute mass realistically across links

## URDF Validation

Always validate your URDF models:

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Visualize the kinematic tree
urdf_to_graphiz /path/to/robot.urdf
```

## Next Steps

After defining your links and joints:

1. **Test kinematics** with forward and inverse kinematics solvers
2. **Add transmission elements** for actuator control
3. **Include Gazebo plugins** for simulation
4. **Create launch files** for easy loading and visualization

Properly defined links and joints form the foundation for successful humanoid robot simulation and control in ROS environments.
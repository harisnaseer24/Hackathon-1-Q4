---
sidebar_position: 3
---

# Chapter 4: Preparing Models for Simulation

## Overview

Creating a URDF model is just the first step in developing a humanoid robot for simulation. To ensure realistic and stable simulation, you need to add additional elements that define how the robot behaves in physics engines like Gazebo. This chapter covers the essential components needed to prepare your URDF models for simulation, including Gazebo-specific tags, transmissions, and plugins.

## Gazebo-Specific URDF Extensions

### Gazebo Materials

You can define materials specifically for Gazebo visualization:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
</gazebo>
```

Common Gazebo materials include:
- `Gazebo/Blue`
- `Gazebo/Red`
- `Gazebo/Green`
- `Gazebo/White`
- `Gazebo/Black`
- `Gazebo/Yellow`

### Gazebo Links

Define physics properties for links in simulation:

```xml
<gazebo reference="link_name">
  <!-- Visual properties -->
  <material>Gazebo/Blue</material>

  <!-- Physics properties -->
  <mu1>0.9</mu1>  <!-- Friction coefficient -->
  <mu2>0.9</mu2>  <!-- Secondary friction coefficient -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>1000000.0</kd>  <!-- Contact damping -->
  <self_collide>false</self_collide>  <!-- Enable/disable self-collision -->
  <gravity>true</gravity>  <!-- Enable/disable gravity for this link -->
</gazebo>
```

### Complete Gazebo Link Example

```xml
<link name="arm_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>

<gazebo reference="arm_link">
  <material>Gazebo/Blue</material>
  <mu1>0.9</mu1>
  <mu2>0.9</mu2>
  <kp>1000000.0</kp>
  <kd>1000000.0</kd>
  <self_collide>false</self_collide>
</gazebo>
```

## Transmissions

Transmissions define the relationship between joints and actuators. This is essential for controlling the robot in simulation.

### Simple Transmission

For a single joint with a simple actuator:

```xml
<transmission name="joint1_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="joint1_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Hardware Interfaces

Common hardware interfaces include:
- `hardware_interface/EffortJointInterface`: Control joint forces/torques
- `hardware_interface/VelocityJointInterface`: Control joint velocities
- `hardware_interface/PositionJointInterface`: Control joint positions

### Complete Transmission Example

```xml
<transmission name="shoulder_yaw_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="shoulder_yaw_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="shoulder_yaw_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Gazebo Plugins

Gazebo plugins provide additional functionality for your robot model in simulation.

### Joint State Publisher

The joint state publisher plugin publishes joint states to ROS:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>joint1</joint_name>
    <joint_name>joint2</joint_name>
    <update_rate>30</update_rate>
    <alwaysOn>true</alwaysOn>
  </plugin>
</gazebo>
```

### Joint Controller

The joint controller plugin allows ROS control of joints:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/robot_name</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>
  </plugin>
</gazebo>
```

### IMU Sensor Plugin

For humanoid robots, IMU sensors are often needed for balance:

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <topic>__default_topic__</topic>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <bodyName>torso</bodyName>
      <topicName>imu/data</topicName>
      <serviceName>imu/service</serviceName>
      <gaussianNoise>0.01</gaussianNoise>
      <updateRateHZ>100.0</updateRateHZ>
      <imu>
        <noise>
          <type>gaussian</type>
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Plugin

For vision-based humanoid robots:

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Complete Example: Humanoid Robot for Gazebo

Here's a complete example of a humanoid robot ready for simulation:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Transmissions -->
  <transmission name="neck_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="neck_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="neck_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_shoulder_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_shoulder_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_shoulder_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo-specific elements -->
  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
    <mu1>0.9</mu1>
    <mu2>0.9</mu2>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Orange</material>
  </gazebo>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- IMU for balance -->
  <gazebo reference="torso">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <topic>__default_topic__</topic>
      <plugin name="torso_imu_plugin" filename="libgazebo_ros_imu.so">
        <bodyName>torso</bodyName>
        <topicName>imu/data</topicName>
        <serviceName>imu/service</serviceName>
        <gaussianNoise>0.01</gaussianNoise>
        <updateRateHZ>100.0</updateRateHZ>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Simulation Setup

### Launch File for Gazebo

Create a launch file to spawn your robot in Gazebo:

```xml
<launch>
  <!-- Load the URDF into the parameter server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot_description)/urdf/robot.urdf.xacro'" />

  <!-- Start Gazebo with an empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn the robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model humanoid_robot"
        respawn="false" output="screen"/>

  <!-- Publish joint states -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="true"/>
  </node>

  <!-- Publish robot state -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
        respawn="false" output="screen"/>
</launch>
```

## Testing Your Model

### URDF Validation

Before simulation, validate your URDF:

```bash
# Check syntax
check_urdf robot.urdf

# Visualize kinematic tree
urdf_to_graphiz robot.urdf

# Check for self-collisions and singularities
```

### Simulation Testing

1. **Load in RViz**: Verify the model displays correctly
2. **Joint Movement**: Test that all joints move as expected
3. **Physics Behavior**: Check that the robot behaves physically
4. **Sensor Output**: Verify sensors publish correct data
5. **Control Interface**: Test that joints respond to commands

## Common Issues and Solutions

### Self-Collision

Problem: Robot links collide with each other in simulation.
Solution: Adjust joint limits or add `<self_collide>false</self_collide>` to specific links.

### Instability

Problem: Robot shakes or behaves erratically in simulation.
Solution: Check inertial properties and adjust physics parameters.

### Joint Control Issues

Problem: Joints don't respond to commands.
Solution: Verify transmissions and hardware interfaces are properly configured.

### Visual Issues

Problem: Robot appears incorrectly in Gazebo.
Solution: Check material definitions and geometry coordinates.

## Advanced Simulation Features

### Gazebo Worlds

Create custom worlds for humanoid robot testing:

```xml
<sdf version='1.6'>
  <world name='humanoid_test_world'>
    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom objects -->
    <model name='obstacle'>
      <pose>2 0 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Controllers Configuration

Create a controller configuration file (YAML) for ros_control:

```yaml
# Controller configuration
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

position_controllers:
  type: position_controllers/JointTrajectoryController
  joints:
    - neck_joint
    - left_shoulder_joint
  constraints:
    goal_time: 0.6
    stopped_velocity_tolerance: 0.05
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
```

## Performance Optimization

### Mesh Simplification

For collision elements, use simplified meshes:

```xml
<link name="complex_link">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/complex_visual.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <!-- Use simpler mesh for collision -->
      <mesh filename="package://my_robot/meshes/simple_collision.stl"/>
    </geometry>
  </collision>
</link>
```

### Physics Parameters

Adjust physics parameters for better performance:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <controlPeriod>0.001</controlPeriod>  <!-- 1ms control cycle -->
  </plugin>
</gazebo>
```

## Next Steps

After preparing your model for simulation:

1. **Test thoroughly** in Gazebo environment
2. **Create controllers** for robot behavior
3. **Develop AI-ROS integration** as covered in Chapter 3
4. **Implement motion planning** algorithms
5. **Test with real-world scenarios**

Properly preparing your URDF model for simulation is crucial for successful humanoid robot development. The combination of accurate physical properties, appropriate transmissions, and well-configured plugins ensures realistic and stable simulation behavior.
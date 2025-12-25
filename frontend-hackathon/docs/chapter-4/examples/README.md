# Chapter 4: Humanoid Modeling with URDF Examples

This directory contains executable examples for Chapter 4: Humanoid Modeling with URDF, covering the structure and purpose of URDF files, links and joints concepts, and preparing models for simulation.

## Available Examples

### 1. simple_humanoid.urdf
A basic humanoid robot model with:
- Torso with head
- Two arms (shoulder and elbow joints)
- Two legs (hip and knee joints)
- Basic visual and collision properties
- Realistic inertial properties

### 2. complex_humanoid.urdf
A more advanced humanoid robot model with:
- Complete URDF structure with Gazebo plugins
- Transmissions for ROS control
- IMU sensor for balance
- Proper material definitions
- Realistic joint limits and properties

### 3. urdf_validator.py
A Python script that demonstrates:
- URDF structure validation
- Kinematic chain analysis
- URDF file summary generation
- Best practices for URDF development

## Usage

### Validating URDF Files

To validate and analyze the URDF files:

```bash
# Make sure you're in the frontend-hackathon directory
cd frontend-hackathon

# Run the validator script
python3 docs/chapter-4/examples/urdf_validator.py
```

### Working with URDF in ROS

To load and visualize these URDF models in ROS:

```bash
# Load the URDF into the parameter server
ros2 param set /robot_state_publisher robot_description "$(cat docs/chapter-4/examples/simple_humanoid.urdf)"

# Or use xacro if you convert to xacro format
ros2 run xacro xacro docs/chapter-4/examples/simple_humanoid.urdf > processed.urdf
```

### Testing in Gazebo

To test the complex humanoid model in Gazebo simulation:

1. Make sure Gazebo is installed and sourced
2. Launch Gazebo with an empty world
3. Spawn the robot model:

```bash
# Spawn the robot in Gazebo
ros2 run gazebo_ros spawn_entity.py -file docs/chapter-4/examples/complex_humanoid.urdf -entity humanoid_robot -x 0 -y 0 -z 1
```

## Key Concepts Demonstrated

### URDF Structure
- Robot element with name attribute
- Link elements with visual, collision, and inertial properties
- Joint elements connecting links with different joint types
- Material definitions for visualization

### Joint Types
- **Revolute**: Rotational joints with limits (elbows, knees)
- **Continuous**: Rotational joints without limits (wheels)
- **Fixed**: Rigid connections (sensors mounted to links)

### Physical Properties
- Mass and inertial tensor for dynamics simulation
- Collision geometry for physics interactions
- Visual geometry for rendering

### Gazebo Integration
- Gazebo-specific material definitions
- Sensor plugins (IMU, cameras, etc.)
- Transmission elements for ROS control
- Physics properties (friction, damping)

## Best Practices Shown

1. **Proper naming conventions** for links and joints
2. **Realistic inertial properties** for stable simulation
3. **Appropriate joint limits** to prevent self-collision
4. **Separate collision and visual geometry** for performance
5. **Gazebo plugins** for simulation functionality
6. **Transmission elements** for ROS control

## Extending the Examples

These examples can be extended for specific humanoid robot applications:

1. Add more joints for complete humanoid kinematics
2. Include hand models with finger joints
3. Add additional sensors (cameras, LIDAR, force/torque sensors)
4. Implement complete transmission sets for all joints
5. Add Gazebo worlds for testing scenarios

## Troubleshooting

- If URDF validation fails, check for missing required elements
- If joints don't move in simulation, verify transmission definitions
- If physics behave strangely, review inertial properties
- If visualization looks wrong, check coordinate frames and origins

These examples provide a solid foundation for creating humanoid robot models in URDF format, ready for simulation and control in ROS environments.
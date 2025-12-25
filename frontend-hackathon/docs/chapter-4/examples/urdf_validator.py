#!/usr/bin/env python3

"""
URDF Validation and Processing Script for Chapter 4
This script demonstrates how to validate URDF files and work with robot models
"""

import xml.etree.ElementTree as ET
import os
import sys
from pathlib import Path

def validate_urdf_structure(urdf_path):
    """
    Validate the basic structure of a URDF file
    """
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Check if it's a robot file
        if root.tag != 'robot':
            print(f"ERROR: {urdf_path} is not a valid robot URDF file")
            return False

        robot_name = root.get('name')
        print(f"✓ Robot name: {robot_name}")

        # Count links and joints
        links = root.findall('link')
        joints = root.findall('joint')

        print(f"✓ Found {len(links)} links")
        print(f"✓ Found {len(joints)} joints")

        # Validate each link
        for link in links:
            link_name = link.get('name')
            if not link_name:
                print(f"ERROR: Link without name in {urdf_path}")
                return False

            # Check for required elements
            visual = link.find('visual')
            collision = link.find('collision')
            inertial = link.find('inertial')

            if inertial is None:
                print(f"WARNING: Link '{link_name}' has no inertial element")
            else:
                mass = inertial.find('mass')
                if mass is None or mass.get('value') is None:
                    print(f"WARNING: Link '{link_name}' has no mass defined")

        # Validate each joint
        for joint in joints:
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            parent = joint.find('parent')
            child = joint.find('child')

            if not all([joint_name, joint_type, parent, child]):
                print(f"ERROR: Joint '{joint_name}' missing required elements")
                return False

            if joint_type not in ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']:
                print(f"WARNING: Joint '{joint_name}' has unknown type: {joint_type}")

            if joint_type in ['revolute', 'prismatic']:
                limit = joint.find('limit')
                if limit is None:
                    print(f"WARNING: Joint '{joint_name}' of type {joint_type} has no limits defined")

        print(f"✓ URDF structure validation passed for {urdf_path}")
        return True

    except ET.ParseError as e:
        print(f"ERROR: Invalid XML in {urdf_path}: {e}")
        return False
    except Exception as e:
        print(f"ERROR: Could not parse {urdf_path}: {e}")
        return False

def analyze_kinematic_chain(urdf_path):
    """
    Analyze the kinematic chain structure of the robot
    """
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        joints = root.findall('joint')
        links = root.findall('link')

        # Create a map of parent-child relationships
        joint_map = {}
        for joint in joints:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            joint_type = joint.get('type')
            joint_name = joint.get('name')

            joint_map[child] = {
                'parent': parent,
                'joint_name': joint_name,
                'joint_type': joint_type
            }

        # Find base link (link that is never a child)
        all_children = set()
        all_parents = set()

        for joint in joints:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            all_children.add(child)
            all_parents.add(parent)

        base_links = all_parents - all_children
        print(f"✓ Base links: {list(base_links)}")

        # Print kinematic chains
        for base in base_links:
            print(f"\nKinematic chain starting from {base}:")
            current = base
            depth = 0
            while current:
                # Find all children of current link
                children = []
                for child, info in joint_map.items():
                    if info['parent'] == current:
                        children.append((child, info))

                if children:
                    for child, info in children:
                        indent = "  " * (depth + 1)
                        print(f"{indent}├── {info['joint_name']} ({info['joint_type']}) → {child}")
                        # For simplicity, just follow the first child in case of branching
                        # In a full implementation, you'd want to handle all branches
                        current = child
                        depth += 1
                        break
                else:
                    break

        return True

    except Exception as e:
        print(f"ERROR: Could not analyze kinematic chain: {e}")
        return False

def generate_urdf_summary(urdf_path):
    """
    Generate a summary of the URDF file
    """
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        print(f"\n{'='*50}")
        print(f"URDF SUMMARY: {os.path.basename(urdf_path)}")
        print(f"{'='*50}")

        print(f"Robot Name: {root.get('name')}")

        # Count elements
        links = root.findall('link')
        joints = root.findall('joint')
        materials = root.findall('.//material')
        transmissions = root.findall('transmission')
        gazebo_elements = root.findall('gazebo')

        print(f"Total Links: {len(links)}")
        print(f"Total Joints: {len(joints)}")
        print(f"Materials: {len(materials)}")
        print(f"Transmissions: {len(transmissions)}")
        print(f"Gazebo Elements: {len(gazebo_elements)}")

        # Joint type breakdown
        joint_types = {}
        for joint in joints:
            jtype = joint.get('type')
            joint_types[jtype] = joint_types.get(jtype, 0) + 1

        print("\nJoint Types:")
        for jtype, count in joint_types.items():
            print(f"  {jtype}: {count}")

        # Link properties
        print("\nLink Analysis:")
        for link in links:
            link_name = link.get('name')
            has_visual = link.find('visual') is not None
            has_collision = link.find('collision') is not None
            has_inertial = link.find('inertial') is not None

            properties = []
            if has_visual: properties.append('visual')
            if has_collision: properties.append('collision')
            if has_inertial: properties.append('inertial')

            print(f"  {link_name}: {', '.join(properties) if properties else 'no properties'}")

        return True

    except Exception as e:
        print(f"ERROR: Could not generate summary: {e}")
        return False

def main():
    """
    Main function to run URDF validation and analysis
    """
    print("URDF Validation and Processing Tool")
    print("Chapter 4: Humanoid Modeling with URDF")
    print()

    # Define URDF files to process
    examples_dir = Path("docs/chapter-4/examples")
    urdf_files = list(examples_dir.glob("*.urdf"))

    if not urdf_files:
        print(f"No URDF files found in {examples_dir}")
        return

    print(f"Found {len(urdf_files)} URDF files to process:")
    for urdf_file in urdf_files:
        print(f"  - {urdf_file.name}")

    print()

    # Process each URDF file
    for urdf_file in urdf_files:
        print(f"\n{'='*60}")
        print(f"Processing: {urdf_file}")
        print(f"{'='*60}")

        # Validate structure
        if validate_urdf_structure(urdf_file):
            print("✓ Structure validation: PASSED")
        else:
            print("✗ Structure validation: FAILED")
            continue

        # Generate summary
        if generate_urdf_summary(urdf_file):
            print("✓ Summary generated successfully")
        else:
            print("✗ Summary generation failed")
            continue

        # Analyze kinematic chain
        if analyze_kinematic_chain(urdf_file):
            print("✓ Kinematic chain analysis completed")
        else:
            print("✗ Kinematic chain analysis failed")

        print(f"\n{'='*60}")
        print(f"Completed processing: {urdf_file.name}")
        print(f"{'='*60}")

    print(f"\nProcessing completed for {len(urdf_files)} URDF files.")

if __name__ == "__main__":
    main()
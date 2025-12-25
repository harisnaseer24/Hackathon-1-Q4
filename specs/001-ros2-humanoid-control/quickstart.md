# Quickstart Guide: Physical AI & Humanoid Robotics Module

## Overview
This guide provides a quick setup process for the Physical AI & Humanoid Robotics educational module. This module teaches ROS 2 concepts for humanoid robotics using Docusaurus-based documentation with executable code examples.

## Prerequisites
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- Node.js 18+ and npm/yarn
- Python 3.8+
- Git

## Setup Process

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Install Docusaurus Dependencies
```bash
cd docs/docusaurus
npm install
```

### 3. Set up ROS 2 Environment
Follow the official ROS 2 Humble Hawksbill installation guide for your platform:
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-build-essential
source /opt/ros/humble/setup.bash
```

### 4. Install ROS Python Dependencies
```bash
pip3 install rclpy
pip3 install rosidl_runtime_py
```

### 5. Start Docusaurus Development Server
```bash
cd docs/docusaurus
npm start
```

## Project Structure
```
docs/
├── docusaurus/
│   ├── docs/              # Chapter content (1-4)
│   ├── src/               # Custom components
│   ├── static/            # Images and assets
│   ├── docusaurus.config.js
│   └── package.json
```

## Running Code Examples
Code examples in each chapter can be run in a ROS 2 environment:

1. Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

2. Navigate to example directory and run:
```bash
python3 example_script.py
```

## Building for Production
```bash
cd docs/docusaurus
npm run build
```

## Deployment
The documentation can be deployed to GitHub Pages using the provided GitHub Actions workflow in `.github/workflows/deploy.yml`.

## Troubleshooting
- If you encounter issues with ROS 2 packages, ensure your environment is properly sourced
- For Docusaurus issues, check that all dependencies are installed correctly
- Code examples require a complete ROS 2 installation to run properly
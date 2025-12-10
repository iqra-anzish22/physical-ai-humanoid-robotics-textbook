---
title: ROS 2 Setup and Configuration
sidebar_position: 2
description: Complete guide to installing and configuring ROS 2 for robotics development
keywords: [ros2, installation, setup, configuration, robotics]
learning_outcomes:
  - Install ROS 2 on different operating systems
  - Configure ROS 2 workspace and environment
  - Create and build ROS 2 packages
  - Understand ROS 2 distribution and versioning
---

import LearningOutcome from '@site/src/components/LearningOutcome/LearningOutcome';

# ROS 2 Setup and Configuration

<LearningOutcome outcomes={[
  "Install ROS 2 on different operating systems",
  "Configure ROS 2 workspace and environment",
  "Create and build ROS 2 packages",
  "Understand ROS 2 distribution and versioning"
]} />

## Introduction

Setting up ROS 2 is the first step toward developing robotics applications. This guide will walk you through the installation process on different platforms and the basic configuration needed to start developing with ROS 2.

## ROS 2 Distributions

ROS 2 follows a distribution model similar to Linux distributions, with each distribution having a name (e.g., Foxy, Galactic, Humble, Iron) and a support timeline. The current recommended distribution is **Humble Hawksbill**, which is an LTS (Long Term Support) release supported until 2027.

### Distribution Timeline

- **Humble Hawksbill (humble)**: LTS release, supported until May 2027
- **Iron Irwini (iron)**: Standard release, supported until November 2024
- **Rolling Ridley (rolling)**: Development release, updated continuously

For production and learning purposes, we recommend using Humble Hawksbill.

## Installation on Ubuntu Linux

### System Requirements

- Ubuntu 22.04 (Jammy Jellyfish) - Recommended
- At least 5GB of free disk space
- Internet connection for package downloads

### Installation Steps

1. **Set up locale**
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Set up sources**
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

3. **Add ROS 2 repository**
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **Install ROS 2 packages**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. **Install colcon build tools**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

6. **Install ROS development tools** (optional but recommended)
   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   sudo rosdep init
   rosdep update
   ```

## Installation on Windows

### System Requirements

- Windows 10 (version 1909 or later) or Windows 11
- WSL2 (Windows Subsystem for Linux) with Ubuntu 22.04 recommended
- At least 5GB of free disk space

### Installation Steps

1. **Install WSL2 with Ubuntu 22.04**
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

2. **Follow the Ubuntu installation steps** within the WSL2 terminal

3. **Configure X11 forwarding** for GUI applications (optional)
   - Install an X-server like VcXsrv or X410
   - Set the DISPLAY environment variable

## Installation on macOS

### System Requirements

- macOS 12 (Monterey) or later
- Homebrew package manager
- At least 5GB of free disk space

### Installation Steps

1. **Install Homebrew** (if not already installed)
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. **Install ROS 2 using binary packages**
   ```bash
   # Install dependencies
   brew install python@3.10
   brew install asio cppcheck eigen3 log4cxx opencv pcre poco tinyxml2 uncrustify
   brew install graphviz
   pip3 install -c https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos ros2cli
   ```

## Environment Setup

### Sourcing the ROS 2 Environment

After installation, you need to source the ROS 2 environment in each terminal where you want to use ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add the following line to your `~/.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Creating a ROS 2 Workspace

1. **Create workspace directory**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **Build the workspace**
   ```bash
   colcon build
   ```

3. **Source the workspace**
   ```bash
   source install/setup.bash
   ```

## Package Management

### Creating a New Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclpy std_msgs my_robot_package
```

### Building Packages

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```

## ROS 2 Environment Variables

Key environment variables for ROS 2:

- `ROS_DOMAIN_ID`: Sets the ROS domain ID (default: 0)
- `RMW_IMPLEMENTATION`: Specifies the middleware implementation
- `ROS_LOG_DIR`: Directory for ROS log files
- `ROS_LOCALHOST_ONLY`: Forces communication via localhost only (1) or not (0)

## Verification

To verify your installation, try running the talker/listener demo:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

If you see messages being passed between the talker and listener, your installation is successful!

## Troubleshooting

### Common Issues

1. **Package not found**: Make sure to source the ROS environment before using ROS commands
2. **Permission errors**: Check file permissions and ensure you're using the correct user
3. **Network issues**: ROS 2 uses DDS which requires proper network configuration
4. **Python path conflicts**: Ensure Python packages are installed in the correct environment

## Next Steps

Now that you have ROS 2 installed and configured, you can start exploring ROS 2 concepts and creating your first robotics applications. The next section will cover practical examples and exercises to reinforce your understanding.

## References

1. ROS 2 Documentation. (2023). Installation Guide. Retrieved from https://docs.ros.org/en/humble/Installation.html
2. Open Robotics. (2023). ROS 2 User Guide. ROS Documentation.
3. Quigley, M., Gerkey, B., & Smart, W. (2022). Programming Robots with ROS: A Practical Introduction to the Robot Operating System. O'Reilly Media.
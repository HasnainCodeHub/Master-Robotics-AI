---
id: ros2-installation
title: "Appendix C: ROS 2 Installation"
sidebar_label: "C. ROS 2 Installation"
sidebar_position: 4
---

# Appendix C: ROS 2 Humble Installation Guide

This appendix provides step-by-step instructions for installing ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS.

---

## Prerequisites {#prerequisites}

### System Requirements

| Requirement | Specification |
|-------------|--------------|
| OS | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| Architecture | amd64 (x86_64) |
| RAM | 8 GB minimum, 16 GB recommended |
| Storage | 10 GB free space |

### Verify Ubuntu Version

```bash
lsb_release -a

# Expected output:
# Distributor ID: Ubuntu
# Description:    Ubuntu 22.04.x LTS
# Release:        22.04
# Codename:       jammy
```

---

## Installation Steps {#installation}

### Step 1: Set Locale

```bash
# Check current locale
locale

# Set locale to UTF-8 if not already
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Enable Universe Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Step 3: Add ROS 2 GPG Key

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Step 4: Add ROS 2 Repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 5: Update Package Index

```bash
sudo apt update
sudo apt upgrade
```

### Step 6: Install ROS 2 Humble

Choose one of the following options:

**Desktop Install (Recommended)** - ROS 2 + RViz + demos:

```bash
sudo apt install ros-humble-desktop
```

**Base Install** - ROS 2 without GUI tools:

```bash
sudo apt install ros-humble-ros-base
```

**Full Install** - Everything including simulation:

```bash
sudo apt install ros-humble-desktop-full
```

### Step 7: Install Development Tools

```bash
sudo apt install ros-dev-tools
```

---

## Environment Setup {#environment}

### Source ROS 2 in Current Terminal

```bash
source /opt/ros/humble/setup.bash
```

### Add to Shell Configuration (Permanent)

For Bash:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

For Zsh:
```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

### Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# Expected output: ros2 0.14.x (or similar)
```

---

## Verify with Examples {#verification}

### Test Publisher-Subscriber

Open two terminals.

**Terminal 1 (Publisher)**:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Subscriber)**:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages flowing from talker to listener.

### Test RViz

```bash
source /opt/ros/humble/setup.bash
rviz2
```

RViz should open with a 3D visualization window.

### Test TurtleSim

**Terminal 1**:
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2**:
```bash
ros2 run turtlesim turtle_teleop_key
```

Use arrow keys to control the turtle.

---

## Workspace Setup {#workspace}

### Create a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build (creates build, install, log directories)
colcon build

# Source workspace
source install/setup.bash
```

### Add Workspace to Shell

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Workspace

```bash
# Check workspace is sourced
echo $COLCON_PREFIX_PATH

# Should include: /home/<user>/ros2_ws/install
```

---

## Common Packages {#packages}

### Navigation (Nav2)

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### MoveIt 2

```bash
sudo apt install ros-humble-moveit
```

### Gazebo Integration

```bash
sudo apt install ros-humble-ros-gz
```

### Visualization

```bash
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-rqt*
```

### Robot Description

```bash
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
```

---

## Troubleshooting {#troubleshooting}

### "Command not found: ros2"

**Cause**: ROS 2 not sourced.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc for permanence
```

### "Package not found"

**Cause**: Package not installed or workspace not sourced.

**Solution**:
```bash
# Check if package exists
apt search ros-humble-<package>

# Install if needed
sudo apt install ros-humble-<package>

# Rebuild and source workspace
cd ~/ros2_ws && colcon build && source install/setup.bash
```

### "Failed to create node"

**Cause**: ROS domain ID conflict.

**Solution**:
```bash
# Set unique domain ID (0-232)
export ROS_DOMAIN_ID=42
```

### "Cannot connect to nodes on other machine"

**Cause**: Firewall or network configuration.

**Solution**:
```bash
# Check ROS domain ID matches on all machines
echo $ROS_DOMAIN_ID

# Disable firewall for testing (not recommended for production)
sudo ufw disable
```

### Dependency Issues

```bash
# Fix broken dependencies
sudo apt --fix-broken install

# Update and clean
sudo apt update && sudo apt upgrade
sudo apt autoremove
```

### Build Failures

```bash
# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build

# Build specific package with debug
colcon build --packages-select <package> --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

---

## ROS 2 CLI Quick Reference {#cli-reference}

### Node Commands

```bash
ros2 node list          # List active nodes
ros2 node info <node>   # Show node details
```

### Topic Commands

```bash
ros2 topic list         # List topics
ros2 topic info <topic> # Show topic details
ros2 topic echo <topic> # Print topic messages
ros2 topic pub <topic> <type> <data>  # Publish message
```

### Service Commands

```bash
ros2 service list       # List services
ros2 service type <srv> # Show service type
ros2 service call <srv> <type> <args> # Call service
```

### Action Commands

```bash
ros2 action list        # List actions
ros2 action info <act>  # Show action details
ros2 action send_goal <act> <type> <goal> # Send goal
```

### Package Commands

```bash
ros2 pkg list           # List packages
ros2 pkg executables    # List executables
ros2 run <pkg> <exec>   # Run executable
```

---

## Next Steps

After installing ROS 2:

1. **Create your first package**: Follow Module 2, Chapter 1
2. **Set up simulation**: See [Appendix D: Simulation Installation](/appendix/simulation-installation)
3. **Configure development tools**: Install VS Code with ROS extension

---

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Discourse (Community)](https://discourse.ros.org/)
- [ROS Answers (Q&A)](https://answers.ros.org/)

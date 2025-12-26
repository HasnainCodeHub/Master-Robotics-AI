---
id: simulation-installation
title: "Appendix D: Simulation Installation"
sidebar_label: "D. Simulation Installation"
sidebar_position: 5
---

# Appendix D: Simulation Environment Installation

This appendix provides installation guides for Gazebo and NVIDIA Isaac Sim, the two primary simulation environments used in this course.

---

## Gazebo Harmonic Installation {#gazebo}

Gazebo (formerly Ignition Gazebo) is an open-source robotics simulator with excellent ROS 2 integration.

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble installed (see [Appendix C](/appendix/ros2-installation))

### Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt update
sudo apt install gz-harmonic
```

### Install ROS 2 - Gazebo Integration

```bash
sudo apt install ros-humble-ros-gz
```

### Verify Gazebo Installation

```bash
# Launch empty world
gz sim

# Check version
gz sim --version
```

### Test ROS 2 Integration

**Terminal 1**:
```bash
source /opt/ros/humble/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
```

**Terminal 2**:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
# Should show /clock and Gazebo-related topics
```

### Gazebo Troubleshooting

**Black screen or crash**:
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"

# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gz sim
```

**Missing models**:
```bash
# Download fuel models
gz fuel download -u https://fuel.gazebosim.org/1.0/openrobotics/models/ground_plane
```

---

## NVIDIA Isaac Sim Installation {#isaac-sim}

Isaac Sim is NVIDIA's GPU-accelerated robotics simulator with RTX ray tracing and PhysX 5.

### Prerequisites

| Requirement | Specification |
|-------------|--------------|
| OS | Ubuntu 22.04 LTS |
| GPU | NVIDIA RTX (2070 minimum, 3070+ recommended) |
| Driver | 525.60+ (535+ recommended) |
| VRAM | 8 GB minimum, 16 GB recommended |
| RAM | 32 GB minimum, 64 GB recommended |
| Storage | 50 GB free space |

### Step 1: Install NVIDIA Driver

```bash
# Check current driver
nvidia-smi

# If not installed or outdated:
sudo apt update
sudo apt install nvidia-driver-535

# Reboot
sudo reboot
```

### Step 2: Install Omniverse Launcher

1. Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/download/)
2. Make executable and run:

```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

3. Create NVIDIA account and log in

### Step 3: Install Isaac Sim via Launcher

1. Open Omniverse Launcher
2. Go to **Exchange** tab
3. Search for "Isaac Sim"
4. Click **Install**
5. Wait for download (~30 GB)

### Step 4: Verify Installation

Launch Isaac Sim from Omniverse Launcher:

1. Go to **Library** tab
2. Click **Launch** on Isaac Sim
3. Wait for first-time shader compilation (5-15 minutes)
4. Create new stage and verify rendering

### Step 5: Install Isaac ROS

```bash
# Add Isaac ROS repository
sudo apt update
sudo apt install curl gnupg lsb-release

# Add key
sudo curl -sSL https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac_ros_common/main/scripts/install_nvidia_key.sh | bash

# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-object-detection
```

### Isaac Sim Configuration

**Environment Variables** (add to ~/.bashrc):

```bash
# Isaac Sim path
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac_sim-2023.1.1"

# Python path for Isaac Sim
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# ROS 2 workspace
alias isaacsim="${ISAACSIM_PATH}/isaac-sim.sh"
```

**First Launch Optimizations**:

```bash
# Disable nucleus local server if not needed (saves resources)
# In Isaac Sim: Edit > Preferences > Omniverse > disable local server

# Set to low-spec mode for testing
# In Isaac Sim: Edit > Preferences > Rendering > set to "Low"
```

### Isaac Sim Troubleshooting

**"GPU not supported"**:
```bash
# Verify RTX GPU
nvidia-smi -q | grep "Product Name"
# Must show RTX series

# Check CUDA
nvcc --version
# Should show 11.x or 12.x
```

**Out of memory**:
```bash
# Monitor VRAM usage
watch -n 1 nvidia-smi

# Reduce scene complexity
# Use "Low" rendering preset
# Close other GPU applications
```

**Slow first launch**:
- First launch compiles shaders (normal)
- Subsequent launches faster
- Consider SSD for faster asset loading

**ROS 2 bridge not connecting**:
```bash
# Enable ROS 2 bridge in Isaac Sim
# Window > Extensions > search "ROS2 Bridge" > Enable

# Verify topics
ros2 topic list
```

**Camera not publishing**:
```bash
# Ensure camera is added to viewport
# Enable "Publish RGB" in camera properties
# Check topic: ros2 topic echo /rgb
```

---

## Verification Tests {#verification}

### Test 1: Gazebo with TurtleBot

```bash
# Install TurtleBot packages
sudo apt install ros-humble-turtlebot3*

# Set model
export TURTLEBOT3_MODEL=burger

# Launch simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Test 2: Isaac Sim with ROS 2

1. Open Isaac Sim
2. Load sample: **Isaac Examples > ROS2 > Navigation**
3. Click **Play**
4. Verify ROS 2 topics:

```bash
ros2 topic list
# Should show /scan, /odom, /cmd_vel, etc.
```

### Test 3: Run VSLAM

```bash
# Launch VSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py

# Check odometry output
ros2 topic echo /visual_slam/tracking/odometry
```

---

## Performance Optimization {#optimization}

### Gazebo Performance

```bash
# Limit physics rate
gz sim -r --physics-rate 250

# Reduce sensor rates in SDF
# <update_rate>30</update_rate>

# Use simpler collision meshes
# <collision><geometry><box>... instead of mesh
```

### Isaac Sim Performance

| Setting | Low Spec | Medium | High |
|---------|----------|--------|------|
| Render Resolution | 720p | 1080p | 4K |
| Ray Tracing | Off | Path Tracing | Full |
| Physics Rate | 60 Hz | 120 Hz | 240 Hz |
| Sensor Rate | 10 Hz | 30 Hz | 60 Hz |

**Access Settings**: Edit > Preferences > Rendering

---

## Docker Installation (Alternative) {#docker}

### Gazebo in Docker

```bash
# Pull ROS 2 + Gazebo image
docker pull osrf/ros:humble-desktop

# Run with GUI
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop
```

### Isaac Sim in Docker

```bash
# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run with GPU
docker run --gpus all -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

---

## Summary

| Simulator | Best For | Requirements |
|-----------|----------|--------------|
| Gazebo | Learning, light simulation | Any GPU |
| Isaac Sim | Realistic sim, perception, sim-to-real | RTX GPU, 8GB+ VRAM |

For this course:
- Use **Gazebo** for Module 3 (basics, prototyping)
- Use **Isaac Sim** for Modules 4-5 and Capstone (full pipeline)

---

## Additional Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/)

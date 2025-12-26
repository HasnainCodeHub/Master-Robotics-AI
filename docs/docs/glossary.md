---
id: glossary
title: "Glossary"
sidebar_label: "Glossary"
sidebar_position: 100
---

# Glossary of Physical AI & Robotics Terms

This glossary defines key technical terms used throughout the textbook. Terms are organized alphabetically for easy reference.

---

## A

### Action (ROS 2)
A ROS 2 communication pattern for long-running tasks with feedback. Actions allow cancellation and progress monitoring, unlike services which are blocking. Example: navigation to a goal provides distance feedback during execution.

### Actuator
A component that converts control signals into physical motion or force. Common types include electric motors, hydraulic cylinders, and pneumatic actuators. Key parameters: torque, speed, position accuracy.

### AMCL (Adaptive Monte Carlo Localization)
A probabilistic localization algorithm that estimates robot pose using a particle filter. Commonly used with 2D LiDAR for indoor navigation.

---

## B

### Base Link
The root link in a robot's kinematic chain, typically attached to the main body. All other links are defined relative to this frame.

---

## C

### Collision Avoidance
Techniques to prevent a robot from colliding with obstacles or itself. Includes reactive methods (stopping when too close) and predictive methods (planning collision-free paths).

### Costmap
A grid-based representation of traversability used for navigation. Cells contain cost values indicating obstacles, inflation zones, or free space.

### Control Loop
The continuous cycle of reading sensors, computing control outputs, and commanding actuators. Loop frequency determines response speed and stability.

---

## D

### Depth Camera
A sensor that captures depth (distance) information per pixel. Technologies include structured light (e.g., RealSense), time-of-flight, and stereo vision. Output: depth images or point clouds.

### Digital Twin
A virtual replica of a physical system that mirrors its state and behavior. Used for testing, monitoring, and simulation.

### Domain Randomization
A sim-to-real technique that varies simulation parameters (textures, lighting, physics) during training to improve transfer to reality.

### DOF (Degrees of Freedom)
The number of independent parameters needed to specify a robot's configuration. A 6-DOF arm can reach positions and orientations in 3D space.

---

## E

### Embodied Intelligence
AI systems that exist within physical bodies and must interact with the physical world through sensors and actuators, as opposed to disembodied AI that operates purely in software.

### End Effector
The device at the end of a robot arm that interacts with the environment. Examples: grippers, suction cups, welding tools.

---

## F

### Forward Kinematics
Computing the end effector pose given joint angles. Unambiguous: each joint configuration produces exactly one pose.

### Frame (Coordinate)
A coordinate system attached to a point on the robot or in the environment. Transformations between frames enable spatial reasoning.

---

## G

### Gazebo
An open-source robotics simulator supporting physics, sensors, and ROS integration. Part of the Open Robotics ecosystem.

### Grounding (VLA)
The process of mapping symbolic references (e.g., "red mug") to physical quantities (e.g., 3D position). Connects language to robot perception.

### Gripper
An end effector designed to grasp objects. Types include parallel jaw, three-finger, and suction grippers.

---

## H

### Hallucination
When an AI system generates plausible but factually incorrect information. In robotics, VLMs may hallucinate objects that don't exist.

### HITL (Human-in-the-Loop)
System design where human operators can intervene, provide guidance, or override automated decisions. Essential for safety-critical operations.

---

## I

### IMU (Inertial Measurement Unit)
A sensor combining accelerometers and gyroscopes to measure linear acceleration and angular velocity. Used for odometry and state estimation.

### Inverse Kinematics (IK)
Computing joint angles that achieve a desired end effector pose. May have multiple solutions or no solution.

### Isaac ROS
NVIDIA's GPU-accelerated ROS packages for perception and navigation. Uses NITROS for zero-copy GPU pipeline communication.

### Isaac Sim
NVIDIA's GPU-based robotics simulator built on Omniverse. Features RTX rendering, PhysX physics, and synthetic data generation.

---

## J

### Joint
A connection between two links that allows relative motion. Types: revolute (rotation), prismatic (linear), fixed (no motion).

### Joint State
The current position, velocity, and effort of each robot joint. Published on `/joint_states` in ROS 2.

---

## L

### Latency
Time delay between input and output. In robotics, includes sensor capture, processing, communication, and actuation delays. Critical for real-time control.

### LiDAR (Light Detection and Ranging)
A sensor that measures distances using laser pulses. Produces 2D scans or 3D point clouds. Key parameters: range, resolution, scan rate.

### Link
A rigid body segment in a robot's kinematic chain. Connected by joints.

### LLM (Large Language Model)
A neural network trained on text data that can understand and generate natural language. Used in VLA for task planning.

---

## M

### MoveIt 2
A motion planning framework for ROS 2. Provides inverse kinematics, collision checking, and path planning for manipulation.

---

## N

### Nav2
The ROS 2 navigation stack. Provides autonomous navigation including localization, path planning, and obstacle avoidance.

### NITROS
NVIDIA Isaac Transport for ROS. A framework for GPU-accelerated, zero-copy communication between ROS nodes.

### Node (ROS 2)
An executable that performs computation. Nodes communicate via topics, services, and actions. Each node should have a single responsibility.

---

## O

### Odometry
Estimation of position change over time using wheel encoders, IMU, or visual features. Subject to drift without absolute references.

### OMPL (Open Motion Planning Library)
A library of sampling-based motion planning algorithms (RRT, PRM, etc.) used by MoveIt for path planning.

---

## P

### Perception-Action Loop
The continuous cycle of sensing the environment, processing information, deciding on actions, and executing them. Fundamental to all robotic systems.

### PhysX
NVIDIA's physics simulation engine. Used in Isaac Sim for rigid body dynamics, articulations, and contact physics.

### Point Cloud
A set of 3D points representing a surface or scene. Generated by LiDAR or depth cameras.

### Pose
A combination of position (x, y, z) and orientation (quaternion or Euler angles). Represents an object's location and facing direction in space.

---

## Q

### QoS (Quality of Service)
ROS 2 communication settings controlling reliability, durability, and history. Configured per topic to match sensor/control requirements.

### Quaternion
A four-component representation of 3D rotation (x, y, z, w). Avoids gimbal lock and is numerically stable.

---

## R

### Reality Gap
The difference between simulated and real-world behavior. Caused by simplified physics, sensor models, and unmodeled dynamics. A key challenge in sim-to-real transfer.

### ROS 2 (Robot Operating System 2)
A middleware framework for robotics providing communication, tools, and libraries. The standard for robotics software development.

### RTF (Real-Time Factor)
The ratio of simulated time to wall-clock time. RTF=1.0 means simulation runs at real speed; RTF \< 1.0 means slower.

---

## S

### Service (ROS 2)
A ROS 2 communication pattern for synchronous request-response interactions. Blocking: the caller waits for a response.

### Sim-to-Real Transfer
Deploying policies or models trained in simulation to physical robots. Requires addressing the reality gap.

### SLAM (Simultaneous Localization and Mapping)
Algorithms that build a map while simultaneously localizing within it. Essential for autonomous navigation in unknown environments.

---

## T

### tf2
ROS 2's transform library. Manages coordinate frame relationships over time, enabling spatial reasoning across the robot.

### Topic (ROS 2)
A ROS 2 communication channel for asynchronous publish-subscribe messaging. Named streams of typed messages.

### Transform
The mathematical relationship (translation and rotation) between two coordinate frames.

---

## U

### URDF (Unified Robot Description Format)
An XML format describing robot structure: links, joints, visuals, collisions, and inertias. Used for simulation and motion planning.

### USD (Universal Scene Description)
Pixar's scene description format adopted by NVIDIA Omniverse. Supports composition, layering, and real-time collaboration.

---

## V

### VLA (Vision-Language-Action)
Systems that combine vision understanding, language processing, and action execution. Enable natural language control of robots.

### VLM (Vision-Language Model)
Neural networks that process both images and text, enabling visual question answering and scene understanding.

### VSLAM (Visual SLAM)
SLAM using camera data (monocular, stereo, or RGB-D) rather than LiDAR. Provides rich feature information but is sensitive to lighting.

---

## W

### Whisper
OpenAI's automatic speech recognition model. Used in VLA for voice command input.

### Workspace
The set of all positions reachable by a robot's end effector. Defined by kinematic limits and obstacle locations.

---

## X-Z

### Xacro
An XML macro language extending URDF. Enables reusable, parameterized robot descriptions.

### Zero-Copy
Memory optimization where data is shared rather than copied between processes. Used in NITROS for GPU pipeline efficiency.

---

## Related Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **MoveIt 2 Documentation**: https://moveit.picknik.ai/
- **Isaac Sim Documentation**: https://docs.omniverse.nvidia.com/isaacsim/
- **Gazebo Documentation**: https://gazebosim.org/docs

---
id: chapter-5-urdf-robot-description
title: "Chapter 5: URDF and Robot Description"
sidebar_label: "5. URDF & Robot Description"
sidebar_position: 6
---

# Chapter 5: URDF and Robot Description

## Chapter Goal

By the end of this chapter, you will be able to **write and validate URDF descriptions for articulated robots**, including visual geometry, collision geometry, joint definitions with limits, and sensor frame attachments.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 5.1 | Write URDF links with visual and collision geometry for robot components |
| 5.2 | Write URDF joints connecting links with appropriate joint types and limits |
| 5.3 | Attach sensor frames to URDF with correct transformations |
| 5.4 | Validate URDF and visualize in RViz2 |
| 5.5 | Explain the relationship between URDF, joint_states, and tf2 |

---

## Why Robots Need Formal Descriptions

Every robot has physical structure:
- Links (rigid bodies)
- Joints (connections that allow motion)
- Sensors (attached at specific locations)

This structure affects everything:
- Where is the camera relative to the base?
- How far can each joint move?
- What does the robot look like for visualization?

**URDF (Unified Robot Description Format)** captures this structure in XML, enabling:
- Visualization in RViz2
- Simulation in Gazebo (Module 3)
- Motion planning with MoveIt2
- Transform computation with tf2

---

## URDF Structure Overview {#urdf-overview}

A URDF file contains:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links: rigid bodies -->
  <link name="base_link">...</link>
  <link name="arm_link">...</link>

  <!-- Joints: connections between links -->
  <joint name="arm_joint" type="revolute">...</joint>

  <!-- Materials: colors/textures -->
  <material name="blue">...</material>
</robot>
```

### The Kinematic Tree

URDF describes a **tree structure**:
- One root link (usually `base_link`)
- Each joint connects a parent link to a child link
- No loops allowed (tree, not graph)

```
base_link (root)
    │
    ├── shoulder_joint ── upper_arm_link
    │                         │
    │                         └── elbow_joint ── lower_arm_link
    │                                               │
    │                                               └── wrist_joint ── hand_link
    │
    └── camera_joint ── camera_link
```

---

## Links: Rigid Bodies {#links}

A link represents a rigid body with visual appearance and collision geometry.

### Basic Link Structure

```xml
<link name="base_link">
  <!-- Visual: what you see in RViz -->
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.4 0.3 0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>

  <!-- Collision: for physics simulation and planning -->
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.42 0.32 0.22"/>  <!-- Slightly larger for safety -->
    </geometry>
  </collision>

  <!-- Inertial: mass and inertia for dynamics -->
  <inertial>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Geometry Types

```xml
<!-- Primitive shapes -->
<geometry>
  <box size="x y z"/>           <!-- Dimensions in meters -->
  <cylinder radius="r" length="l"/>
  <sphere radius="r"/>
</geometry>

<!-- Mesh from file -->
<geometry>
  <mesh filename="package://my_robot/meshes/base.stl" scale="1.0 1.0 1.0"/>
</geometry>
```

### Visual vs Collision Geometry

| Aspect | Visual | Collision |
|--------|--------|-----------|
| Purpose | Display | Physics, planning |
| Detail | High (meshes OK) | Low (primitives preferred) |
| Size | Exact | Often slightly larger |
| Required? | Optional | Optional (but recommended) |

**Physical Grounding**: Collision geometry should include **safety margins** based on M1 position uncertainty. If your arm has ±5mm position accuracy, collision boxes should be at least 5mm larger.

---

## Joints: Connections {#joints}

Joints define how links move relative to each other.

### Joint Types

| Type | Motion | Example |
|------|--------|---------|
| `fixed` | No motion | Camera mount |
| `revolute` | Rotation with limits | Elbow joint |
| `continuous` | Rotation without limits | Wheel |
| `prismatic` | Linear with limits | Linear actuator |
| `floating` | 6-DOF free motion | Drone |
| `planar` | 2D translation + rotation | (Rare) |

### Revolute Joint Example

```xml
<joint name="elbow_joint" type="revolute">
  <!-- Parent and child links -->
  <parent link="upper_arm_link"/>
  <child link="lower_arm_link"/>

  <!-- Joint axis (local to child frame) -->
  <axis xyz="0 1 0"/>  <!-- Rotate around Y axis -->

  <!-- Position of joint relative to parent -->
  <origin xyz="0 0 0.3" rpy="0 0 0"/>

  <!-- Limits from M1 actuator specifications -->
  <limit
    lower="-1.57"    <!-- -90 degrees in radians -->
    upper="1.57"     <!-- +90 degrees -->
    velocity="1.0"   <!-- rad/s max velocity -->
    effort="50.0"    <!-- Nm max torque -->
  />

  <!-- Optional dynamics -->
  <dynamics damping="0.1" friction="0.05"/>
</joint>
```

### Joint Limits from M1 Specifications

**Physical Grounding**: Joint limits come directly from M1 actuator analysis:

| Parameter | Source | Example Value |
|-----------|--------|---------------|
| `lower/upper` | Mechanical range of motion | ±90° = ±1.57 rad |
| `velocity` | Motor speed × gear ratio | 60 rpm = 6.28 rad/s |
| `effort` | Motor torque × gear ratio | 0.5 Nm × 100 = 50 Nm |

---

## Sensor Frames {#sensor-frames}

Sensors need frames defining their position and orientation.

### Camera Frame Example

```xml
<!-- Camera mount (fixed joint) -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0.2 0"/>  <!-- Tilted down 0.2 rad -->
</joint>

<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.03 0.08 0.03"/>
    </geometry>
    <material name="black"/>
  </visual>
</link>

<!-- Camera optical frame (Z forward, standard for cameras) -->
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
  <!-- Rotate to camera optical conventions: Z forward, X right, Y down -->
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
</joint>

<link name="camera_optical_frame"/>
```

### IMU Frame Example

```xml
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- IMU on top of base -->
</joint>

<link name="imu_link"/>
```

**Physical Grounding**: Sensor frame positions must match physical mounting. A camera published as `camera_link` must actually be at that position—errors here cause perception failures.

---

## tf2: The Transform System {#tf2}

URDF defines static structure. **tf2** provides dynamic transforms at runtime.

### How URDF Connects to tf2

1. **robot_state_publisher** reads URDF
2. It publishes **static transforms** (fixed joints)
3. **joint_state_publisher** publishes joint positions
4. **robot_state_publisher** computes and publishes transforms for movable joints

```bash
# Launch robot description
ros2 launch my_robot display.launch.py

# View transform tree
ros2 run tf2_tools view_frames

# Lookup specific transform
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

### Transform Lookup in Code

```python
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Create tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_camera_pose(self):
        """Get camera position relative to base."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_optical_frame',
                rclpy.time.Time(),  # Latest available
                timeout=Duration(seconds=1.0)
            )
            return transform
        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
            return None
```

---

## Complete URDF Example {#complete-example}

A simple 2-DOF arm with camera:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.12" length="0.06"/>
      </geometry>
    </collision>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="1.0" effort="10.0"/>
  </joint>

  <!-- Upper arm -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.06 0.06 0.32"/>
      </geometry>
    </collision>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" velocity="1.5" effort="8.0"/>
  </joint>

  <!-- Lower arm -->
  <link name="lower_arm">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.04 0.04 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.22"/>
      </geometry>
    </collision>
  </link>

  <!-- Camera mount -->
  <joint name="camera_joint" type="fixed">
    <parent link="lower_arm"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.2" rpy="0 0.5 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Camera optical frame -->
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <link name="camera_optical_frame"/>

</robot>
```

---

## Validation and Visualization {#validation}

### Check URDF Syntax

```bash
# Install check tool
sudo apt install liburdfdom-tools

# Check URDF
check_urdf simple_arm.urdf
```

### Visualize in RViz2

Create a launch file:

```python
# launch/display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'simple_arm.urdf'
    )

    return LaunchDescription([
        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Joint state publisher (GUI for testing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config/display.rviz']
        ),
    ])
```

---

## Summary

This chapter covered URDF for robot description:

1. **Links** define rigid bodies with visual appearance, collision geometry, and inertial properties.

2. **Joints** connect links and define motion type, limits, and dynamics—limits come from M1 actuator specifications.

3. **Sensor frames** must be placed accurately in URDF—errors cause perception failures.

4. **tf2** provides runtime transforms computed from URDF and joint states.

5. **Validation** with `check_urdf` and visualization in RViz2 catches errors before they cause problems.

---

## Self-Assessment Questions

1. **Link Design**: You need to add a gripper to your arm URDF. It has two fingers that open/close symmetrically. What joint type(s) would you use?

2. **Joint Limits**: Your servo motor specs show: max rotation 270°, max speed 60 rpm, stall torque 2.5 kg·cm. Convert these to URDF limit parameters.

3. **Sensor Frames**: A camera is mounted 10cm forward and 5cm up from the robot base, tilted down 15°. Write the joint/link URDF elements.

4. **tf2 Usage**: Your perception node needs to transform a detected object from camera frame to base frame. What tf2 function would you use?

5. **Validation**: Your URDF loads but the arm appears detached in RViz. What is the most likely cause?

---

## What's Next

In [Chapter 6: Integration Patterns](/module-2/chapter-6-integration-patterns), you'll synthesize all ROS 2 communication patterns into complete Python agents that implement perception-action loops.

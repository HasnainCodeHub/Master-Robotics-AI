---
id: chapter-2-gazebo-world-building
title: "Chapter 2: Gazebo World Building"
sidebar_label: "2. Gazebo World Building"
sidebar_position: 3
---

# Chapter 2: Gazebo World Building with SDF

## Chapter Goal

By the end of this chapter, you will be able to **master SDF (Simulation Description Format) for creating simulation environments** with terrain, obstacles, lighting, and environmental features.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 2.1 | Write SDF files that define world geometry with correct coordinate frames |
| 2.2 | Configure physics properties (friction, restitution) for world objects |
| 2.3 | Add lighting that affects simulated cameras |
| 2.4 | Import mesh assets and configure collision geometry |
| 2.5 | Organize complex worlds using includes and model composition |

---

## Why Worlds Matter

A robot navigation algorithm that works in an empty world will fail in a cluttered warehouse. The environment shapes robot behavior:

- **Floor friction** determines if wheels slip
- **Obstacle density** affects path planning
- **Lighting** changes camera perception
- **Object physics** affects manipulation

Your simulation is only as good as your world.

---

## SDF Structure Overview {#sdf-structure}

SDF (Simulation Description Format) is Gazebo's native world description language.

### Basic Structure

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="my_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <real_time_factor>1.0</real_time_factor>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <!-- Light configuration -->
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <!-- Model definition -->
    </model>

    <!-- Other models -->
    <model name="obstacle_1">
      <!-- Model definition -->
    </model>

  </world>
</sdf>
```

### Hierarchy

```
<world>
├── <physics>      → Simulation parameters
├── <light>        → Lighting sources
├── <model>        → Physical objects
│   ├── <link>     → Rigid body
│   │   ├── <visual>     → Appearance
│   │   ├── <collision>  → Physics shape
│   │   └── <inertial>   → Mass properties
│   └── <joint>    → Connections
└── <include>      → Reference external models
```

---

## Creating Ground Planes {#ground-planes}

Every world needs a ground. Here's a proper ground plane:

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <!-- Visual: what you see -->
    <visual name="visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>

    <!-- Collision: what physics uses -->
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>

      <!-- Surface properties affect robot interaction -->
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>      <!-- Friction coefficient -->
            <mu2>1.0</mu2>    <!-- Secondary friction -->
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.0</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
  </link>
</model>
```

### Friction Configuration

**Physical Grounding**: Floor friction directly affects robot locomotion.

| Surface Type | mu Value | Effect on Robot |
|-------------|----------|-----------------|
| Concrete | 0.8-1.0 | Good traction |
| Wood | 0.4-0.6 | Moderate traction |
| Tile | 0.3-0.4 | Low traction |
| Ice | 0.05-0.1 | Very low traction |

```xml
<!-- Warehouse concrete floor -->
<surface>
  <friction>
    <ode>
      <mu>0.9</mu>
      <mu2>0.9</mu2>
    </ode>
  </friction>
</surface>

<!-- Slippery tile -->
<surface>
  <friction>
    <ode>
      <mu>0.3</mu>
      <mu2>0.3</mu2>
    </ode>
  </friction>
</surface>
```

---

## Static Obstacles {#static-obstacles}

Static objects don't move during simulation.

### Primitive Shapes

```xml
<!-- Box obstacle -->
<model name="box_obstacle">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>1.0 0.5 1.0</size>  <!-- x y z dimensions -->
        </box>
      </geometry>
      <material>
        <ambient>0.3 0.3 0.8 1</ambient>
        <diffuse>0.3 0.3 0.8 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1.0 0.5 1.0</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>

<!-- Cylinder obstacle -->
<model name="cylinder_obstacle">
  <static>true</static>
  <pose>0 3 0.75 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.25</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.25</radius>
          <length>1.5</length>
        </cylinder>
      </geometry>
    </collision>
  </link>
</model>
```

### Coordinate Frames

```
World Frame (fixed)
    │
    ├── X: forward (red axis)
    ├── Y: left (green axis)
    └── Z: up (blue axis)

Pose format: <pose>x y z roll pitch yaw</pose>
    - Position: meters
    - Orientation: radians (roll=X, pitch=Y, yaw=Z)
```

---

## Lighting Configuration {#lighting}

Lighting affects simulated cameras. Configure it to match your deployment environment.

### Directional Light (Sun)

```xml
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.1 -0.9</direction>
</light>
```

### Point Light (Indoor)

```xml
<light type="point" name="ceiling_light">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>1.0 1.0 0.9 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
</light>
```

### Environment Lighting Profiles

```xml
<!-- Outdoor daylight -->
<light type="directional" name="sun">
  <diffuse>1.0 1.0 1.0 1</diffuse>
  <direction>-0.3 0.2 -0.9</direction>
</light>

<!-- Indoor warehouse (fluorescent) -->
<light type="point" name="warehouse_light_1">
  <pose>5 5 4 0 0 0</pose>
  <diffuse>0.9 0.9 1.0 1</diffuse>  <!-- Slightly blue -->
  <attenuation>
    <range>15</range>
  </attenuation>
</light>

<!-- Dim storage room -->
<light type="point" name="dim_light">
  <pose>0 0 2.5 0 0 0</pose>
  <diffuse>0.3 0.3 0.3 1</diffuse>
  <attenuation>
    <range>5</range>
  </attenuation>
</light>
```

**Physical Grounding**: Camera sensor performance depends on lighting. Test your perception algorithms under varied lighting conditions.

---

## Importing Mesh Assets {#meshes}

For complex objects, use mesh files.

### Mesh Model Definition

```xml
<model name="shelf">
  <static>true</static>
  <pose>5 2 0 0 0 0</pose>
  <link name="link">
    <!-- Visual: detailed mesh -->
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>file://meshes/shelf.dae</uri>
          <scale>1.0 1.0 1.0</scale>
        </mesh>
      </geometry>
    </visual>

    <!-- Collision: simplified geometry -->
    <collision name="collision">
      <geometry>
        <box>
          <size>2.0 0.5 2.0</size>
        </box>
      </geometry>
    </collision>

    <inertial>
      <mass>50.0</mass>
    </inertial>
  </link>
</model>
```

### Visual vs Collision Geometry

| Aspect | Visual | Collision |
|--------|--------|-----------|
| Purpose | Rendering | Physics |
| Detail | High (thousands of triangles) | Low (primitives) |
| Performance | Affects GPU | Affects physics CPU |

**Best Practice**: Use simplified collision geometry:

```xml
<!-- Complex chair mesh -->
<visual name="visual">
  <geometry>
    <mesh>
      <uri>file://meshes/chair_detailed.dae</uri>
    </mesh>
  </geometry>
</visual>

<!-- Simplified collision -->
<collision name="collision_seat">
  <pose>0 0 0.45 0 0 0</pose>
  <geometry>
    <box><size>0.45 0.45 0.05</size></box>
  </geometry>
</collision>
<collision name="collision_back">
  <pose>0 -0.2 0.7 0 0 0</pose>
  <geometry>
    <box><size>0.45 0.05 0.5</size></box>
  </geometry>
</collision>
<collision name="collision_legs">
  <pose>0 0 0.22 0 0 0</pose>
  <geometry>
    <cylinder><radius>0.2</radius><length>0.45</length></cylinder>
  </geometry>
</collision>
```

---

## Model Composition {#composition}

Organize complex worlds using reusable models and includes.

### Model Files

Create a reusable model in its own file:

```xml
<!-- models/warehouse_shelf/model.sdf -->
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="warehouse_shelf">
    <static>true</static>
    <link name="frame">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://warehouse_shelf/meshes/shelf.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>2.0 0.6 2.4</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Including Models in World

```xml
<world name="warehouse">
  <!-- Include multiple instances of the same model -->
  <include>
    <uri>model://warehouse_shelf</uri>
    <name>shelf_row1_1</name>
    <pose>2 0 0 0 0 0</pose>
  </include>

  <include>
    <uri>model://warehouse_shelf</uri>
    <name>shelf_row1_2</name>
    <pose>2 3 0 0 0 0</pose>
  </include>

  <include>
    <uri>model://warehouse_shelf</uri>
    <name>shelf_row2_1</name>
    <pose>6 0 0 0 0 0</pose>
  </include>

  <!-- Include models from Gazebo fuel -->
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
  </include>
</world>
```

---

## Complete Warehouse World Example {#complete-example}

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="warehouse_simulation">

    <!-- Physics configuration -->
    <physics type="ode">
      <real_time_factor>1.0</real_time_factor>
      <max_step_size>0.001</max_step_size>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
        </solver>
      </ode>
    </physics>

    <!-- Warehouse lighting -->
    <light type="directional" name="overhead_1">
      <pose>10 10 8 0 0 0</pose>
      <diffuse>0.8 0.8 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0 0 -1</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <light type="point" name="area_light_1">
      <pose>5 5 4 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <attenuation>
        <range>12</range>
        <constant>0.5</constant>
        <linear>0.01</linear>
      </attenuation>
    </light>

    <!-- Concrete floor -->
    <model name="floor">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.9</mu>
                <mu2>0.9</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- Walls -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 10 1.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>20 0.2 3</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>20 0.2 3</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Shelf row 1 -->
    <model name="shelf_1">
      <static>true</static>
      <pose>-5 2 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>4 0.8 2.4</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
            <diffuse>0.4 0.3 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>4 0.8 2.4</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="shelf_2">
      <static>true</static>
      <pose>-5 6 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>4 0.8 2.4</size></box>
          </geometry>
          <material>
            <ambient>0.4 0.3 0.2 1</ambient>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>4 0.8 2.4</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Pallets with boxes -->
    <model name="pallet_1">
      <static>true</static>
      <pose>3 4 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>1.2 1.0 0.2</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.5 0.3 1</ambient>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>1.2 1.0 0.2</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="box_on_pallet">
      <static>false</static>  <!-- Movable! -->
      <pose>3 4 0.45 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>0.4 0.4 0.5</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.5 0.2 1</ambient>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>0.4 0.4 0.5</size></box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.1</iyy>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

---

## Summary

This chapter covered creating Gazebo worlds with SDF:

1. **SDF structure** defines worlds hierarchically: world → models → links → visual/collision.

2. **Ground planes** need proper friction configuration matching your deployment surface.

3. **Static obstacles** use `<static>true</static>` for objects that don't move.

4. **Lighting** affects camera simulation—configure for your target environment.

5. **Mesh imports** enable realistic visuals; use simplified collision geometry for performance.

6. **Model composition** enables reusable, organized world definitions.

---

## Self-Assessment Questions

1. **Friction Effects**: Your robot slips when accelerating in simulation but not in the real warehouse. The real floor is polished concrete. What SDF change would make simulation more realistic?

2. **Collision Geometry**: You import a detailed mesh of a wire rack shelf (50,000 triangles). Simulation runs at RTF=0.3. How would you improve performance while maintaining physical accuracy?

3. **Lighting Configuration**: Your camera-based object detection works in simulation but fails in the real dimly-lit warehouse. What lighting changes would create a more representative test?

4. **Coordinate Frames**: You want to place a shelf at position (5, 3) meters, rotated 45 degrees around the vertical axis. Write the `<pose>` element.

5. **Model Composition**: You need 20 identical shelves in a grid pattern. What approach would you use to avoid duplicating SDF code?

---

## What's Next

In [Chapter 3: Simulated Sensors](/module-3/chapter-3-simulated-sensors), you'll configure camera, LiDAR, and IMU sensors with noise models that match real hardware specifications.

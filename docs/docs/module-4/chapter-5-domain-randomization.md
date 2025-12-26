---
id: chapter-5-domain-randomization
title: "Chapter 5: Domain Randomization"
sidebar_label: "5. Domain Randomization"
sidebar_position: 6
---

# Chapter 5: Domain Randomization and Synthetic Data

## Chapter Goal

By the end of this chapter, you will be able to **master domain randomization using Isaac Replicator** to generate robust perception systems that transfer from simulation to reality.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 5.1 | Explain why domain randomization improves sim-to-real transfer |
| 5.2 | Implement texture and material randomization |
| 5.3 | Implement lighting randomization |
| 5.4 | Implement physics randomization |
| 5.5 | Implement sensor noise randomization |
| 5.6 | Generate synthetic datasets with ground truth |

---

## Why Domain Randomization?

### The Domain Gap Problem

From Module 3, Chapter 6:

```
Training Distribution:        Real World Distribution:
┌─────────────────────┐       ┌─────────────────────┐
│    Simulation       │       │     Reality         │
│    (narrow)         │       │     (broad)         │
│                     │       │                     │
│    ●                │       │         ●●●●●       │
│  (one texture)      │       │  (many variations)  │
└─────────────────────┘       └─────────────────────┘
              │                         │
              └─────── GAP ─────────────┘
```

Models trained on narrow simulation fail when reality differs.

### The Randomization Solution

```
Randomized Training:          Real World:
┌─────────────────────┐       ┌─────────────────────┐
│    Many variations  │       │     Reality         │
│    ●●●●●●●●●●       │       │     (subset)        │
│                     │       │                     │
│    ●●●●●●●●●●       │       │         ●●●●●       │
│    ●●●●●●●●●●       │       │                     │
└─────────────────────┘       └─────────────────────┘
              │                         │
              └───── COVERED ───────────┘
```

If training covers enough variation, real-world becomes just another sample.

---

## Isaac Replicator Architecture {#replicator}

Isaac Replicator is the domain randomization tool in Isaac Sim:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac Replicator                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Randomizers                    Writers                          │
│  ┌──────────────────┐          ┌──────────────────┐             │
│  │ Texture          │          │ RGB Image        │             │
│  │ Material         │          │ Depth Image      │             │
│  │ Lighting         │          │ Segmentation     │             │
│  │ Position         │  ──────► │ Bounding Boxes   │             │
│  │ Physics          │          │ Pose Annotations │             │
│  │ Sensor Params    │          │ Point Cloud      │             │
│  └──────────────────┘          └──────────────────┘             │
│           │                             │                        │
│           ▼                             ▼                        │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │               Synthetic Dataset                           │   │
│  │  (Images + Annotations for Training)                      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Basic Replicator Script

```python
import omni.replicator.core as rep

# Setup scene
rep.set_global_seed(42)

# Define what to randomize
with rep.new_layer():
    # Get objects to randomize
    boxes = rep.get.prims(path_pattern="/World/Box_*")
    floor = rep.get.prim(path="/World/Floor")

    # Define randomization
    with rep.trigger.on_frame():
        with boxes:
            rep.modify.pose(
                position=rep.distribution.uniform((-2, -2, 0), (2, 2, 0)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
            )
        with floor:
            rep.modify.attribute(
                name="material:diffuse_color",
                value=rep.distribution.uniform((0.2, 0.2, 0.2), (0.8, 0.8, 0.8))
            )
```

---

## Visual Randomization {#visual}

### Texture Randomization

```python
import omni.replicator.core as rep

# Create texture randomizer
textures = [
    "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Wood/Wood_Plank.mdl",
    "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Concrete/Concrete.mdl",
    "omniverse://localhost/NVIDIA/Materials/vMaterials_2/Metal/Brushed_Metal.mdl",
]

with rep.trigger.on_frame():
    floor = rep.get.prim(path="/World/Floor")
    with floor:
        rep.randomizer.texture(textures)
```

### Material Property Randomization

```python
with rep.trigger.on_frame():
    objects = rep.get.prims(semantics=[("class", "target_object")])
    with objects:
        rep.modify.attribute(
            # Albedo (base color)
            name="material:albedo_color",
            value=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9))
        )
        rep.modify.attribute(
            # Roughness affects reflections
            name="material:roughness",
            value=rep.distribution.uniform(0.3, 0.9)
        )
        rep.modify.attribute(
            # Metallic appearance
            name="material:metallic",
            value=rep.distribution.uniform(0.0, 0.3)
        )
```

### Physical Grounding

Material properties must stay physically plausible:

| Property | Physical Range | Why |
|----------|----------------|-----|
| Roughness | 0.3-0.9 | < 0.3 unrealistically smooth |
| Metallic | 0.0-0.3 | Non-metallic objects |
| Albedo | 0.1-0.9 | Physically valid reflectance |

---

## Lighting Randomization {#lighting}

### Light Intensity and Color

```python
with rep.trigger.on_frame():
    lights = rep.get.light()
    with lights:
        rep.modify.attribute(
            name="intensity",
            value=rep.distribution.uniform(500, 2000)
        )
        rep.modify.attribute(
            name="color",
            value=rep.distribution.uniform((0.8, 0.8, 0.9), (1.0, 1.0, 1.0))
        )
```

### Time of Day Simulation

```python
# Simulate different times of day
def create_lighting_presets():
    presets = {
        "morning": {"intensity": 800, "color": (1.0, 0.95, 0.8), "angle": 30},
        "noon": {"intensity": 2000, "color": (1.0, 1.0, 1.0), "angle": 80},
        "afternoon": {"intensity": 1200, "color": (1.0, 0.9, 0.7), "angle": 50},
        "evening": {"intensity": 600, "color": (1.0, 0.8, 0.6), "angle": 20},
    }
    return presets

with rep.trigger.on_frame():
    sun = rep.get.light(path="/World/Sun")
    preset = rep.distribution.choice(list(create_lighting_presets().values()))
    with sun:
        rep.modify.attribute(name="intensity", value=preset["intensity"])
        rep.modify.attribute(name="color", value=preset["color"])
```

### Additional Light Sources

```python
# Randomize number and position of point lights
with rep.trigger.on_frame():
    # Remove existing random lights
    rep.delete.prims(path_pattern="/World/RandomLight_*")

    # Create 1-5 random lights
    num_lights = rep.distribution.uniform(1, 5)
    for i in range(int(num_lights)):
        rep.create.light(
            light_type="sphere",
            position=rep.distribution.uniform((-5, -5, 2), (5, 5, 4)),
            intensity=rep.distribution.uniform(100, 500),
            color=rep.distribution.uniform((0.9, 0.9, 0.9), (1.0, 1.0, 1.0))
        )
```

---

## Physics Randomization {#physics}

### Friction Randomization

```python
with rep.trigger.on_frame():
    floor = rep.get.prim(path="/World/Floor")
    with floor:
        rep.physics.attribute(
            name="physics:staticFriction",
            value=rep.distribution.uniform(0.3, 1.0)
        )
        rep.physics.attribute(
            name="physics:dynamicFriction",
            value=rep.distribution.uniform(0.3, 0.9)
        )
```

### Mass and Inertia Randomization

```python
with rep.trigger.on_frame():
    objects = rep.get.prims(semantics=[("class", "manipulatable")])
    with objects:
        # ±20% mass variation (manufacturing tolerance)
        base_mass = 1.0  # kg
        rep.physics.attribute(
            name="physics:mass",
            value=rep.distribution.uniform(base_mass * 0.8, base_mass * 1.2)
        )
```

### Joint Parameter Randomization

```python
with rep.trigger.on_frame():
    joints = rep.get.prims(path_pattern="/World/Robot/*/joint_*")
    with joints:
        # Damping variation
        rep.physics.attribute(
            name="physics:damping",
            value=rep.distribution.uniform(0.1, 0.5)
        )
        # Friction variation
        rep.physics.attribute(
            name="physics:jointFriction",
            value=rep.distribution.uniform(0.01, 0.1)
        )
```

**Physical Grounding**: Ranges should match physical possibilities:

| Parameter | Range | Physical Basis |
|-----------|-------|----------------|
| Floor friction | 0.3-1.0 | Tile to rubber |
| Mass | ±20% | Manufacturing tolerance |
| Joint damping | 0.1-0.5 | Lubrication variation |

---

## Sensor Randomization {#sensor}

### Camera Parameter Randomization

```python
with rep.trigger.on_frame():
    camera = rep.get.camera(path="/World/Robot/Camera")
    with camera:
        # Exposure variation
        rep.modify.attribute(
            name="camera:exposure",
            value=rep.distribution.uniform(-1.0, 1.0)
        )
        # Focal length variation (zoom)
        rep.modify.attribute(
            name="camera:focalLength",
            value=rep.distribution.uniform(1.8, 2.1)  # mm
        )
```

### Sensor Noise Randomization

```python
with rep.trigger.on_frame():
    camera = rep.get.camera(path="/World/Robot/Camera")
    with camera:
        # Noise level variation
        rep.modify.attribute(
            name="camera:noiseStdDev",
            value=rep.distribution.uniform(0.005, 0.02)
        )

    lidar = rep.get.sensor(path="/World/Robot/LiDAR")
    with lidar:
        # LiDAR range noise variation
        rep.modify.attribute(
            name="lidar:rangeNoiseStdDev",
            value=rep.distribution.uniform(0.01, 0.05)
        )
```

---

## Synthetic Dataset Generation {#dataset}

### Writer Configuration

```python
import omni.replicator.core as rep

# Configure output
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="./synthetic_dataset",
    rgb=True,
    depth=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    bounding_box_2d_tight=True,
)

# Attach to render products
render_product = rep.create.render_product(
    camera="/World/Robot/Camera",
    resolution=(640, 480)
)
writer.attach([render_product])
```

### Dataset Generation Loop

```python
import omni.replicator.core as rep

# Number of frames to generate
NUM_FRAMES = 10000

# Setup randomization
with rep.new_layer():
    with rep.trigger.on_frame(num_frames=NUM_FRAMES):
        # All randomizations here
        boxes = rep.get.prims(path_pattern="/World/Box_*")
        with boxes:
            rep.modify.pose(
                position=rep.distribution.uniform((-2, -2, 0), (2, 2, 0))
            )
            rep.randomizer.texture(textures)

        # Lighting
        lights = rep.get.light()
        with lights:
            rep.modify.attribute(
                name="intensity",
                value=rep.distribution.uniform(500, 2000)
            )

# Run generation
rep.orchestrator.run()
```

### Output Format

```
synthetic_dataset/
├── rgb/
│   ├── rgb_000000.png
│   ├── rgb_000001.png
│   └── ...
├── depth/
│   ├── depth_000000.npy
│   └── ...
├── semantic_segmentation/
│   ├── semantic_000000.png
│   └── ...
├── instance_segmentation/
│   ├── instance_000000.png
│   └── ...
└── bounding_box_2d_tight/
    ├── bbox_000000.json
    └── ...
```

### Annotation Format

```json
// bbox_000000.json
{
  "annotations": [
    {
      "class": "box",
      "instance_id": 1,
      "bbox": [120, 80, 200, 160],  // x, y, width, height
      "confidence": 1.0
    },
    {
      "class": "shelf",
      "instance_id": 2,
      "bbox": [300, 50, 150, 400],
      "confidence": 1.0
    }
  ]
}
```

---

## When Randomization Helps (and Doesn't) {#limitations}

### Effective Against

| Gap Type | Randomization | Effectiveness |
|----------|---------------|---------------|
| Visual appearance | Texture, lighting | High |
| Object positions | Pose randomization | High |
| Sensor noise levels | Noise parameter | High |
| Friction variation | Physics params | Moderate |

### Not Effective Against

| Gap Type | Why | Alternative |
|----------|-----|-------------|
| Unmodeled physics | Can't randomize what's not modeled | System ID |
| Systematic sensor bias | Randomization is around wrong mean | Calibration |
| Missing phenomena | Not in simulation at all | Better simulation |
| Fundamental domain shift | Too different to cover | Domain adaptation |

---

## Summary

This chapter covered domain randomization with Isaac Replicator:

1. **Domain randomization** trains models on varied conditions so real world is "just another sample."

2. **Visual randomization** (textures, materials, lighting) addresses perception gap.

3. **Physics randomization** (friction, mass, damping) addresses physics gap.

4. **Sensor randomization** (noise, parameters) improves robustness to sensor variation.

5. **Synthetic datasets** with ground truth annotations enable training perception models.

6. **Limitations** exist—randomization can't fix unmodeled phenomena.

---

## Self-Assessment Questions

1. **Randomization Design**: You're training an object detector for cardboard boxes. What visual properties would you randomize? What ranges?

2. **Physics Ranges**: Why randomize friction from 0.3-1.0 rather than 0-1.0? What physical principle constrains this?

3. **Sensor Noise**: Your real camera has different noise at different ISO settings. How would you represent this in randomization?

4. **Dataset Size**: How many synthetic images would you generate for training? What factors influence this decision?

5. **Limitation Analysis**: Your trained model works on all synthetic variations but fails on real images with motion blur. Why didn't randomization help?

---

## What's Next

In [Chapter 6: Sim-to-Real Transfer](/module-4/chapter-6-sim-to-real-transfer), you'll synthesize all Module 4 capabilities into comprehensive transfer strategies, preparing for capstone integration and hardware deployment.

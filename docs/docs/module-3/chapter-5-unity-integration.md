---
id: chapter-5-unity-integration
title: "Chapter 5: Unity Integration"
sidebar_label: "5. Unity Integration"
sidebar_position: 6
---

# Chapter 5: Unity and ROS 2 Integration

## Chapter Goal

By the end of this chapter, you will be able to **use Unity as an alternative simulation environment with ROS 2 integration**, understanding its strengths for visualization and when to choose it over Gazebo.

## Learning Objectives

After completing this chapter, you will be able to:

| ID | Objective |
|----|-----------|
| 5.1 | Install and configure Unity with Robotics packages |
| 5.2 | Import a URDF robot into Unity and configure articulation physics |
| 5.3 | Configure Unity cameras publishing to ROS 2 Image topics |
| 5.4 | Configure Unity sensor simulation with ROS 2 publishing |
| 5.5 | Implement bidirectional control: ROS 2 commands driving Unity robots |
| 5.6 | Articulate when to choose Unity vs Gazebo for simulation |

---

## Why Unity for Robotics?

Unity offers capabilities that Gazebo doesn't:

| Capability | Unity | Gazebo |
|------------|-------|--------|
| Visual fidelity | Photorealistic (HDRP) | Functional |
| Asset ecosystem | Extensive marketplace | Limited |
| Ray tracing | RTX support | Not available |
| VR/AR integration | Native | Limited |
| Cross-platform | Yes | Linux-focused |

**Use Unity when**: Perception training data, demonstrations, visualization-intensive scenarios.

**Use Gazebo when**: Physics accuracy, control algorithm validation, headless batch testing.

---

## Unity Robotics Hub {#unity-robotics}

Unity's robotics tools consist of several packages:

```
Unity Robotics
├── URDF Importer          → Import robot descriptions
├── ROS-TCP-Connector      → ROS 2 communication bridge
├── Perception Package     → Synthetic data generation
└── Articulation Bodies    → Robot physics simulation
```

### Installation

1. **Install Unity Hub** from unity.com
2. **Install Unity Editor** (2021.3 LTS or newer)
3. **Create new 3D project**
4. **Add packages via Package Manager**:

```
Window → Package Manager → + → Add package from git URL

https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

### Project Configuration

Enable the new input system and linear color space:

```
Edit → Project Settings → Player
  - Other Settings → Color Space: Linear
  - Other Settings → Active Input Handling: Both
```

---

## URDF Import {#urdf-import}

### Import Process

1. **Place URDF files** in `Assets/URDFs/` folder
2. **Include meshes** in relative paths
3. **Right-click URDF** → Import Robot from URDF

### Import Settings

```
URDF Import Settings:
├── Axis Type: Z-Up (match ROS convention)
├── Mesh Decomposer: VHACD (for concave meshes)
├── Create Collision: True
├── Use Inertia from URDF: True
└── Rigidbody → Articulation Body: True
```

### Articulation Bodies

Unity uses **Articulation Bodies** for robots—more stable than regular rigidbodies for kinematic chains.

```csharp
// Articulation body properties
ArticulationBody body = GetComponent<ArticulationBody>();

// Set joint type
body.jointType = ArticulationJointType.RevoluteJoint;

// Set limits
ArticulationDrive drive = body.xDrive;
drive.lowerLimit = -90f;  // degrees
drive.upperLimit = 90f;
drive.stiffness = 10000f;
drive.damping = 100f;
body.xDrive = drive;
```

---

## ROS-TCP-Connector {#ros-tcp}

### Architecture

```
┌─────────────────┐         TCP/IP          ┌─────────────────┐
│                 │ ◄─────────────────────► │                 │
│  Unity Editor   │      Port 10000         │  ROS 2 System   │
│                 │                         │                 │
│  - Publishers   │                         │  - Subscribers  │
│  - Subscribers  │                         │  - Publishers   │
│  - Services     │                         │  - Services     │
└─────────────────┘                         └─────────────────┘
        │                                           │
        ▼                                           ▼
   ROSConnection                            ros_tcp_endpoint
   (Unity C#)                               (ROS 2 node)
```

### ROS 2 Endpoint Setup

```bash
# Install ros_tcp_endpoint
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git -b ROS2v0.7.0
cd ..
colcon build

# Run the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

### Unity Configuration

In Unity, add ROS Connection:

```
Robotics → ROS Settings
  - ROS IP Address: 127.0.0.1
  - ROS Port: 10000
  - Connect on Start: True
  - Protocol: ROS2
```

### Testing Connection

```bash
# Terminal 1: Run ROS endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: Check topics
ros2 topic list

# In Unity: Press Play
# Should see connection established
```

---

## Camera Simulation {#camera}

### Camera Publisher Script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class CameraPublisher : MonoBehaviour
{
    public Camera robotCamera;
    public string topicName = "/camera/image_raw";
    public float publishRate = 30f;

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Create render texture matching camera resolution
        renderTexture = new RenderTexture(640, 480, 24);
        robotCamera.targetTexture = renderTexture;

        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (Time.time - lastPublishTime < 1f / publishRate)
            return;

        lastPublishTime = Time.time;
        PublishImage();
    }

    void PublishImage()
    {
        // Capture camera view
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        // Create ROS message
        byte[] imageData = texture2D.GetRawTextureData();

        ImageMsg msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "camera_link"
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            is_bigendian = 0,
            step = 640 * 3,
            data = imageData
        };

        ros.Publish(topicName, msg);
    }
}
```

### High-Quality Rendering (HDRP)

For photorealistic images:

1. **Install HDRP** via Package Manager
2. **Convert project** to HDRP
3. **Configure camera** for HDRP settings

```csharp
// HDRP camera settings for realistic rendering
HDAdditionalCameraData hdCamera = camera.GetComponent<HDAdditionalCameraData>();
hdCamera.antialiasing = HDAdditionalCameraData.AntialiasingMode.TemporalAntialiasing;
```

---

## LiDAR Simulation {#lidar}

### Raycasting LiDAR

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class LidarPublisher : MonoBehaviour
{
    public string topicName = "/scan";
    public float publishRate = 10f;

    [Header("LiDAR Parameters")]
    public float minAngle = -3.14159f;
    public float maxAngle = 3.14159f;
    public int numRays = 360;
    public float minRange = 0.1f;
    public float maxRange = 30f;

    private ROSConnection ros;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
    }

    void Update()
    {
        if (Time.time - lastPublishTime < 1f / publishRate)
            return;

        lastPublishTime = Time.time;
        PublishScan();
    }

    void PublishScan()
    {
        float[] ranges = new float[numRays];
        float angleIncrement = (maxAngle - minAngle) / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = minAngle + i * angleIncrement;
            Vector3 direction = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0) * transform.forward;

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxRange;  // No hit
            }
        }

        LaserScanMsg msg = new LaserScanMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "lidar_link"
            },
            angle_min = minAngle,
            angle_max = maxAngle,
            angle_increment = angleIncrement,
            time_increment = 0f,
            scan_time = 1f / publishRate,
            range_min = minRange,
            range_max = maxRange,
            ranges = ranges,
            intensities = new float[0]
        };

        ros.Publish(topicName, msg);
    }
}
```

---

## Robot Control {#control}

### Velocity Command Subscriber

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class DiffDriveController : MonoBehaviour
{
    public string cmdVelTopic = "/cmd_vel";
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    public float wheelRadius = 0.1f;
    public float wheelSeparation = 0.5f;

    private Vector3 targetVelocity = Vector3.zero;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(
            cmdVelTopic, OnCmdVelReceived);
    }

    void OnCmdVelReceived(TwistMsg msg)
    {
        // Convert Twist to wheel velocities
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;

        float leftVel = (linear - angular * wheelSeparation / 2) / wheelRadius;
        float rightVel = (linear + angular * wheelSeparation / 2) / wheelRadius;

        // Apply to articulation bodies
        SetWheelVelocity(leftWheel, leftVel);
        SetWheelVelocity(rightWheel, rightVel);
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = velocity * Mathf.Rad2Deg;  // Unity uses degrees
        wheel.xDrive = drive;
    }
}
```

### Odometry Publisher

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public class OdometryPublisher : MonoBehaviour
{
    public string topicName = "/odom";
    public float publishRate = 50f;

    private ROSConnection ros;
    private Vector3 lastPosition;
    private Quaternion lastRotation;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);

        lastPosition = transform.position;
        lastRotation = transform.rotation;
    }

    void Update()
    {
        if (Time.time - lastPublishTime < 1f / publishRate)
            return;

        lastPublishTime = Time.time;
        PublishOdometry();
    }

    void PublishOdometry()
    {
        OdometryMsg msg = new OdometryMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "odom"
            },
            child_frame_id = "base_link",
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg
                    {
                        x = transform.position.z,  // ROS X = Unity Z
                        y = -transform.position.x, // ROS Y = -Unity X
                        z = transform.position.y   // ROS Z = Unity Y
                    },
                    orientation = new QuaternionMsg
                    {
                        // Convert Unity to ROS quaternion
                        x = transform.rotation.z,
                        y = -transform.rotation.x,
                        z = transform.rotation.y,
                        w = -transform.rotation.w
                    }
                }
            }
        };

        ros.Publish(topicName, msg);
    }
}
```

---

## Decision Framework: Unity vs Gazebo {#decision}

### When to Use Unity

| Scenario | Why Unity |
|----------|-----------|
| Training perception models | Photorealistic images, domain randomization |
| Customer demonstrations | High visual quality |
| VR/AR applications | Native support |
| Complex visual environments | Asset store availability |
| Cross-platform deployment | Windows/Mac/Linux/Web |

### When to Use Gazebo

| Scenario | Why Gazebo |
|----------|-----------|
| Control algorithm development | Better physics accuracy |
| Contact-rich manipulation | More accurate contact dynamics |
| Headless batch testing | Native headless mode |
| ROS 2 integration | Tighter integration, less latency |
| Open-source requirement | Fully open source |

### Decision Flowchart

```
Primary goal?
├── Perception/Vision → Unity (better rendering)
├── Control/Dynamics → Gazebo (better physics)
├── Navigation → Either (similar capability)
└── Manipulation → Gazebo (contact physics)

Need photorealistic images?
├── Yes → Unity
└── No → Gazebo (faster)

Running batch tests?
├── Yes → Gazebo (headless support)
└── No → Either

Team expertise?
├── Game development → Unity
├── ROS ecosystem → Gazebo
└── Both → Choose by task
```

---

## Physical Grounding Considerations {#physical}

### Timing Differences

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| ROS bridge | ros_gz (shared memory) | TCP (network) |
| Typical latency | 1-5 ms | 5-20 ms |
| Determinism | High | Moderate |

The TCP bridge adds latency that doesn't exist in ros_gz. Account for this in timing-sensitive applications.

### Physics Differences

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| Physics engine | ODE/Bullet/DART | PhysX |
| Contact accuracy | Higher | Moderate |
| Articulation | ros2_control | ArticulationBody |
| Friction model | Configurable | PhysX default |

Control gains tuned in Unity may not transfer directly to Gazebo or real hardware.

---

## Summary

This chapter covered Unity as an alternative simulation platform:

1. **Unity Robotics Hub** provides URDF import and ROS integration.

2. **ROS-TCP-Connector** bridges Unity and ROS 2 over TCP.

3. **Camera simulation** leverages Unity's rendering for photorealistic images.

4. **LiDAR simulation** uses raycasting similar to Gazebo.

5. **Robot control** receives commands from ROS 2 and publishes odometry.

6. **Decision framework**: Use Unity for visual fidelity, Gazebo for physics accuracy.

---

## Self-Assessment Questions

1. **Platform Selection**: You need to train a neural network for object detection. Would you use Unity or Gazebo? Why?

2. **Latency Impact**: Your control loop runs at 100 Hz. The Unity TCP bridge adds 15ms latency. What is the maximum achievable control rate?

3. **URDF Import**: Your imported robot falls through the floor. What physics component should you check?

4. **Camera Publishing**: Your ROS 2 node receives camera images but they appear upside-down. Where is the coordinate transform issue?

5. **Use Case**: You need to demonstrate a robot to investors with high visual quality, but also need to test the navigation algorithm accurately. What would you recommend?

---

## What's Next

In [Chapter 6: Reality Gap](/module-3/chapter-6-reality-gap), you'll critically examine the differences between simulation and reality—understanding what transfers and what doesn't, preparing you for Module 4's sim-to-real techniques.

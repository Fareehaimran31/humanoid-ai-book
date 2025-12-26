---
sidebar_position: 2
title: "Unity Simulation Alternative"
---

# Unity Simulation Alternative

## Concept Overview

Unity is a powerful 3D development platform that provides high-fidelity graphics, realistic physics, and extensive tooling for creating immersive simulation environments. While Gazebo is the primary simulation environment for this course, Unity offers an alternative with superior visual quality and advanced rendering capabilities. Unity is particularly useful for applications requiring photorealistic rendering, complex lighting scenarios, or advanced visual effects.

## Architecture Diagram: Unity Integration

```
┌─────────────────────────────────────────────────────────────────┐
│                  Unity ROS Integration                          │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ROS# or    ┌─────────────────┐         │
│  │   ROS 2 Nodes   │◄──RosBridge──►│   Unity         │         │
│  │                 │              │   Simulator     │         │
│  │  ┌───────────┐  │              │                 │         │
│  │  │ Publisher │  │              │  ┌───────────┐  │         │
│  │  │   Node    │  │              │  │ Physics   │  │         │
│  │  └───────────┘  │              │  │ Engine    │  │         │
│  │  ┌───────────┐  │              │  └───────────┘  │         │
│  │  │ Subscriber│  │              │  ┌───────────┐  │         │
│  │  │   Node    │  │              │  │ Rendering │  │         │
│  │  └───────────┘  │              │  │ Engine    │  │         │
│  └─────────────────┘              │  └───────────┘  │         │
│                                   │  ┌───────────┐  │         │
│                                   │  │ AI/ML     │  │         │
│                                   │  │ Tools     │  │         │
│                                   │  └───────────┘  │         │
│                                   └─────────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

## Unity ROS Integration Setup

### Option 1: Unity Robotics Hub

Unity Robotics Hub provides official tools for connecting Unity to ROS 2:

1. Install Unity (version 2021.3 LTS or later recommended)
2. Install Unity Robotics Hub from the Unity Asset Store or GitHub
3. Import the ROS-TCP-Connector and ROS-TCP-Endpoint packages

### Option 2: ROS# (Alternative)

ROS# is a community-developed solution for Unity-ROS integration:

1. Install Unity
2. Import the ROS# package into your Unity project
3. Configure the ROS communication settings

## Basic Unity Scene Setup for Robotics

Here's a basic Unity C# script to interface with ROS 2:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string robotName = "simple_humanoid";

    // Robot components
    public GameObject head;
    public GameObject leftArm;
    public GameObject rightArm;

    // Sensor publishers
    private float publishRate = 10f; // 10 Hz
    private float lastPublishTime = 0f;

    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>($"joint_states");

        // Subscribe to command topics
        ros.Subscribe<TwistMsg>($"{robotName}/cmd_vel", CmdVelCallback);
    }

    void Update()
    {
        // Publish sensor data at specified rate
        if (Time.time - lastPublishTime > 1f / publishRate)
        {
            PublishJointStates();
            lastPublishTime = Time.time;
        }
    }

    void CmdVelCallback(TwistMsg cmd_vel)
    {
        // Process velocity commands
        float linear_x = (float)cmd_vel.linear.x;
        float angular_z = (float)cmd_vel.angular.z;

        // Move the robot based on the command
        transform.Translate(Vector3.forward * linear_x * Time.deltaTime);
        transform.Rotate(Vector3.up, angular_z * Time.deltaTime);
    }

    void PublishJointStates()
    {
        // Create and publish joint state message
        JointStateMsg jointState = new JointStateMsg();
        jointState.name = new string[] { "head_joint", "left_arm_joint", "right_arm_joint" };
        jointState.position = new double[] { head.transform.localEulerAngles.y,
                                           leftArm.transform.localEulerAngles.x,
                                           rightArm.transform.localEulerAngles.x };
        jointState.header.stamp = new TimeStamp(ros.Clock.time);
        jointState.header.frame_id = robotName;

        // Publish the message
        ros.Publish<JointStateMsg>($"joint_states", jointState);
    }
}
```

## Unity Sensor Simulation

### Camera Sensor Simulation

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine;

public class UnityCameraSensor : MonoBehaviour
{
    public Camera cameraComponent;
    public string topicName = "camera/image_raw";
    public int width = 640;
    public int height = 480;
    public int publishRate = 30; // Hz

    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private ROSConnection ros;
    private float lastPublishTime = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Create render texture
        renderTexture = new RenderTexture(width, height, 24);
        cameraComponent.targetTexture = renderTexture;

        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (Time.time - lastPublishTime > 1f / publishRate)
        {
            PublishImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishImage()
    {
        // Copy the render texture to a readable texture
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();

        // Convert to ROS image message
        ImageMsg imageMsg = new ImageMsg();
        imageMsg.header.stamp = new TimeStamp(ros.Clock.time);
        imageMsg.header.frame_id = transform.name;
        imageMsg.height = (uint)height;
        imageMsg.width = (uint)width;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(width * 3); // 3 bytes per pixel (RGB)

        // Convert texture to bytes
        byte[] imageData = texture2D.EncodeToPNG();
        imageMsg.data = imageData;

        // Publish the image
        ros.Publish<ImageMsg>(topicName, imageMsg);
    }
}
```

### LiDAR Sensor Simulation

Unity doesn't have built-in LiDAR simulation, but we can simulate it using raycasting:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine;

public class UnityLidarSensor : MonoBehaviour
{
    public string topicName = "scan";
    public int rayCount = 360;
    public float angleMin = -Mathf.PI;
    public float angleMax = Mathf.PI;
    public float rangeMin = 0.1f;
    public float rangeMax = 30.0f;
    public float publishRate = 10f; // Hz

    private ROSConnection ros;
    private float lastPublishTime = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    void Update()
    {
        if (Time.time - lastPublishTime > 1f / publishRate)
        {
            PublishLidarScan();
            lastPublishTime = Time.time;
        }
    }

    void PublishLidarScan()
    {
        LaserScanMsg scanMsg = new LaserScanMsg();
        scanMsg.header.stamp = new TimeStamp(ros.Clock.time);
        scanMsg.header.frame_id = transform.name;
        scanMsg.angle_min = angleMin;
        scanMsg.angle_max = angleMax;
        scanMsg.angle_increment = (angleMax - angleMin) / rayCount;
        scanMsg.time_increment = 0f;
        scanMsg.scan_time = 1f / publishRate;
        scanMsg.range_min = rangeMin;
        scanMsg.range_max = rangeMax;

        // Perform raycasts
        float[] ranges = new float[rayCount];
        for (int i = 0; i < rayCount; i++)
        {
            float angle = angleMin + i * scanMsg.angle_increment;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            direction = transform.TransformDirection(direction);

            if (Physics.Raycast(transform.position, direction, out RaycastHit hit, rangeMax))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = float.PositiveInfinity; // or rangeMax
            }
        }

        scanMsg.ranges = ranges;

        // Publish the scan
        ros.Publish<LaserScanMsg>(topicName, scanMsg);
    }
}
```

## Unity Package Manager Setup

To set up a Unity project for robotics simulation:

1. Create a new 3D project in Unity Hub
2. Add the following packages via Window > Package Manager:
   - Unity Robotics Hub (includes ROS-TCP-Connector)
   - ProBuilder (for quick environment creation)
   - Timeline (for animation sequences)

3. Create a basic scene hierarchy:
   ```
   Scene
   ├── Robot (Prefab)
   │   ├── Torso
   │   ├── Head
   │   ├── Left Arm
   │   └── Right Arm
   ├── Environment
   │   ├── Ground
   │   ├── Walls
   │   └── Obstacles
   └── ROSConnectionManager (Empty GameObject)
   ```

## Unity Environment Creation

### Creating a Basic Environment

1. Create a ground plane: GameObject > 3D Object > Plane
2. Scale it to desired size (e.g., 20x20 units)
3. Add a material with appropriate texture
4. Create obstacles using basic shapes (cubes, spheres, etc.)

### Adding Physics

Unity uses PhysX for physics simulation. To make objects interact properly:

1. Add Rigidbody components to objects that should move physically
2. Add Collider components to all objects for collision detection
3. Configure Physic Materials for realistic friction and bounce

## Unity-ROS Communication Patterns

### Publisher Example

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine;

public class UnityPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_robot_pose";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }

    void Update()
    {
        // Publish robot pose every frame or at specific intervals
        PoseMsg pose = new PoseMsg();
        pose.position.x = transform.position.x;
        pose.position.y = transform.position.y;
        pose.position.z = transform.position.z;

        // Convert Unity quaternion to ROS quaternion
        pose.orientation.x = transform.rotation.x;
        pose.orientation.y = transform.rotation.y;
        pose.orientation.z = transform.rotation.z;
        pose.orientation.w = transform.rotation.w;

        ros.Publish<PoseMsg>(topicName, pose);
    }
}
```

### Subscriber Example

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine;

public class UnitySubscriber : MonoBehaviour
{
    public string topicName = "move_base_simple/goal";

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PoseStampedMsg>(topicName, GoalCallback);
    }

    void GoalCallback(PoseStampedMsg goal)
    {
        // Move this object to the received goal position
        transform.position = new Vector3((float)goal.pose.position.x,
                                        (float)goal.pose.position.y,
                                        (float)goal.pose.position.z);
    }
}
```

## Lab Instructions

### Exercise 1: Unity Robot Setup

1. Create a new Unity 3D project
2. Import Unity Robotics Hub
3. Create a simple humanoid robot using basic primitives
4. Add the RobotController script to enable ROS communication
5. Test the connection with a simple ROS node

### Exercise 2: Unity Sensor Integration

1. Add camera and LiDAR simulation scripts to your Unity robot
2. Configure the sensors with appropriate parameters
3. Verify that sensor data is being published to ROS topics
4. Visualize the data in RViz2

### Exercise 3: Unity Environment Simulation

1. Create a complex environment with obstacles
2. Implement path planning visualization
3. Test robot navigation in the Unity environment
4. Compare simulation results with Gazebo

## Unity vs Gazebo Comparison

| Feature | Unity | Gazebo |
|---------|-------|--------|
| Graphics Quality | Excellent | Good |
| Physics Simulation | Good | Excellent |
| ROS Integration | Moderate | Excellent |
| Learning Curve | Steeper | Moderate |
| Performance | Good | Good |
| Community Support | Large (game dev) | Robotics-specific |
| Cost | Free for personal use | Free |
| Real-time Rendering | Excellent | Good |

## Learning Objectives

After completing this chapter, you should be able to:
- Set up Unity for robotics simulation with ROS integration
- Create basic robot models and environments in Unity
- Implement sensor simulation in Unity (camera, LiDAR)
- Establish communication between Unity and ROS 2
- Understand when to use Unity vs Gazebo for different applications
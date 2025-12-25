---
sidebar_position: 1
---

# Unity Setup and ROS 2 Integration

## Introduction to Unity for Robotics

Unity is a powerful game engine that has been adapted for robotics applications through the Unity Robotics packages. For digital twin applications, Unity provides high-quality visualization, VR/AR capabilities, and flexible user interfaces that complement Gazebo's physics simulation capabilities.

## Installing Unity

### System Requirements
- Windows 10/11, macOS 10.14+, or Ubuntu 18.04+
- 8GB+ RAM (16GB+ recommended)
- Graphics card supporting DirectX 10, OpenGL 3.3, or Metal
- 20GB+ free disk space

### Unity Hub Installation
1. Download Unity Hub from [Unity's website](https://unity.com/download)
2. Install Unity Hub (required for managing Unity versions)
3. Sign in with a Unity ID (free account)

### Unity Editor Installation
1. Open Unity Hub
2. Go to the "Installs" tab
3. Click "Add" and select a Unity version (2021.3 LTS or newer recommended)
4. Select the "Universal Render Pipeline" and "XR" modules during installation

## Installing Unity Robotics Packages

### Unity Robotics Hub
The Unity Robotics Hub provides essential tools for robotics simulation:

1. Open Unity Hub and create a new 3D project
2. Open the Package Manager (Window → Package Manager)
3. Click the "+" button and select "Add package from git URL..."
4. Add the following packages:
   - `com.unity.robotics.ros-tcp-connector` - ROS TCP Connector
   - `com.unity.robotics.urdf-importer` - URDF Importer
   - `com.unity.robotics.visualizations` - Visualization tools

### Alternative Installation via Git
You can also install packages directly from GitHub:
- ROS TCP Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector.git
- URDF Importer: https://github.com/Unity-Technologies/URDF-Importer.git

## Setting Up ROS 2 Connection

### ROS 2 Bridge Installation
Install the ROS 2 bridge tools:

```bash
# Install the ROS 2 Unity bridge
sudo apt install ros-humble-rosbridge-suite
```

### Unity ROS TCP Connector Setup

1. In Unity, create a new scene
2. Add the ROS TCP Connection prefab to your scene:
   - Go to GameObject → Unity Robotics → ROS TCP Connection
   - This adds the ROSConnection singleton to your scene

3. Configure the connection in the Inspector:
   - Set the ROS IP Address (typically "127.0.0.1" for local connection)
   - Set the ROS Port (typically 10000)

### Basic Connection Script

Create a simple script to test the connection:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class UnityROSTest : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        // Get the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisteredUri = "http://localhost:11311"; // Default ROS master URI

        // Test the connection
        ros.SendServiceMessage("/unity_test", new TestMessage(), TestCallback);
    }

    void TestCallback(TestMessage response)
    {
        Debug.Log("Received response from ROS: " + response.data);
    }
}

// Example message structure
[System.Serializable]
public class TestMessage
{
    public string data;
}
```

## URDF Importer Setup

### Importing Robot Models

The URDF Importer allows you to import robot models directly from URDF files:

1. Place your URDF files in the Assets folder
2. Select the URDF file in the Project window
3. In the Inspector, configure import settings:
   - Set the appropriate coordinate system (typically Z-up for Unity)
   - Configure joint limits and physical properties
   - Adjust visual and collision properties

### Customizing Imported Models

After importing, you can customize the robot model:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class RobotController : MonoBehaviour
{
    public float jointSpeed = 1.0f;
    private ArticulationBody[] joints;

    void Start()
    {
        // Find all articulation bodies (joints) in the robot
        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void Update()
    {
        // Example: Control joints with keyboard input
        if (Input.GetKey(KeyCode.UpArrow))
        {
            MoveJoint(0, jointSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            MoveJoint(0, -jointSpeed * Time.deltaTime);
        }
    }

    void MoveJoint(int jointIndex, float targetPosition)
    {
        if (jointIndex < joints.Length)
        {
            ArticulationDrive drive = joints[jointIndex].jointDrive;
            drive.target += targetPosition;
            joints[jointIndex].jointDrive = drive;
        }
    }
}
```

## Digital Twin Specific Configuration

### High-Quality Rendering Setup

For digital twin applications, configure Unity for high-quality visualization:

1. Install the Universal Render Pipeline (URP) or High Definition Render Pipeline (HDRP)
2. Configure lighting and materials for photorealistic rendering
3. Set up post-processing effects for enhanced visual quality

### Performance Optimization

Balance visual quality with performance for real-time digital twin operation:

- Use Level of Detail (LOD) groups for complex models
- Implement occlusion culling for large environments
- Optimize shader complexity based on target hardware
- Use efficient lighting solutions

## Unity-ROS 2 Communication Patterns

### Publisher-Subscriber Pattern

Implement ROS 2 communication patterns in Unity:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine;

public class UnityCameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Camera unityCamera;
    private int sequenceId = 0;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        InvokeRepeating("PublishCameraInfo", 0.0f, 1.0f); // Publish every second
    }

    void PublishCameraInfo()
    {
        var cameraInfo = new SensorMsgsCameraInfoMsg
        {
            header = new std_msgs.HeaderMsg
            {
                stamp = new builtin_interfaces.TimeMsg { sec = 0, nanosec = 0 },
                frame_id = "unity_camera_frame",
                seq = (uint)sequenceId++
            },
            width = (uint)unityCamera.pixelWidth,
            height = (uint)unityCamera.pixelHeight,
            // Add other camera parameters as needed
        };

        ros.Publish("unity_camera_info", cameraInfo);
    }
}
```

### Service Calls

Implement service calls for bidirectional communication:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine;

public class UnityServiceClient : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    public void CallService()
    {
        var request = new StdSrvsEmptySrvRequest();
        ros.CallService<StdSrvsEmptySrvResponse>("unity_service",
            ServiceCallback, request);
    }

    void ServiceCallback(StdSrvsEmptySrvResponse response)
    {
        Debug.Log("Service call completed");
    }
}
```

## Next Steps

With Unity properly set up and connected to ROS 2, you can now implement visualization techniques and sensor integration for your digital twin application. The following sections will cover advanced visualization and sensor data integration.
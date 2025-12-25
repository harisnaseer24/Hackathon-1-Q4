---
sidebar_position: 3
---

# Sensor Data Integration in Unity

## Overview of Sensor Integration

Integrating sensor data into Unity is crucial for creating an accurate digital twin that reflects the real-world state of the physical robot. This chapter covers how to process and visualize various sensor data streams within the Unity environment.

## ROS 2 Message Types for Sensors

### Standard Sensor Message Types

Unity can receive various ROS 2 sensor message types:

- `sensor_msgs/JointState` - Joint positions, velocities, efforts
- `sensor_msgs/Image` - Camera images
- `sensor_msgs/LaserScan` - 2D LIDAR data
- `sensor_msgs/PointCloud2` - 3D point cloud data
- `sensor_msgs/Imu` - Inertial measurement unit data
- `sensor_msgs/MagneticField` - Magnetic field readings
- `nav_msgs/Odometry` - Odometry information
- `geometry_msgs/TransformStamped` - Transform data

### Setting Up Message Subscriptions

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public class SensorDataManager : MonoBehaviour
{
    ROSConnection ros;

    [Header("Sensor Topics")]
    public string jointStatesTopic = "/joint_states";
    public string imuTopic = "/imu";
    public string odometryTopic = "/odom";
    public string laserScanTopic = "/scan";

    [Header("Sensor Data Storage")]
    private SensorMsgsJointStateMsg jointStateData;
    private SensorMsgsImuMsg imuData;
    private NavMsgsOdometryMsg odometryData;
    private SensorMsgsLaserScanMsg laserScanData;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to sensor topics
        ros.Subscribe<SensorMsgsJointStateMsg>(jointStatesTopic, JointStateCallback);
        ros.Subscribe<SensorMsgsImuMsg>(imuTopic, IMUCallback);
        ros.Subscribe<NavMsgsOdometryMsg>(odometryTopic, OdometryCallback);
        ros.Subscribe<SensorMsgsLaserScanMsg>(laserScanTopic, LaserScanCallback);
    }

    void JointStateCallback(SensorMsgsJointStateMsg msg)
    {
        jointStateData = msg;
        ProcessJointStateData();
    }

    void IMUCallback(SensorMsgsImuMsg msg)
    {
        imuData = msg;
        ProcessIMUData();
    }

    void OdometryCallback(NavMsgsOdometryMsg msg)
    {
        odometryData = msg;
        ProcessOdometryData();
    }

    void LaserScanCallback(SensorMsgsLaserScanMsg msg)
    {
        laserScanData = msg;
        ProcessLaserScanData();
    }

    void ProcessJointStateData()
    {
        // Process joint state data
        if (jointStateData != null)
        {
            // Update robot visualization based on joint positions
            // This would typically call a separate joint visualizer
        }
    }

    void ProcessIMUData()
    {
        // Process IMU data
        if (imuData != null)
        {
            // Update robot orientation, acceleration, etc.
        }
    }

    void ProcessOdometryData()
    {
        // Process odometry data
        if (odometryData != null)
        {
            // Update robot position in Unity world
        }
    }

    void ProcessLaserScanData()
    {
        // Process laser scan data
        if (laserScanData != null)
        {
            // Update LIDAR visualization
        }
    }
}
```

## Image Sensor Integration

### Camera Image Processing

Processing camera images from ROS 2 topics:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class CameraImageProcessor : MonoBehaviour
{
    ROSConnection ros;
    public string cameraTopic = "/camera/image_raw";
    public RawImage displayImage; // UI element to show camera feed

    private Texture2D cameraTexture;
    private byte[] imageData;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorMsgsImageMsg>(cameraTopic, ImageCallback);
    }

    void ImageCallback(SensorMsgsImageMsg msg)
    {
        // Process image data
        if (msg.data != null && msg.data.Array.Length > 0)
        {
            // Create or update texture based on image dimensions
            if (cameraTexture == null ||
                cameraTexture.width != msg.width ||
                cameraTexture.height != msg.height)
            {
                cameraTexture = new Texture2D((int)msg.width, (int)msg.height, TextureFormat.RGB24, false);
            }

            // Convert ROS image data to Unity texture format
            imageData = msg.data.Array;

            // Handle different encodings (typically "rgb8" or "bgr8")
            if (msg.encoding == "rgb8")
            {
                cameraTexture.LoadRawTextureData(imageData);
                cameraTexture.Apply();

                // Update display if using RawImage
                if (displayImage != null)
                {
                    displayImage.texture = cameraTexture;
                }
            }
        }
    }
}
```

### Point Cloud Integration

Processing 3D point cloud data:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class PointCloudProcessor : MonoBehaviour
{
    ROSConnection ros;
    public string pointCloudTopic = "/velodyne_points";
    public GameObject pointPrefab;
    public Transform pointCloudParent;

    private List<GameObject> pointObjects = new List<GameObject>();
    private int maxPoints = 10000;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorMsgsPointCloud2Msg>(pointCloudTopic, PointCloudCallback);
    }

    void PointCloudCallback(SensorMsgsPointCloud2Msg msg)
    {
        // Clear previous points
        foreach (var point in pointObjects)
        {
            if (point != null)
                DestroyImmediate(point);
        }
        pointObjects.Clear();

        // Process point cloud data
        if (msg.data != null && msg.data.Array.Length > 0)
        {
            // Parse point cloud data (simplified - actual parsing is more complex)
            // This assumes points are stored as x,y,z values in the data array
            int pointSize = 16; // Typical size for XYZ + padding
            int pointCount = msg.data.Array.Length / pointSize;

            for (int i = 0; i < Mathf.Min(pointCount, maxPoints); i++)
            {
                // Extract x, y, z coordinates (this is a simplified example)
                // Actual parsing depends on the point cloud format
                float x = System.BitConverter.ToSingle(msg.data.Array, i * pointSize);
                float y = System.BitConverter.ToSingle(msg.data.Array, i * pointSize + 4);
                float z = System.BitConverter.ToSingle(msg.data.Array, i * pointSize + 8);

                Vector3 pointPos = new Vector3(x, y, z);

                GameObject point = Instantiate(pointPrefab, pointPos, Quaternion.identity, pointCloudParent);
                pointObjects.Add(point);
            }
        }
    }
}
```

## Sensor Fusion in Unity

### Combining Multiple Sensor Data

Creating a fused sensor representation:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;
using System.Collections.Generic;

public class SensorFusionManager : MonoBehaviour
{
    ROSConnection ros;

    [Header("Sensor Topics")]
    public string imuTopic = "/imu";
    public string odometryTopic = "/odom";
    public string gpsTopic = "/gps/fix";
    public string laserScanTopic = "/scan";

    [Header("Fusion Settings")]
    public float fusionUpdateRate = 60f; // Hz
    public Transform robotTransform;

    private SensorMsgsImuMsg imuData;
    private NavMsgsOdometryMsg odometryData;
    private SensorMsgsNavSatFixMsg gpsData;
    private Vector3 fusedPosition;
    private Quaternion fusedOrientation;

    private float lastFusionUpdate = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<SensorMsgsImuMsg>(imuTopic, IMUCallback);
        ros.Subscribe<NavMsgsOdometryMsg>(odometryTopic, OdometryCallback);
        ros.Subscribe<SensorMsgsNavSatFixMsg>(gpsTopic, GPSCallback);

        InvokeRepeating("UpdateFusion", 0.0f, 1.0f / fusionUpdateRate);
    }

    void IMUCallback(SensorMsgsImuMsg msg)
    {
        imuData = msg;
    }

    void OdometryCallback(NavMsgsOdometryMsg msg)
    {
        odometryData = msg;
    }

    void GPSCallback(SensorMsgsNavSatFixMsg msg)
    {
        gpsData = msg;
    }

    void UpdateFusion()
    {
        if (Time.time - lastFusionUpdate >= 1.0f / fusionUpdateRate)
        {
            PerformSensorFusion();
            lastFusionUpdate = Time.time;
        }
    }

    void PerformSensorFusion()
    {
        // Simple sensor fusion algorithm
        // In practice, you'd use more sophisticated filtering (EKF, UKF, etc.)

        if (odometryData != null)
        {
            // Use odometry for position
            var pos = odometryData.pose.pose.position;
            fusedPosition = new Vector3((float)pos.x, (float)pos.z, (float)pos.y);
        }

        if (imuData != null)
        {
            // Use IMU for orientation
            var orient = imuData.orientation;
            fusedOrientation = new Quaternion((float)orient.x, (float)orient.z, (float)orient.y, (float)orient.w);
        }

        // Update robot transform in Unity
        if (robotTransform != null)
        {
            robotTransform.position = fusedPosition;
            robotTransform.rotation = fusedOrientation;
        }
    }
}
```

## Humanoid-Specific Sensor Integration

### Humanoid Joint State Processing

Processing humanoid robot joint states with proper kinematic chains:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class HumanoidJointProcessor : MonoBehaviour
{
    ROSConnection ros;
    public string jointStatesTopic = "/joint_states";

    [System.Serializable]
    public class HumanoidJointMapping
    {
        public string jointName; // ROS joint name
        public Transform jointTransform; // Unity transform
        public JointType jointType;
        public float minAngle = -90f;
        public float maxAngle = 90f;
        public Vector3 rotationAxis = Vector3.right; // Unity rotation axis
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }

    public List<HumanoidJointMapping> jointMappings = new List<HumanoidJointMapping>();

    // Humanoid-specific joint groups
    [Header("Humanoid Joint Groups")]
    public List<Transform> leftArmJoints = new List<Transform>();
    public List<Transform> rightArmJoints = new List<Transform>();
    public List<Transform> leftLegJoints = new List<Transform>();
    public List<Transform> rightLegJoints = new List<Transform>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorMsgsJointStateMsg>(jointStatesTopic, JointStateCallback);
    }

    void JointStateCallback(SensorMsgsJointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Array.Length; i++)
        {
            string jointName = msg.name.Array[i];
            double position = msg.position[i];

            var jointMapping = jointMappings.Find(j => j.jointName == jointName);
            if (jointMapping != null && jointMapping.jointTransform != null)
            {
                ApplyJointPosition(jointMapping, (float)position);
            }
        }
    }

    void ApplyJointPosition(HumanoidJointMapping jointMapping, float position)
    {
        switch (jointMapping.jointType)
        {
            case JointType.Revolute:
                // Apply rotational joint
                jointMapping.jointTransform.localRotation =
                    Quaternion.AngleAxis(Mathf.Rad2Deg * position, jointMapping.rotationAxis);
                break;

            case JointType.Prismatic:
                // Apply linear joint
                jointMapping.jointTransform.localPosition =
                    jointMapping.rotationAxis * (float)position;
                break;

            case JointType.Fixed:
                // No movement for fixed joints
                break;
        }
    }

    public void UpdateHumanoidPose()
    {
        // Additional processing for humanoid-specific pose updates
        // This could include inverse kinematics, balance control visualization, etc.
    }
}
```

## Sensor Data Validation and Filtering

### Data Quality Assessment

Validating sensor data quality before integration:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorDataValidator : MonoBehaviour
{
    [System.Serializable]
    public class SensorValidationParams
    {
        public string sensorName;
        public float maxFrequency = 100f; // Hz
        public float minFrequency = 5f;   // Hz
        public float maxValue = float.MaxValue;
        public float minValue = float.MinValue;
        public float maxRateOfChange = float.MaxValue;
    }

    public List<SensorValidationParams> validationParams = new List<SensorValidationParams>();
    private Dictionary<string, float> lastValues = new Dictionary<string, float>();
    private Dictionary<string, float> lastUpdateTimes = new Dictionary<string, float>();

    public bool ValidateData(string sensorName, float value)
    {
        var param = validationParams.Find(p => p.sensorName == sensorName);
        if (param == null)
            return true; // No validation rules, accept data

        // Check value bounds
        if (value < param.minValue || value > param.maxValue)
        {
            Debug.LogWarning($"Sensor {sensorName} value {value} out of bounds [{param.minValue}, {param.maxValue}]");
            return false;
        }

        // Check rate of change
        if (lastValues.ContainsKey(sensorName))
        {
            float deltaTime = Time.time - lastUpdateTimes[sensorName];
            float rateOfChange = Mathf.Abs(value - lastValues[sensorName]) / deltaTime;

            if (rateOfChange > param.maxRateOfChange)
            {
                Debug.LogWarning($"Sensor {sensorName} rate of change {rateOfChange} exceeds limit {param.maxRateOfChange}");
                return false;
            }
        }

        // Store current values for next validation
        lastValues[sensorName] = value;
        lastUpdateTimes[sensorName] = Time.time;

        return true;
    }
}
```

## Performance Optimization for Sensor Integration

### Efficient Data Processing

Optimizing sensor data processing for real-time performance:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class OptimizedSensorProcessor : MonoBehaviour
{
    [Header("Performance Settings")]
    public int maxSensorUpdatesPerFrame = 100;
    public float sensorUpdateInterval = 0.016f; // ~60Hz

    private Queue<System.Action> sensorUpdateQueue = new Queue<System.Action>();
    private float lastSensorUpdate = 0f;

    void Update()
    {
        ProcessSensorUpdates();
    }

    public void QueueSensorUpdate(System.Action updateAction)
    {
        sensorUpdateQueue.Enqueue(updateAction);
    }

    void ProcessSensorUpdates()
    {
        if (Time.time - lastSensorUpdate >= sensorUpdateInterval && sensorUpdateQueue.Count > 0)
        {
            int processed = 0;
            while (sensorUpdateQueue.Count > 0 && processed < maxSensorUpdatesPerFrame)
            {
                var updateAction = sensorUpdateQueue.Dequeue();
                updateAction?.Invoke();
                processed++;
            }

            lastSensorUpdate = Time.time;
        }
    }
}
```

## Debugging and Visualization Tools

### Sensor Data Debugging

Creating tools to visualize and debug sensor data:

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class SensorDebugger : MonoBehaviour
{
    [Header("Debug UI")]
    public Text debugText;
    public bool showDebugInfo = true;

    private Dictionary<string, List<float>> sensorHistory = new Dictionary<string, List<float>>();
    private int maxHistoryPoints = 100;

    public void LogSensorData(string sensorName, float value)
    {
        if (!sensorHistory.ContainsKey(sensorName))
        {
            sensorHistory[sensorName] = new List<float>();
        }

        sensorHistory[sensorName].Add(value);
        if (sensorHistory[sensorName].Count > maxHistoryPoints)
        {
            sensorHistory[sensorName].RemoveAt(0);
        }

        if (showDebugInfo && debugText != null)
        {
            UpdateDebugDisplay();
        }
    }

    void UpdateDebugDisplay()
    {
        string debugInfo = "Sensor Debug Info:\n";
        foreach (var kvp in sensorHistory)
        {
            float avg = kvp.Value.Count > 0 ? kvp.Value.Average() : 0;
            float min = kvp.Value.Count > 0 ? kvp.Value.Min() : 0;
            float max = kvp.Value.Count > 0 ? kvp.Value.Max() : 0;

            debugInfo += $"{kvp.Key}: Avg={avg:F3}, Min={min:F3}, Max={max:F3}\n";
        }

        debugText.text = debugInfo;
    }
}
```

This comprehensive sensor data integration approach ensures that your Unity-based digital twin accurately reflects the state of the physical robot through proper processing and visualization of sensor data streams.
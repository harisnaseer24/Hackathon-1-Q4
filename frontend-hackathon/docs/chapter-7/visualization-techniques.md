---
sidebar_position: 2
---

# Visualization Techniques for Digital Twins

## Advanced Rendering in Unity

### High-Quality Materials and Shaders

For digital twin applications, creating photorealistic materials is essential for accurate representation:

```csharp
using UnityEngine;

public class DigitalTwinMaterialController : MonoBehaviour
{
    public Material[] robotMaterials;
    public Renderer[] robotRenderers;

    [Header("Material Properties")]
    public Color baseColor = Color.gray;
    public float metallic = 0.1f;
    public float smoothness = 0.7f;

    void Start()
    {
        UpdateMaterials();
    }

    void UpdateMaterials()
    {
        foreach (var renderer in robotRenderers)
        {
            if (renderer != null && renderer.material != null)
            {
                var material = renderer.material;
                material.color = baseColor;
                material.SetFloat("_Metallic", metallic);
                material.SetFloat("_Smoothness", smoothness);
            }
        }
    }
}
```

### Realistic Lighting Setup

Configure lighting to match the physical environment:

```csharp
using UnityEngine;

public class DigitalTwinLighting : MonoBehaviour
{
    public Light mainLight;
    public Color ambientColor = new Color(0.2f, 0.2f, 0.2f, 1);
    public float intensityMultiplier = 1.0f;

    [Header("Environment Lighting")]
    public Cubemap reflectionCubemap;

    void Start()
    {
        SetupLighting();
    }

    void SetupLighting()
    {
        // Configure main directional light
        if (mainLight != null)
        {
            mainLight.intensity = 1.0f * intensityMultiplier;
            mainLight.color = Color.white;
        }

        // Set ambient lighting
        RenderSettings.ambientLight = ambientColor;
        RenderSettings.ambientIntensity = 1.0f;

        // Configure reflections if using HDRP/URP
        if (reflectionCubemap != null)
        {
            RenderSettings.customReflection = reflectionCubemap;
        }
    }
}
```

## Robot Visualization

### Joint Position Visualization

Visualize robot joint states from ROS 2 in real-time:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class JointVisualizer : MonoBehaviour
{
    ROSConnection ros;
    public string jointStatesTopic = "/joint_states";

    [System.Serializable]
    public class JointMapping
    {
        public string jointName;
        public ArticulationBody joint;
        [Range(-180, 180)]
        public float angleOffset = 0f;
    }

    public List<JointMapping> jointMappings = new List<JointMapping>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorMsgsJointStateMsg>(jointStatesTopic, JointStateCallback);
    }

    void JointStateCallback(SensorMsgsJointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Array.Length; i++)
        {
            string jointName = jointState.name.Array[i];
            float jointPosition = jointState.position[i];

            var jointMapping = jointMappings.Find(j => j.jointName == jointName);
            if (jointMapping != null && jointMapping.joint != null)
            {
                // Convert ROS joint position (radians) to Unity joint drive target
                float targetAngle = Mathf.Rad2Deg * jointPosition + jointMapping.angleOffset;

                ArticulationDrive drive = jointMapping.joint.jointDrive;
                drive.target = targetAngle;
                jointMapping.joint.jointDrive = drive;
            }
        }
    }
}
```

### Sensor Data Visualization

Visualize sensor data streams in Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class SensorVisualizer : MonoBehaviour
{
    ROSConnection ros;

    [Header("LIDAR Visualization")]
    public GameObject lidarPointPrefab;
    public Transform lidarOrigin;
    public int maxPoints = 1000;
    private List<GameObject> lidarPoints = new List<GameObject>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorMsgsLaserScanMsg>("/scan", LaserScanCallback);

        // Initialize LIDAR points
        InitializeLidarPoints();
    }

    void InitializeLidarPoints()
    {
        for (int i = 0; i < maxPoints; i++)
        {
            GameObject point = Instantiate(lidarPointPrefab, lidarOrigin);
            point.SetActive(false);
            lidarPoints.Add(point);
        }
    }

    void LaserScanCallback(SensorMsgsLaserScanMsg scan)
    {
        float angle = scan.angle_min;

        for (int i = 0; i < scan.ranges.Length && i < maxPoints; i++)
        {
            float range = scan.ranges[i];

            if (range >= scan.range_min && range <= scan.range_max)
            {
                Vector3 pointPos = new Vector3(
                    range * Mathf.Cos(angle),
                    0,
                    range * Mathf.Sin(angle)
                );

                lidarPoints[i].transform.position = lidarOrigin.position + pointPos;
                lidarPoints[i].SetActive(true);
            }
            else
            {
                lidarPoints[i].SetActive(false);
            }

            angle += scan.angle_increment;
        }
    }
}
```

## Camera and View Management

### Multiple Camera Views

Set up different camera views for comprehensive robot monitoring:

```csharp
using UnityEngine;

public class DigitalTwinCameraManager : MonoBehaviour
{
    public Camera mainCamera;
    public Camera[] robotCameras; // eg: head camera, gripper camera
    public Camera[] environmentCameras; // eg: overhead, side views

    [Header("Camera Switching")]
    public KeyCode mainCamKey = KeyCode.Alpha1;
    public KeyCode robotCamKey = KeyCode.Alpha2;
    public KeyCode envCamKey = KeyCode.Alpha3;

    private Camera currentCamera;

    void Start()
    {
        currentCamera = mainCamera;
        SetCameraActive(currentCamera, true);
    }

    void Update()
    {
        if (Input.GetKeyDown(mainCamKey))
            SwitchCamera(mainCamera);
        else if (Input.GetKeyDown(robotCamKey))
            SwitchCamera(robotCameras.Length > 0 ? robotCameras[0] : mainCamera);
        else if (Input.GetKeyDown(envCamKey))
            SwitchCamera(environmentCameras.Length > 0 ? environmentCameras[0] : mainCamera);
    }

    void SwitchCamera(Camera newCamera)
    {
        if (newCamera != null && newCamera != currentCamera)
        {
            SetCameraActive(currentCamera, false);
            currentCamera = newCamera;
            SetCameraActive(currentCamera, true);
        }
    }

    void SetCameraActive(Camera cam, bool active)
    {
        if (cam != null)
        {
            cam.gameObject.SetActive(active);
            if (active)
            {
                cam.enabled = true;
            }
        }
    }
}
```

### VR/AR Integration

For immersive digital twin experiences:

```csharp
using UnityEngine;
using UnityEngine.XR;

public class DigitalTwinVRManager : MonoBehaviour
{
    [Header("VR Settings")]
    public bool enableVR = true;
    public Transform robotOrigin;
    public float vrScale = 1.0f;

    void Start()
    {
        if (enableVR)
        {
            EnableVR();
        }
    }

    void EnableVR()
    {
        // Check if XR is available
        if (XRSettings.enabled)
        {
            XRSettings.enabled = true;
            // Configure VR-specific settings
            ConfigureVR();
        }
        else
        {
            Debug.LogWarning("VR not available on this platform");
        }
    }

    void ConfigureVR()
    {
        // Adjust scale for VR
        robotOrigin.localScale = Vector3.one * vrScale;

        // Configure VR-specific interactions
        // Add teleportation, interaction handlers, etc.
    }
}
```

## Data Overlay and UI

### Real-time Data Display

Create UI elements to display real-time robot data:

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class DigitalTwinUIData : MonoBehaviour
{
    ROSConnection ros;

    [Header("UI Elements")]
    public Text jointStateText;
    public Text sensorDataText;
    public Text statusText;

    [Header("Robot Data")]
    private SensorMsgsJointStateMsg lastJointState;
    private SensorMsgsImuMsg lastIMUData;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorMsgsJointStateMsg>("/joint_states", JointStateCallback);
        ros.Subscribe<SensorMsgsImuMsg>("/imu", IMUCallback);

        InvokeRepeating("UpdateUI", 0.0f, 0.1f); // Update UI every 100ms
    }

    void JointStateCallback(SensorMsgsJointStateMsg jointState)
    {
        lastJointState = jointState;
    }

    void IMUCallback(SensorMsgsImuMsg imuData)
    {
        lastIMUData = imuData;
    }

    void UpdateUI()
    {
        if (lastJointState != null)
        {
            string jointText = "Joint States:\n";
            for (int i = 0; i < Mathf.Min(5, lastJointState.name.Array.Length); i++)
            {
                jointText += $"{lastJointState.name.Array[i]}: {lastJointState.position[i]:F2} rad\n";
            }
            jointStateText.text = jointText;
        }

        if (lastIMUData != null)
        {
            sensorDataText.text = $"IMU: ({lastIMUData.linear_acceleration.x:F2}, " +
                                $"{lastIMUData.linear_acceleration.y:F2}, " +
                                $"{lastIMUData.linear_acceleration.z:F2})";
        }

        statusText.text = $"ROS Connected: {ros.IsConnected}";
    }
}
```

## Performance Optimization

### Level of Detail (LOD)

Implement LOD for complex robot models:

```csharp
using UnityEngine;

[RequireComponent(typeof(LODGroup))]
public class DigitalTwinLODController : MonoBehaviour
{
    private LODGroup lodGroup;
    private Renderer[] renderers;

    void Start()
    {
        lodGroup = GetComponent<LODGroup>();
        renderers = GetComponentsInChildren<Renderer>();

        SetupLODs();
    }

    void SetupLODs()
    {
        // Define LOD levels
        LOD[] lods = new LOD[3];

        // LOD 0: High detail (0-50m)
        Renderer[] highDetailRenderers = renderers; // All renderers
        lods[0] = new LOD(0.5f, highDetailRenderers);

        // LOD 1: Medium detail (50-100m)
        Renderer[] mediumDetailRenderers = GetMediumDetailRenderers();
        lods[1] = new LOD(0.25f, mediumDetailRenderers);

        // LOD 2: Low detail (100m+)
        Renderer[] lowDetailRenderers = GetLowDetailRenderers();
        lods[2] = new LOD(0.05f, lowDetailRenderers);

        lodGroup.SetLODS(lods);
        lodGroup.RecalculateBounds();
    }

    Renderer[] GetMediumDetailRenderers()
    {
        // Return only major components for medium detail
        return renderers;
    }

    Renderer[] GetLowDetailRenderers()
    {
        // Return only basic shape for low detail
        return renderers;
    }
}
```

### Occlusion Culling

For large environments, implement occlusion culling:

```csharp
using UnityEngine;

public class DigitalTwinOcclusion : MonoBehaviour
{
    public Camera mainCamera;
    private List<Renderer> visibleRenderers = new List<Renderer>();
    private List<Renderer> allRenderers;

    void Start()
    {
        // Get all renderers that should be considered for occlusion
        allRenderers = new List<Renderer>(FindObjectsOfType<Renderer>());
    }

    void Update()
    {
        UpdateVisibility();
    }

    void UpdateVisibility()
    {
        if (mainCamera != null)
        {
            foreach (var renderer in allRenderers)
            {
                if (renderer != null)
                {
                    // Check if renderer is visible to camera
                    bool isVisible = IsVisible(renderer.bounds, mainCamera);
                    renderer.enabled = isVisible;
                }
            }
        }
    }

    bool IsVisible(Bounds bounds, Camera camera)
    {
        var planes = GeometryUtility.CalculateFrustumPlanes(camera);
        return GeometryUtility.TestPlanesAABB(planes, bounds);
    }
}
```

## Advanced Visualization Techniques

### Particle Systems for Sensor Visualization

Use particle systems to visualize sensor data:

```csharp
using UnityEngine;

public class ParticleSensorVisualization : MonoBehaviour
{
    public ParticleSystem particleSystem;
    private ParticleSystem.MainModule mainModule;
    private ParticleSystem.EmissionModule emissionModule;

    void Start()
    {
        if (particleSystem == null)
            particleSystem = GetComponent<ParticleSystem>();

        mainModule = particleSystem.main;
        emissionModule = particleSystem.emission;
    }

    public void VisualizeSensorData(Vector3[] positions, Color[] colors)
    {
        // Configure particle system for sensor visualization
        emissionModule.rateOverTime = positions.Length;
        mainModule.startColor = Color.white;
        mainModule.startSize = 0.1f;

        // Set particle positions (simplified - actual implementation may vary)
        // This would typically require custom particle manipulation
    }
}
```

These visualization techniques enable rich, interactive digital twin experiences that accurately represent the physical robot's state and environment in real-time.
---
title: Gazebo & Unity Examples
sidebar_position: 3
description: Practical examples and exercises for Gazebo and Unity simulation
keywords: [gazebo, unity, examples, simulation, robotics, physical ai]
---

import Exercise from '@site/src/components/Exercise/Exercise';

# Gazebo & Unity Examples

## Example 1: Basic Robot Simulation in Gazebo

This example demonstrates creating a simple differential drive robot in Gazebo.

### Robot Model Definition (SDF)

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="diff_drive_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx> <ixy>0</ixy> <ixz>0</ixz>
          <iyy>0.1</iyy> <iyz>0</iyz> <izz>0.1</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>0.5 0.3 0.2</size></box>
        </geometry>
      </visual>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <link name="left_wheel">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.01</ixx> <ixy>0</ixy> <ixz>0</ixz>
          <iyy>0.01</iyy> <iyz>0</iyz> <izz>0.01</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
        </geometry>
      </visual>
    </link>

    <!-- Right wheel similar to left -->
  </model>
</sdf>
```

### ROS2 Controller Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.left_wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/left_wheel_controller/commands',
            10)

        self.right_wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/right_wheel_controller/commands',
            10)

        self.wheel_base = 0.3  # Distance between wheels
        self.wheel_radius = 0.1  # Wheel radius

    def cmd_vel_callback(self, msg):
        # Convert linear and angular velocity to wheel velocities
        left_vel = (msg.linear.x - msg.angular.z * self.wheel_base / 2.0) / self.wheel_radius
        right_vel = (msg.linear.x + msg.angular.z * self.wheel_base / 2.0) / self.wheel_radius

        # Publish wheel velocities
        left_msg = Float64MultiArray()
        left_msg.data = [left_vel]
        self.left_wheel_pub.publish(left_msg)

        right_msg = Float64MultiArray()
        right_msg.data = [right_vel]
        self.right_wheel_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = DiffDriveController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 1: Differential Drive Enhancement

<Exercise
  title="Enhanced Differential Drive Robot"
  difficulty="intermediate"
  type="practical"
  instructions="Extend the basic differential drive robot by adding a camera sensor, implementing proper joint controllers, and creating a navigation stack that works in the simulation."
  expectedOutcome="Students will understand how to create complete robot models with sensors and controllers in Gazebo."
/>

## Example 2: Unity Robotics Simulation

This example demonstrates setting up a basic robot in Unity with physics and sensor simulation.

### Robot Controller Script

```csharp
using UnityEngine;
using System.Collections;

public class UnityRobotController : MonoBehaviour
{
    public float linearSpeed = 2.0f;
    public float angularSpeed = 50.0f;
    public Transform leftWheel;
    public Transform rightWheel;
    public float wheelRadius = 0.1f;

    private float leftWheelVelocity = 0f;
    private float rightWheelVelocity = 0f;

    void Update()
    {
        // Simulate ROS cmd_vel input (in a real implementation, this would come from ROS)
        float linearVel = Input.GetAxis("Vertical") * linearSpeed;
        float angularVel = Input.GetAxis("Horizontal") * angularSpeed;

        // Convert to wheel velocities
        float leftVel = (linearVel - angularVel * 0.15f) / wheelRadius;  // 0.15 is half the wheelbase
        float rightVel = (linearVel + angularVel * 0.15f) / wheelRadius;

        // Apply wheel rotations
        leftWheel.Rotate(Vector3.right, leftVel * Time.deltaTime);
        rightWheel.Rotate(Vector3.right, rightVel * Time.deltaTime);

        // Update robot position based on wheel velocities
        float avgVel = (leftVel + rightVel) * wheelRadius / 2.0f;
        float angVel = (rightVel - leftVel) * wheelRadius / 0.3f;  // 0.3 is wheelbase

        transform.Translate(Vector3.forward * avgVel * Time.deltaTime);
        transform.Rotate(Vector3.up, angVel * Time.deltaTime);
    }
}
```

### Camera Sensor Script

```csharp
using UnityEngine;
using System.Collections;

public class UnityCameraSensor : MonoBehaviour
{
    public Camera cam;
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float updateRate = 30.0f;  // Hz

    private RenderTexture renderTexture;
    private float nextUpdateTime = 0.0f;

    void Start()
    {
        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        cam.targetTexture = renderTexture;

        // Initialize camera parameters
        cam.orthographic = false;  // Perspective camera
    }

    void Update()
    {
        if (Time.time >= nextUpdateTime)
        {
            // Capture image and process
            ProcessImage();
            nextUpdateTime = Time.time + 1.0f / updateRate;
        }
    }

    void ProcessImage()
    {
        // Activate render texture
        RenderTexture.active = renderTexture;

        // Render the camera
        cam.Render();

        // Create texture to read from
        Texture2D imageTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        // Read pixels from render texture
        imageTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        imageTexture.Apply();

        // Convert to byte array or process as needed
        byte[] imageData = imageTexture.EncodeToPNG();

        // In a real implementation, send this data to ROS or process it
        Debug.Log("Camera image captured: " + imageData.Length + " bytes");

        // Clean up
        Destroy(imageTexture);
    }
}
```

### Exercise 2: Unity Navigation System

<Exercise
  title="Unity Navigation with Path Planning"
  difficulty="advanced"
  type="practical"
  instructions="Create a Unity scene with a robot that can navigate to waypoints using A* pathfinding. Implement obstacle avoidance and path following algorithms."
  expectedOutcome="Students will understand how to implement navigation systems in Unity and integrate path planning algorithms."
/>

## Example 3: Unity ML-Agents Integration

This example demonstrates using Unity ML-Agents for robot training.

### Robot Academy Script

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class RobotAcademy : Agent
{
    [Header("Robot Specific")]
    public Transform target;
    public float moveSpeed = 1.0f;
    public float rotationSpeed = 100.0f;

    private Rigidbody m_Rb;
    private Vector3 m_StartingPos;

    public override void Initialize()
    {
        m_Rb = GetComponent<Rigidbody>();
        m_StartingPos = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        // Reset agent position
        transform.position = m_StartingPos;
        transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);

        // Randomize target position
        target.position = new Vector3(
            Random.Range(-8f, 8f),
            0.5f,
            Random.Range(-8f, 8f)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent position and rotation
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.rotation.eulerAngles);

        // Target position relative to agent
        sensor.AddObservation(target.position - transform.position);

        // Distance to target
        sensor.AddObservation(Vector3.Distance(transform.position, target.position));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions: [0] = move forward/back, [1] = rotate left/right
        float forward = actionBuffers.ContinuousActions[0];
        float rotate = actionBuffers.ContinuousActions[1];

        // Apply movement
        transform.Translate(Vector3.forward * forward * moveSpeed * Time.deltaTime);
        transform.Rotate(Vector3.up, rotate * rotationSpeed * Time.deltaTime);

        // Apply penalty for each step to encourage efficiency
        SetReward(-0.01f);

        // Check if reached target
        if (Vector3.Distance(transform.position, target.position) < 1.5f)
        {
            SetReward(10.0f);
            EndEpisode();
        }

        // Check if agent is too far from start
        if (Vector3.Distance(transform.position, m_StartingPos) > 20f)
        {
            SetReward(-5.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");  // Forward/back
        continuousActionsOut[1] = Input.GetAxis("Horizontal");  // Rotate
    }
}
```

### Exercise 3: Reinforcement Learning Navigation

<Exercise
  title="RL-Based Navigation Training"
  difficulty="advanced"
  type="simulation"
  instructions="Train a robot to navigate through a complex environment using reinforcement learning in Unity ML-Agents. Implement curriculum learning to gradually increase difficulty."
  expectedOutcome="Students will understand how to train robots using reinforcement learning and implement curriculum learning strategies."
/>

## Example 4: Gazebo-ROS Integration

This example shows how to integrate Gazebo with ROS for more complex simulations.

### Launch File

```xml
<launch>
  <!-- Start Gazebo with a world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find my_robot_gazebo)/worlds/simple_room.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-file $(find my_robot_description)/urdf/my_robot.urdf
              -urdf -model my_robot
              -x 0 -y 0 -z 0.1" />

  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher"
        type="robot_state_publisher" />

  <!-- Joint state publisher -->
  <node name="joint_state_publisher" pkg="joint_state_publisher"
        type="joint_state_publisher" />
</launch>
```

### Controller Configuration

```yaml
# my_robot_controllers.yaml
diff_drive_controller:
  type: diff_drive_controller/DiffDriveController
  left_wheel: ['left_wheel_joint']
  right_wheel: ['right_wheel_joint']
  publish_rate: 50
  pose_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
  twist_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
  cmd_vel_timeout: 0.25
  velocity_rolling_window_size: 2
  linear:
    x:
      has_velocity_limits: true
      max_velocity: 2.0
      has_acceleration_limits: true
      max_acceleration: 1.0
  angular:
    z:
      has_velocity_limits: true
      max_velocity: 4.0
      has_acceleration_limits: true
      max_acceleration: 2.0
```

### Exercise 4: Complex Multi-Robot Simulation

<Exercise
  title="Multi-Robot Coordination in Gazebo"
  difficulty="advanced"
  type="practical"
  instructions="Create a simulation with multiple robots that must coordinate to complete a task. Implement communication protocols and collision avoidance."
  expectedOutcome="Students will understand how to simulate multi-robot systems and implement coordination algorithms."
/>

## Example 5: Unity-ROS Bridge

This example demonstrates connecting Unity to ROS using the unity-ros-bridge.

### Unity ROS Connector

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class UnityRosConnector : MonoBehaviour
{
    public string rosBridgeServerUrl = "ws://192.168.1.1:9090";
    private RosSocket rosSocket;

    void Start()
    {
        // Connect to ROS bridge
        rosSocket = new RosSocket(new WebSocketNetWebSocket(rosBridgeServerUrl));

        // Subscribe to topics
        rosSocket.Subscribe<Unity.Robotics.ROSTCPConnector.Messages.Std_msgs.String>(
            "unity_to_ros",
            ReceiveMessageFromUnity
        );

        // Publish to topics
        PublishToRos();
    }

    void ReceiveMessageFromUnity(
        Unity.Robotics.ROSTCPConnector.Messages.Std_msgs.String message)
    {
        Debug.Log("Received from ROS: " + message.data);
    }

    void PublishToRos()
    {
        var message = new Unity.Robotics.ROSTCPConnector.Messages.Std_msgs.String();
        message.data = "Hello from Unity!";

        rosSocket.Publish("unity_to_ros", message);
    }
}
```

### Exercise 5: Unity-ROS Integration

<Exercise
  title="Unity-ROS Communication System"
  difficulty="advanced"
  type="practical"
  instructions="Create a system that allows real-time communication between Unity simulation and ROS. Implement sensor data publishing and command subscription."
  expectedOutcome="Students will understand how to integrate Unity with ROS for hybrid simulation systems."
/>

## Exercise 6: Physics Parameter Tuning

<Exercise
  title="Simulation Fidelity Optimization"
  difficulty="intermediate"
  type="practical"
  instructions="Tune physics parameters in both Gazebo and Unity to match real-world robot behavior. Compare simulation results with physical robot data."
  expectedOutcome="Students will understand how to optimize simulation fidelity and minimize the reality gap."
/>

## Exercise 7: Sensor Noise Modeling

<Exercise
  title="Realistic Sensor Simulation"
  difficulty="intermediate"
  type="practical"
  instructions="Implement realistic noise models for camera, LIDAR, and IMU sensors in both simulation environments. Validate the noise characteristics."
  expectedOutcome="Students will understand how to create realistic sensor simulations with proper noise modeling."
/>

## Exercise 8: Domain Randomization Implementation

<Exercise
  title="Domain Randomization for Robust Training"
  difficulty="advanced"
  type="simulation"
  instructions="Implement domain randomization techniques in Unity ML-Agents to improve sim-to-real transfer. Test the robustness of trained policies."
  expectedOutcome="Students will understand domain randomization techniques and their impact on sim-to-real transfer."
/>

## Exercise 9: High-Fidelity Environment Creation

<Exercise
  title="Complex Environment Modeling"
  difficulty="intermediate"
  type="practical"
  instructions="Create a complex simulation environment with realistic physics, lighting, and interactive objects in both Gazebo and Unity."
  expectedOutcome="Students will understand how to create complex, realistic simulation environments."
/>

## Exercise 10: Performance Optimization

<Exercise
  title="Simulation Performance Tuning"
  difficulty="advanced"
  type="practical"
  instructions="Optimize simulation performance while maintaining required fidelity. Implement level-of-detail systems and parallel processing."
  expectedOutcome="Students will understand performance optimization techniques for robotics simulations."
/>

## Summary

These examples demonstrate the practical implementation of simulation techniques in both Gazebo and Unity environments. Students should practice implementing these patterns and extending them to create more complex robotics simulations.
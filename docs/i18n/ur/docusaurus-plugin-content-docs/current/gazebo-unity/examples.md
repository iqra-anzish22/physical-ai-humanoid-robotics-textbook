---
title: گزیبو اور یونٹی کی مثالیں
sidebar_position: 3
description: گزیبو اور یونٹی سیمولیشن کے عملی مثالیں اور مشقیں
keywords: [gazebo, unity, examples, simulation, robotics, physical ai]
---

import Exercise from '@site/src/components/Exercise/Exercise';

# گزیبو اور یونٹی کی مثالیں

## مثال 1: گزیبو میں بنیادی روبوٹ سیمولیشن

یہ مثال گزیبو میں ایک سادہ ڈفرینشل ڈرائیو روبوٹ تخلیق کرنا دکھاتی ہے۔

### روبوٹ ماڈل کی تعریف (SDF)

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

### ROS2 کنٹرولر نوڈ

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

### مشق 1: ڈفرینشل ڈرائیو کی توسیع

<Exercise
  title="اعلی درجے کا ڈفرینشل ڈرائیو روبوٹ"
  difficulty="intermediate"
  type="practical"
  instructions="بنیادی ڈفرینشل ڈرائیو روبوٹ کو کیمرہ سینسر شامل کر کے، مناسب جوائنٹ کنٹرولر نافذ کر کے، اور ایک نیوی گیشن اسٹیک تخلیق کر کے جو سیمولیشن میں کام کرے، توسیع دیں۔"
  expectedOutcome="طلباء گزیبو میں سینسرز اور کنٹرولرز کے ساتھ مکمل روبوٹ ماڈلز کیسے تخلیق کریں یہ سمجھیں گے۔"
/>

## مثال 2: یونٹی روبوٹکس سیمولیشن

یہ مثال یونٹی میں ایک بنیادی روبوٹ کو فزکس اور سینسر سیمولیشن کے ساتھ سیٹ اپ کرنا دکھاتی ہے۔

### روبوٹ کنٹرولر اسکرپٹ

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

### کیمرہ سینسر اسکرپٹ

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

### مشق 2: یونٹی نیوی گیشن سسٹم

<Exercise
  title="A* راستہ تلاش کے ساتھ یونٹی نیوی گیشن"
  difficulty="advanced"
  type="practical"
  instructions="ایک یونٹی منظر تخلیق کریں جس میں ایک روبوٹ ہو جو A* راستہ تلاش کا استعمال کرتے ہوئے ویزی ویل پوائنٹس تک پہنچ سکے۔ رکاوٹ سے بچاؤ اور راستہ کے اتباع کے الگورتھم نافذ کریں۔"
  expectedOutcome="طلباء یونٹی میں نیوی گیشن سسٹم کیسے نافذ کریں اور راستہ تلاش کے الگورتھم کو ضم کریں یہ سمجھیں گے۔"
/>

## مثال 3: یونٹی ML-ایجنٹس انضمام

یہ مثال یونٹی ML-ایجنٹس کا استعمال کرتے ہوئے روبوٹ تربیت کا مظاہرہ کرتی ہے۔

### روبوٹ اکیڈمی اسکرپٹ

```csharp
using Unity.MLagents;
using Unity.MLagents.Sensors;
using Unity.MLagents.Actuators;
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

### مشق 3: رین فورسمنٹ لرننگ نیوی گیشن

<Exercise
  title="RL-بیسڈ نیوی گیشن تربیت"
  difficulty="advanced"
  type="simulation"
  instructions="یونٹی ML-ایجنٹس میں رین فورسمنٹ لرننگ کا استعمال کرتے ہوئے ایک پیچیدہ ماحول کے ذریعے ایک روبوٹ کو نیوی گیٹ کرنے کی تربیت دیں۔ کوریکولم لرننگ نافذ کر کے مشکل کو تدریجی طور پر بڑھائیں۔"
  expectedOutcome="طلباء رین فورسمنٹ لرننگ کا استعمال کرتے ہوئے روبوٹس کو تربیت دینا اور کوریکولم لرننگ کی حکمت عملیاں نافذ کرنا سمجھیں گے۔"
/>

## مثال 4: گزیبو-ROS انضمام

یہ مثال دکھاتی ہے کہ پیچیدہ سیمولیشن کے لیے گزیبو کو ROS کے ساتھ کیسے ضم کرنا ہے۔

### لانچ فائل

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

### کنٹرولر کنفیگریشن

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

### مشق 4: پیچیدہ ملٹی-روبوٹ سیمولیشن

<Exercise
  title="گزیبو میں ملٹی-روبوٹ کوآرڈی نیشن"
  difficulty="advanced"
  type="practical"
  instructions="ایک سیمولیشن تخلیق کریں جس میں متعدد روبوٹس ہوں جنہیں کام مکمل کرنے کے لیے کوآرڈینیٹ کرنا ہوگا۔ مواصلاتی پروٹوکولز اور کالیژن ایوائڈنس نافذ کریں۔"
  expectedOutcome="طلباء ملٹی-روبوٹ سسٹم کی تشریح کرنا اور کوآرڈی نیشن الگورتھم نافذ کرنا سمجھیں گے۔"
/>

## مثال 5: یونٹی-ROS برج

یہ مثال unity-ros-bridge کا استعمال کرتے ہوئے یونٹی کو ROS سے منسلک کرنا دکھاتی ہے۔

### یونٹی ROS کنیکٹر

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

### مشق 5: یونٹی-ROS انضمام

<Exercise
  title="یونٹی-ROS مواصلاتی سسٹم"
  difficulty="advanced"
  type="practical"
  instructions="ایک سسٹم تخلیق کریں جو یونٹی سیمولیشن اور ROS کے درمیان ریل ٹائم مواصلات کی اجازت دیتا ہے۔ سینسر ڈیٹا پبلشنگ اور کمانڈ سبسکرپشن نافذ کریں۔"
  expectedOutcome="طلباء ہائبرڈ سیمولیشن سسٹم کے لیے یونٹی کو ROS کے ساتھ ضم کرنا سمجھیں گے۔"
/>

## مشق 6: فزکس پیرامیٹر ٹیوننگ

<Exercise
  title="سیمولیشن وفاداری کی اصلاح"
  difficulty="intermediate"
  type="practical"
  instructions="حقیقی دنیا کے روبوٹ کے رویے سے مماثلت رکھنے کے لیے گزیبو اور یونٹی دونوں میں فزکس پیرامیٹر ٹیون کریں۔ سیمولیشن کے نتائج کا حقیقی روبوٹ ڈیٹا کے ساتھ موازنہ کریں۔"
  expectedOutcome="طلباء سیمولیشن وفاداری کو اصلاح کرنا اور ریئلٹی گیپ کو کم سے کم کرنا سمجھیں گے۔"
/>

## مشق 7: سینسر نوائز ماڈلنگ

<Exercise
  title="حقیقی سینسر سیمولیشن"
  difficulty="intermediate"
  type="practical"
  instructions="کیمرہ، لیڈار، اور IMU سینسرز کے لیے دونوں سیمولیشن ماحول میں حقیقی نوائز ماڈلز نافذ کریں۔ نوائز کے خصائص کی توثیق کریں۔"
  expectedOutcome="طلباء مناسب نوائز ماڈلنگ کے ساتھ حقیقی سینسر سیمولیشن کیسے تخلیق کریں یہ سمجھیں گے۔"
/>

## مشق 8: ڈومین رینڈمائزیشن نافذ کاری

<Exercise
  title="مضبوط تربیت کے لیے ڈومین رینڈمائزیشن"
  difficulty="advanced"
  type="simulation"
  instructions="sim-to-real ٹرانسفر کو بہتر بنانے کے لیے یونٹی ML-ایجنٹس میں ڈومین رینڈمائزیشن تکنیکیں نافذ کریں۔ تربیت یافتہ پالیسیز کی مضبوطی کو ٹیسٹ کریں۔"
  expectedOutcome="طلباء ڈومین رینڈمائزیشن تکنیکیں اور ان کے اثرات sim-to-real ٹرانسفر پر سمجھیں گے۔"
/>

## مشق 9: ہائی-فیدلٹی ماحول کی تخلیق

<Exercise
  title="پیچیدہ ماحول ماڈلنگ"
  difficulty="intermediate"
  type="practical"
  instructions="حقیقی فزکس، لائٹنگ، اور انٹرایکٹو اشیاء کے ساتھ گزیبو اور یونٹی دونوں میں ایک پیچیدہ سیمولیشن ماحول تخلیق کریں۔"
  expectedOutcome="طلباء پیچیدہ، حقیقی سیمولیشن ماحول کیسے تخلیق کریں یہ سمجھیں گے۔"
/>

## مشق 10: کارکردگی کی اصلاح

<Exercise
  title="سیمولیشن کارکردگی ٹیوننگ"
  difficulty="advanced"
  type="practical"
  instructions="ضرورت کی وفاداری کو برقرار رکھتے ہوئے سیمولیشن کارکردگی کو اصلاح کریں۔ لیول-آف-ڈیٹیل سسٹم اور پیرالل پروسیسنگ نافذ کریں۔"
  expectedOutcome="طلباء روبوٹکس سیمولیشنز کے لیے کارکردگی اصلاح کی تکنیکیں سمجھیں گے۔"
/>

## خلاصہ

یہ مثالیں دونوں گزیبو اور یونٹی ماحول میں سیمولیشن تکنیکوں کی عملی نافذ کاری کا مظاہرہ کرتی ہیں۔ طلباء کو ان پیٹرنز کو نافذ کرنا چاہیے اور انہیں زیادہ پیچیدہ روبوٹکس سیمولیشنز تخلیق کرنے کے لیے توسیع دینی چاہیے۔